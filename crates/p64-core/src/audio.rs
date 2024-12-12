/*
    The AI (Audio Interface) is one of the I/O interfaces of the RCP which playbacks audio samples
    It fetches samples via DMA (Direct Memory Access) from the RDRAM (Rambus Dynamic Random Access Memory)
    at a specific rate and outputs the audio samples.

    It does not perform any processing on the audio samples (Decompression, Mixing...), that is performed
    by the CPU or RSP (Reality Signal Processor)

    For playback through a DMA channel, the CPU prepares buffers of samples in the RDRAM, subsequently
    writing them to the AI's registers to setup a dma transfer

    Like the RSP and the RDP DMAs the AI DMAs have a double-buffering mechanism, allowing 
    to enqueue a second buffer while the first one is playing back and thus having continuous 
    playback; As soon as a buffer is finished, a second is ready for playback so that 
    the audio is uninterrupted.  

    The AI does not have internal RAM for storing samples as it is connected to the DAC (Digital to Analog Converter)
    which is responsible for converting digital audio signals into analog audio signals which are outputted
    through speakers or headphones.

    https://www.copetti.org/images/consoles/nintendo64/Audio.07d6a36f3757e76528002b27dba5152ccfd0d24a2aa10fa6e0c1b563597503ea.png
        Process of ROM to Audio DAC
        Copetti: http://www.copetti.org/

    The sampling rate can be up to 44.1 kHz, but using the top rate will steal lots of CPU cycles.
    There’s no strict limit on the number of channels, it all depends on how much the RSP is capable of mixing (often around 16-24 channels if processing ADPCM or ~100 if PCM).
    Memory is another concern, while competitors relied on larger mediums (i.e. CD-ROM) and dedicated audio memory, Nintendo 64 cartridges hold much less data (let alone music data) and have to share main memory with other components.

    For those reasons, players may notice that N64 ports contain lesser-quality music or repeated scores. Although, a common workaround is to implement a music sequencer that ‘constructs’ samples at runtime using a pre-populated set of sounds (similar to MIDI music).

    Works Cited:
        - https://n64brew.dev/wiki/Audio_Interface
        - https://www.copetti.org/writings/consoles/nintendo-64/

*/

use p64_gui::audio;
use crate::events::Event;
use super::builder::registers::Registers;
use super::mmu::MmuRegister;
use super::events::create_event;
use super::events::get_event;
use super::events::EventType;
use super::memory;
use super::memory::AccessSize;
use super::mips::clear_rcp_interrupt;
use super::mips::schedule_rcp_interrupt;
use super::mips::MI_INTR_AI;
use super::rdram::RDRAM_MASK;
use super::N64;

/// Audio Interface Register Indices
#[repr(u32)]
pub enum AiRegister {
    DramAddr = 0,
    Len = 1,
    Status = 3,
    DacRate = 4,
    RegsCount = 6,
}

        

/// Audio Interface Status Flags
#[repr(u32)]
pub enum AiStatus {
    Busy = 0x4000_0000,
    Full = 0x8000_0000,
}

pub struct Audio {
    pub registers: Registers,
    pub fifo: [AiDma; 2],
    pub last_read: u64,
    pub delayed_carry: bool,
}

impl Default for Audio {
    fn default() -> Self {
        Self {
            registers: Registers::new()
                .with_size(AiRegister::RegsCount as usize)
                .build(),
            fifo: [AiDma::default(); 2],
            last_read: 0,
            delayed_carry: false,
        }
    }
}

#[derive(Copy, Clone, Default)]
pub struct AiDma {
    pub address: u64,
    pub length: u64,
    pub duration: u64,
}

impl Audio {
    fn is_busy(&self) -> bool { (self.registers[AiRegister::Status as usize] & AiStatus::Busy as u32) != 0 }
    fn is_full(&self) -> bool { (self.registers[AiRegister::Status as usize] & AiStatus::Full as u32) != 0 }
    fn set_status_flag(&mut self, flag: AiStatus) { self.registers[AiRegister::Status as usize] |= flag as u32; }
    fn clear_status_flag(&mut self, flag: AiStatus) { self.registers[AiRegister::Status as usize] &= !(flag as u32); }

    fn update_fifo_entry(&mut self, index: usize, duration: u64, rdram_mask: u64) {
        self.fifo[index].address = self.registers[AiRegister::DramAddr as usize] as u64 & rdram_mask;
        self.fifo[index].length = (self.registers[AiRegister::Len as usize] & !7) as u64;
        self.fifo[index].duration = duration;
    }

    fn shift_fifo(&mut self) { self.fifo[0] = self.fifo[1]; }

    fn get_aligned_dma_length(&self) -> u64 {
        const ALIGNMENT_MASK: u64 = !7;
        self.registers[AiRegister::Len as usize] as u64 & ALIGNMENT_MASK
    }

    fn calculate_sample_rate(&self, vi_clock: u64) -> u64 {
        let dac_rate = self.registers[AiRegister::DacRate as usize] as u64;
        vi_clock / (1 + dac_rate)
    }

    fn calculate_dma_duration(&self, vi_clock: u64, cpu_clock: u64) -> u64 {
        const BYTES_PER_SAMPLE: u64 = 4;

        let dma_length = self.get_aligned_dma_length();
        let sample_rate = self.calculate_sample_rate(vi_clock);
        
        dma_length * cpu_clock / (BYTES_PER_SAMPLE * sample_rate)
    }

    fn calculate_remaining_fifo_length(&self, fifo_index: usize, cpu_count: u64, next_event: Option<Event>) -> u64 {
        let fifo = &self.fifo[fifo_index];
        
        if fifo.duration == 0 || next_event.is_none() { return 0; }

        let time_remaining = next_event.unwrap().count.saturating_sub(cpu_count);
        let bytes_remaining = time_remaining * fifo.length / fifo.duration;
        
        bytes_remaining & !7 // 8-byte alignment
    }

    fn process_dma_transfer(&mut self) {
        const ADDRESS_MASK: u64 = 0x1FFF;
        const ADDRESS_INCREMENT: u64 = 0x2000;

        self.last_read = self.fifo[0].length;

        if self.delayed_carry {
            self.fifo[0].address += ADDRESS_INCREMENT;
        }

        let next_address = self.fifo[0].address + self.fifo[0].length;
        self.delayed_carry = (next_address & ADDRESS_MASK) == 0;
    }

    fn push_to_fifo(&mut self, duration: u64, rdram_mask: u64) -> usize {
        let fifo_index = if self.is_busy() { 1 } else { 0 };
        self.update_fifo_entry(fifo_index, duration, rdram_mask);

        if fifo_index == 1 {
            self.set_status_flag(AiStatus::Full);
        } else {
            self.set_status_flag(AiStatus::Busy);
        }
        
        fifo_index
    }

    fn pop_from_fifo(&mut self) -> bool {
        if self.is_full() {
            self.shift_fifo();
            self.clear_status_flag(AiStatus::Full);
            true
        } else {
            self.clear_status_flag(AiStatus::Busy);
            self.delayed_carry = false;
            false
        }
    }

    fn read_register(&self, index: usize) -> u32 {
        self.registers[index]
    }

    fn write_register(&mut self, index: usize, value: u32, mask: u32) {
        memory::masked_write_32(&mut self.registers[index], value, mask);
    }

    fn play_audio_data(&mut self, current_length: u64) -> Option<(usize, u64)> {
        if current_length >= self.last_read {
            return None;
        }

        let processed_bytes = self.fifo[0].length - self.last_read;
        let bytes_to_play = self.last_read - current_length;
        let play_address = (self.fifo[0].address + processed_bytes) as usize;

        self.last_read = current_length;
        Some((play_address, bytes_to_play))
    }

    fn handle_dma_event(n64: &mut N64) {
        if let Some((addr, len)) = n64.audio.play_audio_data(0) {
            play_audio(n64, addr, len);
        }
        Audio::complete_audio_dma(n64);
    }

    fn calculate_audio_dma_duration(n64: &N64) -> u64 {
        n64.audio.calculate_dma_duration(n64.video.clock, n64.cpu.clock_rate)
    }
    
    fn start_audio_dma(n64: &mut N64) {
        n64.audio.process_dma_transfer();
    
        let current_count = n64.cpu.mmu.regs[MmuRegister::Count as usize];
        let completion_time = current_count + n64.audio.fifo[0].duration;
        
        create_event(n64, EventType::AI, completion_time, Audio::handle_dma_event);
        schedule_rcp_interrupt(n64, MI_INTR_AI);
    }
    
    fn queue_audio_data(n64: &mut N64) {
        let duration = Audio::calculate_audio_dma_duration(n64);
        let fifo_index = n64.audio.push_to_fifo(duration, RDRAM_MASK as u64);
        
        if fifo_index == 0 { Audio::start_audio_dma(n64); }
    }
    
    fn complete_audio_dma(n64: &mut N64) { if n64.audio.pop_from_fifo() { Audio::start_audio_dma(n64) } }
}

pub fn handle_ai_register_read(n64: &mut N64,address: u64, _access_size: AccessSize) -> u32 {
    let reg_index = (address & 0xFFFF) >> 2;
    
    if reg_index as u32 != AiRegister::Len as u32 { return n64.audio.read_register(reg_index as usize) }

    let cpu_count = n64.cpu.mmu.regs[MmuRegister::Count as usize];
    let next_event = get_event(n64, EventType::AI).copied();
    
    // Calculate DMA length
    let current_dma_length = n64.audio.calculate_remaining_fifo_length(0, cpu_count, next_event);

    // Handle audio playback
    if let Some((addr, len)) = n64.audio.play_audio_data(current_dma_length) { play_audio(n64, addr, len) }

    current_dma_length as u32
}

pub fn handle_ai_register_write(n64: &mut N64,address: u64, value: u32, mask: u32) {
    let reg_index = (address & 0xFFFF) >> 2;
    
    match reg_index as u32 {
        x if x == AiRegister::Len as u32 => {
            n64.audio.write_register(reg_index as usize, value, mask);
            if n64.audio.read_register(AiRegister::Len as usize) != 0 { Audio::queue_audio_data(n64) }
        },
        x if x == AiRegister::Status as u32 => clear_rcp_interrupt(n64, MI_INTR_AI),
        x if x == AiRegister::DacRate as u32 => {
            if n64.audio.read_register(reg_index as usize) != value & mask {
                let frequency = n64.video.clock / (1 + (value & mask)) as u64;
                audio::set_frequency(&mut n64.ui, frequency);
            }
            n64.audio.write_register(reg_index as usize, value, mask);
        },
        _ => n64.audio.write_register(reg_index as usize, value, mask),
    }
}

pub fn play_audio(device: &mut N64, dram_addr: usize, length: u64) {
    let audio_device = device.ui.window_audio.audio_device.as_mut().unwrap();
    let length = length as usize / 2;
    let mut primary_buffer: Vec<i16> = vec![0; length];

    for (i, chunk) in primary_buffer.chunks_mut(2).enumerate() {
        let base_addr = dram_addr + i * 4;
        chunk[0] = device.rdram.mem[base_addr + 2] as i16 | (device.rdram.mem[base_addr + 3] as i16) << 8;
        chunk[1] = device.rdram.mem[base_addr] as i16 | (device.rdram.mem[base_addr + 1] as i16) << 8;
    }

    let audio_queued = audio_device.size() as f64;
    let spec_freq = audio_device.spec().freq as f64;
    let acceptable_latency = spec_freq * 0.2 * 4.0;
    let min_latency = spec_freq * 0.02 * 4.0;

    if audio_queued < min_latency {
        let silence_buffer: Vec<i16> = vec![0; ((min_latency - audio_queued) * 2.0) as usize & !1];
        audio_device.queue_audio(&silence_buffer).unwrap();
    }

    if audio_queued < acceptable_latency {
        audio_device.queue_audio(&primary_buffer).unwrap();
    }
}