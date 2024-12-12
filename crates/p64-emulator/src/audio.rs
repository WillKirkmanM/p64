use p64_core::N64;

pub fn play_audio(device: &mut N64, dram_addr: usize, length: u64) {
    let audio_device = device.ui.window_audio.audio_device.as_mut().unwrap();
    let length = length as usize / 2;
    let mut primary_buffer: Vec<i16> = vec![0; length];

    for (i, chunk) in primary_buffer.chunks_mut(2).enumerate() {
        let base_addr = dram_addr + i * 4;
        chunk[0] =
            device.rdram.mem[base_addr + 2] as i16 | (device.rdram.mem[base_addr + 3] as i16) << 8;
        chunk[1] =
            device.rdram.mem[base_addr] as i16 | (device.rdram.mem[base_addr + 1] as i16) << 8;
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