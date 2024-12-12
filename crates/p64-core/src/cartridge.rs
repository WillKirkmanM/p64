use chrono::Datelike;
use chrono::Timelike;

use crate::N64;
use p64_gui::storage::SaveTypes;

use sha2::{Digest, Sha256};

use super::mmu::add_cycles;
use super::events::create_event;
use super::memory;
use super::peripheral;
use super::rdram::RDRAM_MASK;
use super::{builder::memory::Memory, mmu::MmuRegister, events::EventType, peripheral::{PiRegister, PiStatus}};

#[derive(Clone, Debug, PartialEq, Default)]
pub enum CicType {
    #[default]
    CicNus6101,
    CicNus6102,
    CicNus6103,
    CicNus6105,
    CicNus6106,
    CicNus5167,
}

pub const CART_MASK: usize = 0xFFFFFFF;

#[derive(Clone, Default)]
pub struct Cartridge {
    pub rom: Memory,
    pub pal: bool,
    pub latch: u32,
    pub cic_type: CicType,
    pub cic_seed: u8,
    pub rdram_size_offset: usize,
    pub rtc: AfRtc,
}



#[repr(u8)]
#[derive(Copy, Clone)]
pub enum JCmd {
    Status = 0x00,
    EepromRead = 0x04,
    EepromWrite = 0x05,
    AfRtcStatus = 0x06,
    AfRtcRead = 0x07,
    AfRtcWrite = 0x08,
    Reset = 0xff,
}

#[repr(u16)]
#[derive(Copy, Clone)]
pub enum Jdt {
    AfRtc = 0x1000,
    Eeprom4k = 0x8000,
    Eeprom16k = 0xc000,
}

pub const EEPROM_BLOCK_SIZE: usize = 8;
pub const EEPROM_MAX_SIZE: usize = 0x800;

#[derive(Clone, Default)]
pub struct AfRtc {
    pub control: u16,
}

pub fn read_mem_fast(n64: &mut N64, address: u64, _access_size: memory::AccessSize) -> u32 {
    n64.cartridge.rom.read_mem_fast(address)
}

pub fn read_mem(n64: &mut N64, address: u64, _access_size: memory::AccessSize) -> u32 {
    let cycles = peripheral::calculate_cycles(n64, 1, 4);
    add_cycles(n64, cycles);

    if n64.peripheral.regs[PiRegister::Status as usize] & PiStatus::IO_BUSY != 0 {
        n64.cartridge.latch
    } else {
        n64.cartridge.rom.read_mem_fast(address) as u32
    }
}

pub fn write_mem(n64: &mut N64,_address: u64, value: u32, mask: u32) {
    n64.cartridge.latch = value & mask;
    n64.peripheral.regs[PiRegister::Status as usize] |= PiStatus::IO_BUSY;

    let cycles = peripheral::calculate_cycles(n64, 1, 4);
    create_event(n64, EventType::PI, n64.cpu.mmu.regs[MmuRegister::Count as usize] + cycles, peripheral::dma_event);
}

fn dma_transfer(n64: &mut N64, mut cart_addr: u32, mut dram_addr: u32, length: u32, is_read: bool) -> u64 {
    dram_addr &= RDRAM_MASK as u32;
    cart_addr &= CART_MASK as u32;

    let mut i = dram_addr;
    let mut j = cart_addr;

    while i < dram_addr + length && j < n64.cartridge.rom.len() as u32 {
        if is_read {
            n64.cartridge.rom[j as usize] = n64.rdram.mem[i as usize ^ n64.byte_swap];
        } else {
            n64.rdram.mem[i as usize ^ n64.byte_swap] = n64.cartridge.rom[j as usize];
        }
        i += 1;
        j += 1;
    }

    while i < dram_addr + length {
        if !is_read {
            n64.rdram.mem[i as usize ^ n64.byte_swap] = 0;
        }
        i += 1;
    }

    peripheral::calculate_cycles(n64, 1, length)
}

pub fn dma_read(n64: &mut N64, cart_addr: u32, dram_addr: u32, length: u32) -> u64 {
    dma_transfer(n64, cart_addr, dram_addr, length, true)
}

pub fn dma_write(n64: &mut N64, cart_addr: u32, dram_addr: u32, length: u32) -> u64 {
    dma_transfer(n64, cart_addr, dram_addr, length, false)
}

pub fn write_cartridge_to_memory(n64: &mut N64,rom_file: Vec<u8>) {
    n64.cartridge.rom = Memory::new()
        .with_size(rom_file.len())
        .with_mask(CART_MASK)
        .build();

    n64.cartridge.rom.data = rom_file;

    set_system_region(n64, n64.cartridge.rom.data[0x3E]);
    set_cic(n64);

    n64.ui.game_info.hash = calculate_hash(&n64.cartridge.rom.data);

    n64.ui.game_info.id = String::from_utf8(n64.cartridge.rom.data[0x3B..0x3E].to_vec()).unwrap_or_else(|_| String::from("UNK"));
}

pub fn load_rom_save(n64: &mut N64) {
    if n64.ui.save_data.data.romsave.0.is_empty() {
        return;
    }
    for (key, value) in n64.ui.save_data.data.romsave.0.iter() {
        n64.cartridge.rom[key.parse::<usize>().unwrap()] =
            serde_json::from_value(value.clone()).unwrap()
    }
}

pub fn set_system_region(n64: &mut N64,country: u8) {
    let pal_codes: [u8; 8] = [b'D', b'F', b'I', b'P', b'S', b'U', b'X', b'Y'];
    n64.cartridge.pal = pal_codes.contains(&country);
}

fn set_cic(n64: &mut N64) {
    let hash = calculate_hash(&n64.cartridge.rom[0x40..0x1000]);
    let (cic_type, cic_seed, rdram_size_offset) = match hash.as_str() {
        "B99F06C4802C2377E31E388435955EF3E99C618A6D55D24699D828EB1075F1EB" => (CicType::CicNus6101, 0x3F, 0x318),
        "61E88238552C356C23D19409FE5570EE6910419586BC6FC740F638F761ADC46E" => (CicType::CicNus6102, 0x3F, 0x318),
        "BF3620D30817007091EBE9BDDD1B88C23B8A0052170B3309CDE5B6B4238E45E7" => (CicType::CicNus6103, 0x78, 0x318),
        "04B7BC6717A9F0EB724CF927E74AD3876C381CBB280D841736FC5E55580B756B" => (CicType::CicNus6105, 0x91, 0x3F0),
        "36ADC40148AF56F0D78CD505EB6A90117D1FD6F11C6309E52ED36BC4C6BA340E" => (CicType::CicNus6106, 0x85, 0x318),
        "53C0088FB777870D0AF32F0251E964030E2E8B72E830C26042FD191169508C05" => (CicType::CicNus5167, 0xdd, 0x318),
        _ => {
            println!("unknown IPL3 {}", hash);
            (CicType::CicNus6102, 0x3F, 0x318)
        }
    };

    n64.cartridge.cic_type = cic_type;
    n64.cartridge.cic_seed = cic_seed;
    n64.cartridge.rdram_size_offset = rdram_size_offset;
}

fn calculate_hash(data: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(data);
    format!("{:X}", hasher.finalize())
}

fn byte2bcd(mut n: u32) -> u8 {
    n %= 100;
    (((n / 10) << 4) | (n % 10)) as u8
}

pub fn process(n64: &mut N64,channel: usize) {
    let cmd = n64.pif.ram[n64.pif.channels[channel].tx_buf.unwrap()];

    match cmd {
        x if x == JCmd::Reset as u8 || x == JCmd::Status as u8 => handle_status_or_reset(n64, channel),
        x if x == JCmd::EepromRead as u8 => handle_eeprom_read(n64, channel),
        x if x == JCmd::EepromWrite as u8 => handle_eeprom_write(n64, channel),
        x if x == JCmd::AfRtcStatus as u8 => handle_af_rtc_status(n64, channel),
        x if x == JCmd::AfRtcRead as u8 => handle_af_rtc_read(n64, channel),
        x if x == JCmd::AfRtcWrite as u8 => handle_af_rtc_write(n64, channel),
        _ => panic!("unknown cart command"),
    }
}

fn handle_status_or_reset(n64: &mut N64,channel: usize) {
    let eeprom_type = if n64.ui.save_data.save_type.contains(&SaveTypes::Eeprom16k) {
        Jdt::Eeprom16k
    } else if n64.ui.save_data.save_type.contains(&SaveTypes::Eeprom4k) {
        Jdt::Eeprom4k
    } else {
        n64.pif.ram[n64.pif.channels[channel].rx.unwrap()] |= 0x80;
        return;
    };

    let rx_buf = n64.pif.channels[channel].rx_buf.unwrap();
    n64.pif.ram[rx_buf] = eeprom_type as u8;
    n64.pif.ram[rx_buf + 1] = (eeprom_type as u16 >> 8) as u8;
    n64.pif.ram[rx_buf + 2] = 0;
}

fn handle_eeprom_read(n64: &mut N64,channel: usize) { eeprom_read_block(n64, n64.pif.channels[channel].tx_buf.unwrap() + 1, n64.pif.channels[channel].rx_buf.unwrap()) }
fn handle_eeprom_write(n64: &mut N64,channel: usize) { eeprom_write_block(n64, n64.pif.channels[channel].tx_buf.unwrap() + 1, n64.pif.channels[channel].tx_buf.unwrap() + 2, n64.pif.channels[channel].rx_buf.unwrap() ) }

fn handle_af_rtc_status(n64: &mut N64,channel: usize) {
    let rx_buf = n64.pif.channels[channel].rx_buf.unwrap();
    n64.pif.ram[rx_buf] = Jdt::AfRtc as u8;
    n64.pif.ram[rx_buf + 1] = (Jdt::AfRtc as u16 >> 8) as u8;
    n64.pif.ram[rx_buf + 2] = 0x00;
}

fn handle_af_rtc_read(n64: &mut N64,channel: usize) {
    af_rtc_read_block(n64, n64.pif.channels[channel].tx_buf.unwrap() + 1, n64.pif.channels[channel].rx_buf.unwrap(), n64.pif.channels[channel].rx_buf.unwrap() + 8);
}

fn handle_af_rtc_write(n64: &mut N64,channel: usize) {
    af_rtc_write_block(n64, n64.pif.channels[channel].tx_buf.unwrap() + 1, n64.pif.channels[channel].tx_buf.unwrap() + 2, n64.pif.channels[channel].rx_buf.unwrap());
}

fn format_eeprom(n64: &mut N64) {
    if n64.ui.save_data.data.eeprom.0.len() < EEPROM_MAX_SIZE {
        n64.ui.save_data.data.eeprom.0.resize(EEPROM_MAX_SIZE, 0xFF)
    }
}

fn eeprom_read_block(n64: &mut N64,block: usize, offset: usize) {
    let address = n64.pif.ram[block] as usize * EEPROM_BLOCK_SIZE;

    format_eeprom(n64);

    n64.pif.ram[offset..offset + EEPROM_BLOCK_SIZE].copy_from_slice(&n64.ui.save_data.data.eeprom.0[address..address + EEPROM_BLOCK_SIZE]);
}

fn eeprom_write_block(n64: &mut N64,block: usize, offset: usize, status: usize) {
    let address = n64.pif.ram[block] as usize * EEPROM_BLOCK_SIZE;

    format_eeprom(n64);

    n64.ui.save_data.data.eeprom.0[address..address + EEPROM_BLOCK_SIZE].copy_from_slice(&n64.pif.ram[offset..offset + EEPROM_BLOCK_SIZE]);

    n64.pif.ram[status] = 0x00;

    n64.ui.save_data.data.eeprom.1 = true
}

fn time2data(n64: &mut N64,offset: usize) {
    let now: chrono::DateTime<chrono::Local> = chrono::Local::now();

    n64.pif.ram[offset] = byte2bcd(now.second());
    n64.pif.ram[offset + 1] = byte2bcd(now.minute());
    n64.pif.ram[offset + 2] = 0x80 + byte2bcd(now.hour());
    n64.pif.ram[offset + 3] = byte2bcd(now.day());
    n64.pif.ram[offset + 4] = byte2bcd(now.weekday().num_days_from_sunday());
    n64.pif.ram[offset + 5] = byte2bcd(now.month());
    n64.pif.ram[offset + 6] = byte2bcd(now.year() as u32 - 1900);
    n64.pif.ram[offset + 7] = byte2bcd((now.year() as u32 - 1900) / 100);
}

fn af_rtc_read_block(n64: &mut N64,block: usize, offset: usize, status: usize) {
    match n64.pif.ram[block] {
        0 => {
            n64.pif.ram[offset] = n64.cartridge.rtc.control as u8;
            n64.pif.ram[offset + 1] = (n64.cartridge.rtc.control >> 8) as u8;
            n64.pif.ram[status] = 0x00;
        }
        1 => panic!("AF-RTC reading block 1 is not implemented!"),
        2 => {
            time2data(n64, offset);
            n64.pif.ram[status] = 0x00;
        }
        _ => panic!("AF-RTC read invalid block"),
    }
}

fn af_rtc_write_block(n64: &mut N64,block: usize, offset: usize, status: usize) {
    match n64.pif.ram[block] {
        0 => {
            n64.cartridge.rtc.control = (n64.pif.ram[offset + 1] as u16) << 8 | n64.pif.ram[offset] as u16;
            n64.pif.ram[status] = 0x00;
        }
        1 => {
            if (n64.cartridge.rtc.control & 0x01) != 0 {
                return;
            }
            panic!("AF-RTC writing block 1 is not implemented!");
        }
        2 => {
            if (n64.cartridge.rtc.control & 0x02) != 0 {
                return;
            }
            panic!("AF-RTC writing block 2 is not implemented!");
        }
        _ => panic!("AF-RTC write invalid block"),
    }
}