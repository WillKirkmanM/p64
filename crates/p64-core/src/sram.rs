use p64_gui::storage::SaveTypes;

use super::peripheral;
use super::rdram::RDRAM_MASK;
use super::N64;

pub const FLASHRAM_TYPE_ID: u32 = 0x11118001;
pub const MX29L1100_ID: u32 = 0x00c2001e;

const SRAM_MASK: usize = 0xFFFF;
const SRAM_SIZE: usize = 0x8000;
const FLASHRAM_SIZE: usize = 0x20000;
const MX29L0000_ID: u32 = 0x00c20000;
const MX29L0001_ID: u32 = 0x00c20001;

#[derive(Clone, PartialEq, Default)]
pub enum FlashramMode {
    ReadArray,
    #[default]
    Status,
    PageProgram,
}

#[derive(Clone)]
pub struct Flashram {
    pub mode: FlashramMode,
    pub page_buf: [u8; 128],
    pub silicon_id: [u32; 2],
}

impl Default for Flashram {
    fn default() -> Self {
        Flashram {
            mode: FlashramMode::ReadArray,
            page_buf: [0xff; 128],
            silicon_id: [FLASHRAM_TYPE_ID, MX29L1100_ID],
        }
    }
}

fn format_memory(memory: &mut Vec<u8>, size: usize) {
    if memory.len() < size {
        memory.resize(size, 0xFF);
    }
}

fn dma_read_sram(
    n64: &mut N64,
    mut cart_addr: u32,
    mut dram_addr: u32,
    length: u32,
) {
    dram_addr &= RDRAM_MASK as u32;
    cart_addr &= SRAM_MASK as u32;

    format_memory(&mut n64.ui.save_data.data.sram.0, SRAM_SIZE);

    for i in 0..length {
        n64.ui.save_data.data.sram.0[(cart_addr + i) as usize] = n64.rdram.mem[(dram_addr + i) as usize ^ n64.byte_swap];
    }

    n64.ui.save_data.data.sram.1 = true;
}

fn dma_read_flash(n64: &mut N64,cart_addr: u32, dram_addr: u32, length: u32) {
    format_memory(&mut n64.ui.save_data.data.flash.0, FLASHRAM_SIZE);

    if (cart_addr & 0x1ffff) == 0x00000 && length == 128 && n64.flashram.mode == FlashramMode::PageProgram {
        for i in 0..length {
            n64.flashram.page_buf[i as usize] = n64.rdram.mem[(dram_addr + i) as usize ^ n64.byte_swap];
        }
    } else {
        panic!("unknown flash dma read");
    }
}

pub fn dma_read(n64: &mut N64,cart_addr: u32, dram_addr: u32, length: u32) -> u64 {
    if n64.ui.save_data.save_type.contains(&SaveTypes::Sram) {
        dma_read_sram(n64, cart_addr, dram_addr, length)
    } else {
        dma_read_flash(n64, cart_addr, dram_addr, length)
    }
    peripheral::calculate_cycles(n64, 2, length)
}

fn dma_write_sram(n64: &mut N64, mut cart_addr: u32, mut dram_addr: u32, length: u32) {
    dram_addr &= RDRAM_MASK as u32;
    cart_addr &= SRAM_MASK as u32;

    format_memory(&mut n64.ui.save_data.data.sram.0, SRAM_SIZE);

    for i in 0..length {
        n64.rdram.mem[(dram_addr + i) as usize ^ n64.byte_swap] = n64.ui.save_data.data.sram.0[(cart_addr + i) as usize];
    }
}

fn dma_write_flash(n64: &mut N64, mut cart_addr: u32, mut dram_addr: u32, length: u32,) {
    dram_addr &= RDRAM_MASK as u32;

    match (cart_addr & 0x1ffff, length, n64.flashram.mode.clone()) {
        (addr, _, FlashramMode::ReadArray) if addr < 0x10000 => {
            format_memory(&mut n64.ui.save_data.data.flash.0, FLASHRAM_SIZE);
            if n64.flashram.silicon_id[1] == MX29L1100_ID
                || n64.flashram.silicon_id[1] == MX29L0000_ID
                || n64.flashram.silicon_id[1] == MX29L0001_ID
            {
                cart_addr = (cart_addr & 0xffff) * 2;
            } else {
                cart_addr &= 0xffff;
            }

            for i in 0..length {
                n64.rdram.mem[(dram_addr + i) as usize ^ n64.byte_swap] =
                    n64.ui.save_data.data.flash.0[(cart_addr + i) as usize];
            }
        }
        _ => panic!("unknown flash dma write"),
    }
}

pub fn dma_write(n64: &mut N64,cart_addr: u32, dram_addr: u32, length: u32) -> u64 {
    if n64.ui.save_data.save_type.contains(&SaveTypes::Sram) {
        dma_write_sram(n64, cart_addr, dram_addr, length)
    } else {
        dma_write_flash(n64, cart_addr, dram_addr, length)
    }
    peripheral::calculate_cycles(n64, 2, length)
}
