use serde::{Deserialize, Serialize};

use crate::{audio::{handle_ai_register_read, handle_ai_register_write}, cache, cartridge, mips, peripheral, pif, rdp, rdram, rsp, serial, video};

use super::{cache::{dcache_read, dcache_write}, tlb, N64};

#[repr(usize)]
pub enum MemoryMap {
    RdramDram = 0x00000000,
    RspMem = 0x04000000,
    RspRegs = 0x04040000,
    RspRegsPc = 0x04080000,
    DpcRegs = 0x04100000,
    DpsRegs = 0x04200000,
    MiRegs = 0x04300000,
    ViRegs = 0x04400000,
    AiRegs = 0x04500000,
    PiRegs = 0x04600000,
    RiRegs = 0x04700000,
    SerialRegisters = 0x04800000,
    Dom2Addr2 = 0x08000000,
    CartRom = 0x10000000,
    PifMem = 0x1fc00000,
}

const KSEG_MASK: u64 = 0xc0000000;
const KSEG0_BASE: u64 = 0x80000000;
const CACHE_MASK: u64 = 0x20000000;
const PHYSICAL_MASK: u64 = 0x1FFFFFFF;
const MASK_64K: usize = 0xFFFF;

#[derive(PartialEq)]
pub enum AccessType {
    Write,
    Read,
}

#[derive(Copy, Clone, Serialize, Deserialize)]
pub enum AccessSize {
    Word = 4,
    Dword = 8,
    Dcache = 16,
    Icache = 32,
}

pub struct Memory {
    pub fast_read: [fn(&mut N64, u64, AccessSize) -> u32; 0x2000],
    pub memory_map_read: [fn(&mut N64, u64, AccessSize) -> u32; 0x2000],
    pub memory_map_write: [fn(&mut N64, u64, u32, u32); 0x2000],
    pub icache: [cache::ICache; 512],
    pub dcache: [cache::DCache; 512],
}

impl Default for Memory {
    fn default() -> Self {
        Self {
            fast_read: [unmapped_read_mem; 0x2000],
            memory_map_read: [unmapped_read_mem; 0x2000],
            memory_map_write: [unmapped_write_mem; 0x2000],
            icache: [cache::ICache::default(); 512],
            dcache: [cache::DCache::default(); 512],
        }
    }
}

pub fn unmapped_read_mem(_device: &mut N64, _address: u64, _access_size: AccessSize) -> u32 { 3 }
pub fn unmapped_write_mem(_device: &mut N64, _address: u64, _value: u32, _mask: u32) {}

pub fn masked_write_32(dst: &mut u32, value: u32, mask: u32) { *dst = (*dst & !mask) | (value & mask) }
pub fn masked_write_64(dst: &mut u64, value: u64, mask: u64) { *dst = (*dst & !mask) | (value & mask) }

pub fn translate_address(n64: &mut N64, address: u64, access_type: AccessType) -> (u64, bool, bool) {
    match address & KSEG_MASK {
        addr if addr != KSEG0_BASE => tlb::get_physical_address(n64, address, access_type),
        _ => (address & PHYSICAL_MASK, (address & CACHE_MASK) == 0, false),
    }
}

#[inline]
fn get_handler_index(phys_address: u64) -> usize { (phys_address >> 16) as usize }

pub fn read_data(n64: &mut N64, addr: u64, size: AccessSize, cached: bool) -> u32 {
    match cached {
        true => dcache_read(n64, addr),
        false => n64.memory.memory_map_read[get_handler_index(addr)](n64, addr, size),
    }
}

pub fn write_data(n64: &mut N64, addr: u64, value: u32, mask: u32, cached: bool) {
    match cached {
        true => dcache_write(n64, addr, value, mask),
        false => n64.memory.memory_map_write[get_handler_index(addr)](n64, addr, value, mask),
    }
}

macro_rules! map_memory_range {
    ($n64: expr, $start:expr, $end:expr, $read:path, $write:path $(, $fast_read:path)?) => {
        for i in ($start as usize >> 16)..=($end as usize >> 16) {
            $n64.memory.memory_map_read[i] = $read;
            $n64.memory.memory_map_write[i] = $write;
            $($n64.memory.fast_read[i] = $fast_read)?
        }
    };
}

pub fn initialise_memory_map(n64: &mut N64) {
    map_memory_range!(n64, MemoryMap::RdramDram, MemoryMap::RdramDram as usize + 0x03EFFFFF, rdram::read_mem_fast, rdram::write_mem, rdram::read_mem);
    map_memory_range!(n64, MemoryMap::RspMem, MemoryMap::RspMem as usize + 0x3FFFF, rsp::read_mem, rsp::write_mem, rsp::read_mem_fast);
    map_memory_range!(n64, MemoryMap::CartRom, MemoryMap::CartRom as usize + n64.cartridge.rom.len() - 1, cartridge::read_mem, cartridge::write_mem, cartridge::read_mem_fast);

    map_memory_range!(n64, MemoryMap::RspRegs, MemoryMap::RspRegs as usize + MASK_64K, rsp::read_regs, rsp::write_regs);
    map_memory_range!(n64, MemoryMap::RspRegsPc, MemoryMap::RspRegsPc as usize + MASK_64K, rsp::read_regs2, rsp::write_regs2);
    map_memory_range!(n64, MemoryMap::DpcRegs, MemoryMap::DpcRegs as usize + MASK_64K, rdp::read_regs_dpc, rdp::write_regs_dpc);
    map_memory_range!(n64, MemoryMap::DpsRegs, MemoryMap::DpsRegs as usize + MASK_64K, rdp::read_regs_dps, rdp::write_regs_dps);
    map_memory_range!(n64, MemoryMap::MiRegs, MemoryMap::MiRegs as usize + MASK_64K, mips::read_regs, mips::write_regs);
    map_memory_range!(n64, MemoryMap::ViRegs, MemoryMap::ViRegs as usize + MASK_64K, video::read_regs, video::write_regs);
    map_memory_range!(n64, MemoryMap::AiRegs, MemoryMap::AiRegs as usize + MASK_64K, handle_ai_register_read, handle_ai_register_write);
    map_memory_range!(n64, MemoryMap::PiRegs, MemoryMap::PiRegs as usize + MASK_64K, peripheral::read_regs, peripheral::write_regs);
    map_memory_range!(n64, MemoryMap::RiRegs, MemoryMap::RiRegs as usize + MASK_64K, rdram::read_regs, rdram::write_regs);
    map_memory_range!(n64, MemoryMap::SerialRegisters, MemoryMap::SerialRegisters as usize + MASK_64K, serial::read_regs, serial::write_regs);
    map_memory_range!(n64, MemoryMap::PifMem, MemoryMap::PifMem as usize + MASK_64K, pif::read_mem, pif::write_mem);

    cache::init(n64)
}
