use crate::builder::memory::Memory;
use super::memory::AccessSize;

use super::mmu::add_cycles;
use super::N64;

pub const RDRAM_MASK: usize = 0xFFFFFF;
pub const RDRAM_SIZE: usize = 0x800000;

pub struct Rdram {
    pub mem: Memory,
}

impl Default for Rdram {
    fn default() -> Self {
        const ALIGNMENT: usize = 64 * 1024;

        Self {
            mem: Memory::new()
                .with_size(RDRAM_SIZE)
                .with_alignment(ALIGNMENT)
                .with_mask(RDRAM_MASK)
                .is_little_endian()
                .is_rdram()
                .calculate_cycles_from_addr_size()
                .build()
        }
    }
}

pub fn rdram_calculate_cycles(length: u64) -> u64 { 31 + (length / 3) }

pub fn read_mem_fast(n64: &mut N64, address: u64, _access_size: AccessSize) -> u32 {
   n64.rdram.mem.read_mem_fast(address) 
}

pub fn read_mem(n64: &mut N64, address: u64, access_size: AccessSize) -> u32 {
    add_cycles(n64, rdram_calculate_cycles(access_size as u64) / (access_size as u64 / 4));
    n64.rdram.mem.read_mem_fast(address)
}

pub fn write_mem(n64: &mut N64,address: u64, value: u32, _mask: u32) {
    n64.rdram.mem.write_mem(address, value);
}

pub fn get_rdram_shared_pointer(n64: &mut N64) -> (*mut u8, usize) {
    let data_ptr = n64.rdram.mem.data_mut().as_mut_ptr();
    
    (data_ptr, RDRAM_SIZE)
}


#[derive(Debug, Clone, Copy)]
#[repr(u32)]
pub enum RiReg {
    Select = 3,
    RegsCount = 8,
}

#[derive(Clone)]
pub struct RdramInterface {
    pub regs: Memory
}

impl Default for RdramInterface {
    fn default() -> Self {
        Self {
            regs: Memory::new()
                .with_size(RiReg::RegsCount as usize)
        }
    }
}

impl RdramInterface {
    pub fn read(self, address: u64, _access_size: AccessSize) -> u32 {
        let index = ((address & 0xFFFF) >> 2) as u32;
        if index == RiReg::Select as u32 {
            return 0x14;
        }

        self.regs.read_u32(index as u64)
    }

    pub fn write(&mut self, address: u64, value: u32, mask: u32) {
        self.regs.write_u32((address & 0xFFFF) >> 2, value, mask);
    }
}

pub fn read_regs(n64: &mut N64, address: u64, access_size: AccessSize) -> u32 {
    n64.rdram_interface.clone().read(address, access_size)
}

pub fn write_regs(n64: &mut N64,address: u64, value: u32, mask: u32) {
    n64.rdram_interface.write(address, value, mask)
}
