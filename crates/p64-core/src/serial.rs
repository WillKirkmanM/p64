use crate::{mips, pif};
use super::{builder::registers::Registers, mmu::MmuRegister, events::{create_event, EventType}, memory, rdram::RDRAM_MASK, N64};

#[repr(u32)]
pub enum SerialRegister {
    DramAddr = 0,
    PifAddrRd64b = 1,
    PifAddrWr64b = 4,
    Status = 6,
    RegsCount = 7,
}

bitflags::bitflags! {
    pub struct SerialStatus: u32 {
        const DMA_BUSY = 1 << 0;
        const IO_BUSY = 1 << 1;
        const INTERRUPT = 1 << 12;
    }
}

#[derive(PartialEq)]
pub enum DmaDir {
    None,
    Write,
    Read,
}

pub struct Serial {
    pub regs: Registers,
    pub dma_dir: DmaDir,
}

impl Default for Serial {
    fn default() -> Self {
        Self {
            regs: Registers::new()
                .with_size(SerialRegister::RegsCount as usize)
                .build(),
            dma_dir: DmaDir::None,
        }
    }
}

pub fn read_regs(n64: &mut N64, address: u64, _access_size: memory::AccessSize) -> u32 {
    n64.serial.regs[((address & 0xFFFF) >> 2) as usize]
}

pub fn write_regs(n64: &mut N64,address: u64, value: u32, mask: u32) {
    let reg = (address & 0xFFFF) >> 2;
    match reg as u32 {
        x if x == SerialRegister::Status as u32 => {
            n64.serial.regs[reg as usize] &= !SerialStatus::INTERRUPT.bits();
            mips::clear_rcp_interrupt(n64, mips::MI_INTR_SI)
        }
        x if x == SerialRegister::PifAddrRd64b as u32 => dma_read(n64),
        x if x == SerialRegister::PifAddrWr64b as u32 => dma_write(n64),
        _ => memory::masked_write_32(&mut n64.serial.regs[reg as usize], value, mask),
    }
}

pub fn dma_event(n64: &mut N64) {
    if n64.serial.dma_dir == DmaDir::Write {
        pif::process_ram(n64);
    } else if n64.serial.dma_dir == DmaDir::Read {
        copy_pif_rdram(n64);
    } else {
        panic!("si dma unknown")
    }
    n64.serial.dma_dir = DmaDir::None;
    n64.serial.regs[SerialRegister::Status as usize] &= !(SerialStatus::DMA_BUSY.bits() | SerialStatus::IO_BUSY.bits());
    n64.serial.regs[SerialRegister::Status as usize] |= SerialStatus::INTERRUPT.bits();

    mips::set_rcp_interrupt(n64, mips::MI_INTR_SI)
}

fn dma_read(n64: &mut N64) {
    n64.serial.dma_dir = DmaDir::Read;
    let duration = pif::update_pif_ram(n64);
    n64.serial.regs[SerialRegister::Status as usize] |= SerialStatus::DMA_BUSY.bits();
    create_event(n64, EventType::SI, n64.cpu.mmu.regs[MmuRegister::Count as usize] + duration, dma_event)
}

fn dma_write(n64: &mut N64) {
    n64.serial.dma_dir = DmaDir::Write;
    copy_pif_rdram(n64);
    n64.serial.regs[SerialRegister::Status as usize] |= SerialStatus::DMA_BUSY.bits();
    create_event(n64, EventType::SI, n64.cpu.mmu.regs[MmuRegister::Count as usize] + 5200, dma_event)
}

fn copy_pif_rdram(n64: &mut N64) {
    let dram_addr = n64.serial.regs[SerialRegister::DramAddr as usize] as usize & RDRAM_MASK;
    let pif_ram_size = pif::PIF_RAM_SIZE;
    let mut i = 0;
    while i < pif_ram_size {
        let data = match n64.serial.dma_dir {
            DmaDir::Write => u32::from_ne_bytes(n64.rdram.mem[dram_addr + i..dram_addr + i + 4].try_into().unwrap()),
            DmaDir::Read => u32::from_be_bytes(n64.pif.ram[i..i + 4].try_into().unwrap()),
            _ => panic!("si dma unknown"),
        };
        match n64.serial.dma_dir {
            DmaDir::Write => n64.pif.ram[i..i + 4].copy_from_slice(&data.to_be_bytes()),
            DmaDir::Read => n64.rdram.mem[dram_addr + i..dram_addr + i + 4].copy_from_slice(&data.to_ne_bytes()),
            _ => (),
        }
        i += 4;
    }
}