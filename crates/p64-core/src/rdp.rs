use p64_gui::video;

use crate::mips;

use super::builder::registers::Registers;
use super::mmu::MmuRegister;
use super::events::create_event;
use super::events::EventType;
use super::memory;
use super::N64;

#[repr(u32)]
pub enum DpcReg {
    Start = 0,
    End = 1,
    Current = 2,
    Status = 3,
    Clock = 4,
    BufBusy = 5,
    PipeBusy = 6,
    Tmem = 7,
    RegsCount = 8,
}

#[repr(u32)]
pub enum DpsReg {
    RegsCount = 4,
}

bitflags::bitflags! {
    struct DpcStatus: u32 {
        const XBUS_DMEM_DMA = 1 << 0;
        const FREEZE = 1 << 1;
        const FLUSH = 1 << 2;
        const START_GCLK = 1 << 3;
        const TMEM_BUSY = 1 << 4;
        const PIPE_BUSY = 1 << 5;
        const CMD_BUSY = 1 << 6;
        const CBUF_READY = 1 << 7;
        const START_VALID = 1 << 10;
    }
}

bitflags::bitflags! {
    struct DpcStatusWrite: u32 {
        const CLR_XBUS_DMEM_DMA = 1 << 0;
        const SET_XBUS_DMEM_DMA = 1 << 1;
        const CLR_FREEZE = 1 << 2;
        const SET_FREEZE = 1 << 3;
        const CLR_FLUSH = 1 << 4;
        const SET_FLUSH = 1 << 5;
        const CLR_TMEM_CTR = 1 << 6;
        const CLR_PIPE_CTR = 1 << 7;
        const CLR_CMD_CTR = 1 << 8;
        const CLR_CLOCK_CTR = 1 << 9;
    }
}

pub struct Rdp {
    pub regs_dpc: Registers,
    pub regs_dps: Registers,
    pub wait_frozen: bool,
}

impl Default for Rdp {
    fn default() -> Self {
        let mut rdp = Self {
            regs_dpc: Registers::default().with_size(DpcReg::RegsCount as usize).build(),
            regs_dps: Registers::default().with_size(DpcReg::RegsCount as usize).build(),
            wait_frozen: false,
        };

        // Initialize DPC status register
        let mut status = DpcStatus::from_bits_truncate(rdp.regs_dpc[DpcReg::Status as usize]);
        status.insert(DpcStatus::START_GCLK | DpcStatus::PIPE_BUSY | DpcStatus::CBUF_READY);
        rdp.regs_dpc[DpcReg::Status as usize] = status.bits();

        rdp
    }
}

pub fn read_regs_dpc(n64: &mut N64, address: u64, _access_size: memory::AccessSize) -> u32 {
    n64.rdp.regs_dpc[((address & 0xFFFF) >> 2) as usize]
}

pub fn write_regs_dpc(n64: &mut N64,address: u64, value: u32, mask: u32) {
    let reg = (address & 0xFFFF) >> 2;
    match reg as u32 {
        x if x == DpcReg::Current as u32 || x == DpcReg::Clock as u32 || x == DpcReg::BufBusy as u32 || x == DpcReg::PipeBusy as u32 || x == DpcReg::Tmem as u32 => {}
        x if x == DpcReg::Status as u32 => update_dpc_status(n64, value & mask),
        x if x == DpcReg::Start as u32 => {
            if (n64.rdp.regs_dpc[DpcReg::Status as usize] & DpcStatus::START_VALID.bits()) == 0 {
                memory::masked_write_32(
                    &mut n64.rdp.regs_dpc[reg as usize],
                    value & 0xFFFFF8,
                    mask,
                )
            }
            n64.rdp.regs_dpc[DpcReg::Status as usize] |= DpcStatus::START_VALID.bits()
        }
        x if x == DpcReg::End as u32 => {
            memory::masked_write_32(
                &mut n64.rdp.regs_dpc[reg as usize],
                value & 0xFFFFF8,
                mask,
            );
            if (n64.rdp.regs_dpc[DpcReg::Status as usize] & DpcStatus::START_VALID.bits()) != 0 {
                n64.rdp.regs_dpc[DpcReg::Current as usize] =
                    n64.rdp.regs_dpc[DpcReg::Start as usize];
                n64.rdp.regs_dpc[DpcReg::Status as usize] &= !DpcStatus::START_VALID.bits()
            }
            if n64.rdp.regs_dpc[DpcReg::Status as usize] & DpcStatus::FREEZE.bits() == 0 {
                run_rdp(n64)
            } else {
                n64.rdp.wait_frozen = true;
            }
        }
        _ => memory::masked_write_32(&mut n64.rdp.regs_dpc[reg as usize], value, mask),
    }
}

fn run_rdp(n64: &mut N64) {
    let timer = video::process_rdp_list(&mut n64.rdp.regs_dpc.try_as_array_mut().unwrap(), &mut n64.rsp.memory.mem.data);
    if timer != 0 {
        create_event(n64, EventType::DP, n64.cpu.mmu.regs[MmuRegister::Count as usize] + timer, rdp_interrupt_event)
    }
}

pub fn read_regs_dps(n64: &mut N64, address: u64, _access_size: memory::AccessSize) -> u32 {
    n64.rdp.regs_dps[((address & 0xFFFF) >> 2) as usize]
}

pub fn write_regs_dps(n64: &mut N64,address: u64, value: u32, mask: u32) {
    memory::masked_write_32(
        &mut n64.rdp.regs_dps[((address & 0xFFFF) >> 2) as usize],
        value,
        mask,
    )
}

macro_rules! update_status {
    ($write_status:expr, $status:expr, $clr_flag:ident, $set_flag:ident, $status_flag:ident) => {
        if $write_status.contains(DpcStatusWrite::$clr_flag) { $status.remove(DpcStatus::$status_flag); }
        if $write_status.contains(DpcStatusWrite::$set_flag) { $status.insert(DpcStatus::$status_flag); }
    };
    ($write_status:expr, $status:expr, $clr_flag:ident, $status_flag:ident, $n64:expr, $reg:ident) => {
        if $write_status.contains(DpcStatusWrite::$clr_flag) {
        }
    };
}

fn update_dpc_status(n64: &mut N64, w: u32) {
    let mut status = DpcStatus::from_bits_truncate(n64.rdp.regs_dpc[DpcReg::Status as usize]);
    let write_status = DpcStatusWrite::from_bits_truncate(w);

    update_status!(write_status, status, CLR_XBUS_DMEM_DMA, SET_XBUS_DMEM_DMA, XBUS_DMEM_DMA);
    update_status!(write_status, status, CLR_FREEZE, SET_FREEZE, FREEZE);
    update_status!(write_status, status, CLR_FLUSH, SET_FLUSH, FLUSH);

    if write_status.contains(DpcStatusWrite::CLR_FREEZE) {
        if n64.rdp.wait_frozen {
            run_rdp(n64);
            n64.rdp.wait_frozen = false;
        }
    }

    update_status!(write_status, status, CLR_TMEM_CTR, TMEM_BUSY, n64, Tmem);
    update_status!(write_status, status, CLR_PIPE_CTR, PIPE_BUSY, n64, PipeBusy);
    update_status!(write_status, status, CLR_CMD_CTR, CMD_BUSY, n64, BufBusy);

    if write_status.contains(DpcStatusWrite::CLR_CLOCK_CTR) {
        n64.rdp.regs_dpc[DpcReg::Clock as usize] = 0;
    }

    n64.rdp.regs_dpc[DpcReg::Status as usize] = status.bits();
}

fn rdp_interrupt_event(n64: &mut N64) {
    video::rdp_full_sync();
    mips::set_rcp_interrupt(n64, mips::MI_INTR_DP)
}
