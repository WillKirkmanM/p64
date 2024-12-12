use crate::cpu;

use super::{mmu::{add_cycles, MmuCause, MmuRegister, MmuStatus, MmuXContext::{BadVPN2Mask, RegionMask}}, cpu::{in_delay_slot, State}, memory::{self, masked_write_64, AccessType}, N64};

pub fn check_pending_interrupts(n64: &mut N64) {
    let status = n64.cpu.mmu.regs[MmuRegister::Status as usize];
    let cause = n64.cpu.mmu.regs[MmuRegister::Cause as usize];

    // interrupt disabled, or no pending interrupts
    if (status & cause & MmuCause::IPMask as u64) == 0 { return; }

    // interrupts disabled globally, or error/exception is already set
    if (status & (MmuStatus::IE as u64 | MmuStatus::EXL as u64 | MmuStatus::ERL as u64)) != MmuStatus::IE as u64 { return; }

    exception_general(n64, 0x180);
}

pub fn floating_point_exception(n64: &mut N64) { set_cause_and_exception(n64, MmuCause::ExccodeFPE as u64); }
pub fn trap_exception(n64: &mut N64) { set_cause_and_exception(n64, MmuCause::ExccodeTR as u64) }
pub fn syscall_exception(n64: &mut N64) { set_cause_and_exception(n64, MmuCause::ExccodeSys as u64) }
pub fn break_exception(n64: &mut N64) { set_cause_and_exception(n64, MmuCause::ExccodeBP as u64) }
pub fn reserved_exception(n64: &mut N64,cop: u64) { set_cause_and_exception(n64, (MmuCause::ExccodeRI as u64) | cop) }

pub fn cop_unusable_exception(n64: &mut N64,cop: u64) { set_cause_and_exception(n64, (MmuCause::ExccodeCPU as u64) | cop); }

pub fn tlb_miss_exception(n64: &mut N64,address: u64, access_type: memory::AccessType) {
    let cause = if access_type == memory::AccessType::Read {
        MmuCause::ExccodeTLBL as u64
    } else {
        MmuCause::ExccodeTLBS as u64
    };

    n64.cpu.mmu.regs[MmuRegister::Cause as usize] = cause;
    n64.cpu.mmu.regs[MmuRegister::BadVAddr as usize] = address;

    update_context_registers(n64, address);

    let mut vector_offset = 0x180;
    let mut valid = true;

    for entry in &n64.cpu.mmu.tlb_entries {
        if is_address_in_range(address, entry.start_even, entry.end_even) {
            valid = entry.v_even != 0;
            if valid && access_type == AccessType::Write && entry.d_even == 0 {
                n64.cpu.mmu.regs[MmuRegister::Cause as usize] = MmuCause::ExccodeMod as u64;
                valid = false;
            }
            break;
        }
        if is_address_in_range(address, entry.start_odd, entry.end_odd) {
            valid = entry.v_odd != 0;
            if valid && access_type == AccessType::Write && entry.d_odd == 0 {
                n64.cpu.mmu.regs[MmuRegister::Cause as usize] = MmuCause::ExccodeMod as u64;
                valid = false;
            }
            break;
        }
    }

    if n64.cpu.mmu.regs[MmuRegister::Status as usize] & (MmuStatus::EXL as u64) == 0 && valid {
        vector_offset = 0;
    }

    exception_general(n64, vector_offset);
}

fn set_cause_and_exception(n64: &mut N64,cause: u64) {
    n64.cpu.mmu.regs[MmuRegister::Cause as usize] = cause;
    exception_general(n64, 0x180);
}

fn update_context_registers(n64: &mut N64,address: u64) {
    masked_write_64(
        &mut n64.cpu.mmu.regs[MmuRegister::Context as usize],
        address >> 9,
        BadVPN2Mask as u64,
    );
    masked_write_64(
        &mut n64.cpu.mmu.regs[MmuRegister::XContext as usize],
        address >> 9,
        BadVPN2Mask as u64,
    );
    masked_write_64(
        &mut n64.cpu.mmu.regs[MmuRegister::XContext as usize],
        address >> 31,
        RegionMask as u64,
    );
    masked_write_64(
        &mut n64.cpu.mmu.regs[MmuRegister::EntryHi as usize],
        address,
        0xFFFFE000,
    );
}

fn is_address_in_range(address: u64, start: u64, end: u64) -> bool { address & !3 >= start && address & !3 <= end }

fn exception_general(n64: &mut N64,vector_offset: u32) {
    let status = n64.cpu.mmu.regs[MmuRegister::Status as usize];

    if status & (MmuStatus::EXL as u64) == 0 {
        n64.cpu.mmu.regs[MmuRegister::EPC as usize] = n64.cpu.pc;
        if in_delay_slot(n64) {
            n64.cpu.mmu.regs[MmuRegister::Cause as usize] |= MmuCause::BD as u64;
            n64.cpu.mmu.regs[MmuRegister::EPC as usize] -= 4;
        } else {
            n64.cpu.mmu.regs[MmuRegister::Cause as usize] &= !(MmuCause::BD as u64);
        }
    }

    n64.cpu.mmu.regs[MmuRegister::Status as usize] |= MmuStatus::EXL as u64;

    n64.cpu.pc = if status & (MmuStatus::BEV as u64) == 0 {
        cpu::se32((0x80000000 + vector_offset) as i32)
    } else {
        cpu::se32((0xBFC00200 + vector_offset) as i32)
    };

    n64.cpu.branch_state.state = State::Exception;

    add_cycles(n64, 2);
}