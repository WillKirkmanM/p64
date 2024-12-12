use crate::{cpu, events, N64};
use crate::tlb::TlbEntry;

use super::cpu::State;
use super::events::{create_event, EventType};
use super::exceptions::{break_exception, check_pending_interrupts, reserved_exception, syscall_exception};
use super::{memory, tlb};
use super::{fpu, tlb::TlbLookupTable};
use super::builder::registers::Registers;
use crate::mmu;

#[repr(u32)]
#[derive(PartialEq, Copy, Clone, Debug)]
pub enum MmuRegister {
    Index = 0,
    Random = 1,
    EntryLo0 = 2,
    EntryLo1 = 3,
    Context = 4,
    PageMask = 5,
    Wired = 6,
    BadVAddr = 8,
    Count = 9,
    EntryHi = 10,
    Compare = 11,
    Status = 12,
    Cause = 13,
    EPC = 14,
    PrevID = 15,
    Config = 16,
    LLAddr = 17,
    XContext = 20,
    TagLo = 28,
    ErrorEPC = 30,
    RegsCount = 32,
}

impl<T> std::ops::Index<MmuRegister> for Registers<T> {
    type Output = T;   
    fn index(&self, reg: MmuRegister) -> &Self::Output { &self.regs[reg as usize] }
}

impl<T> std::ops::IndexMut<MmuRegister> for Registers<T> {
    fn index_mut(&mut self, reg: MmuRegister) -> &mut Self::Output { &mut self.regs[reg as usize] }
}

#[repr(u64)]
pub enum MmuStatus {
    IE = 1 << 0,
    EXL = 1 << 1,
    ERL = 1 << 2,
    BEV = 1 << 22,
    FR = 1 << 26,
    CU1 = 1 << 29,
    CU2 = 1 << 30,
}

#[repr(u64)]
pub enum MmuCause {
    ExccodeMod = 1 << 2,
    ExccodeTLBL = 2 << 2,
    ExccodeTLBS = 3 << 2,
    ExccodeSys = 8 << 2,
    ExccodeBP = 9 << 2,
    ExccodeRI = 10 << 2,
    ExccodeCPU = 11 << 2,
    ExccodeTR = 13 << 2,
    ExccodeFPE = 15 << 2,
    IP2 = 1 << 10,
    IP7 = 1 << 15,
    BD = 1 << 31,
    CE1 = 1 << 28,
    CE2 = 1 << 29,
    IPMask = 0b00000000000000001111111100000000,
    ExccodeMask = 0x1F << 2,
}

#[repr(u64)]
pub enum MmuXContext {
    BadVPN2Mask = 0b01111111111111111111111111110000,
    RegionMask = 0b110000000000000000000000000000000,
}

#[repr(u64)]
pub enum MmuMask {
    Index = 0b10000000000000000000000000111111,
    EntryLo = 0b0000000000000000000000000000000000111111111111111111111111111111,
    Context = 0b1111111111111111111111111111111111111111100000000000000000000000,
    PageMask = 0b00000001111111111110000000000000,
    Wired = 0b00000000000000000000000000111111,
    EntryHi = 0b1100000000000000000000001111111111111111111111111110000011111111,
    Status = 0b11111111010101111111111111111111,
    Cause = 0b00000000000000000000001100000000,
    Config = 0b00001111000000001000000000001111,
    LLAddr = 0b11111111111111111111111111111111,
    WatchLo = 0b11111111111111111111111111111011,
    WatchHi = 0b00000000000000000000000000001111,
    XContext = 0b1111111111111111111111111111111000000000000000000000000000000000,
    ParityErr = 0b00000000000000000000000011111111,
    TagLo = 0b00001111111111111111111111000000,
}

pub struct Mmu {
    pub reg_latch: u64,
    pub regs: Registers<u64>,
    pub reg_write_masks: [u64; MmuRegister::RegsCount as usize],
    pub instrs: [fn(&mut N64, u32); 32],
    pub instrs2: [fn(&mut N64, u32); 32],
    pub tlb_lut_r: Vec<TlbLookupTable>,
    pub tlb_lut_w: Vec<TlbLookupTable>,
    pub tlb_entries: [TlbEntry; 32],
}

impl Default for Mmu {
    fn default() -> Self {
        fn default_instruction(_device: &mut N64, _value: u32) {}

        Mmu {
            reg_latch: 0,
            regs: Registers::new().with_size(MmuRegister::RegsCount as usize),
            reg_write_masks: [0; MmuRegister::RegsCount as usize],
            instrs: [default_instruction; 32],
            instrs2: [default_instruction; 32],
            tlb_lut_r: vec![
                TlbLookupTable {
                    address: 0,
                    cached: false,
                };
                0x100000
            ],
            tlb_lut_w: vec![
                TlbLookupTable {
                    address: 0,
                    cached: false,
                };
                0x100000
            ],
            tlb_entries: [TlbEntry::default(); 32],
        }
    }
}

pub fn add_cycles(n64: &mut N64,cycles: u64) { n64.cpu.mmu.regs[MmuRegister::Count as usize] += cycles }

pub fn syscall(n64: &mut N64,_opcode: u32) { syscall_exception(n64) }
pub fn break_(n64: &mut N64,_opcode: u32) { break_exception(n64) }
pub fn reserved(n64: &mut N64,_opcode: u32) { reserved_exception(n64, 0) }

fn mfc0(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[cpu::rt(opcode) as usize] = cpu::se32(
        (get_control_registers(n64, cpu::rd(opcode))) as u32 as i32,
    )
}

fn dmfc0(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[cpu::rt(opcode) as usize] = get_control_registers(n64, cpu::rd(opcode))
}

fn mtc0(n64: &mut N64,opcode: u32) {
    mmu::set_control_registers(
        n64,
        cpu::rd(opcode),
        cpu::se32(
            n64.cpu.gpr[cpu::rt(opcode) as usize] as u32 as i32,
        ),
    )
}

fn dmtc0(n64: &mut N64,opcode: u32) {
    mmu::set_control_registers(n64, cpu::rd(opcode), n64.cpu.gpr[cpu::rt(opcode) as usize])
}

fn tlbr(n64: &mut N64,_opcode: u32) { tlb::read(n64, n64.cpu.mmu.regs[MmuRegister::Index as usize]) }
fn tlbwi(n64: &mut N64,_opcode: u32) { tlb::write(n64, n64.cpu.mmu.regs[MmuRegister::Index as usize]) }
fn tlbwr(n64: &mut N64,_opcode: u32) {
    let random = set_random_register(n64);
    tlb::write(n64, random)
}

fn tlbp(n64: &mut N64,_opcode: u32) { tlb::probe(n64) }

fn eret(n64: &mut N64,_opcode: u32) {
    if n64.cpu.mmu.regs[MmuRegister::Status as usize] & MmuStatus::ERL as u64 != 0 {
        n64.cpu.pc = n64.cpu.mmu.regs[MmuRegister::ErrorEPC as usize];
        n64.cpu.mmu.regs[MmuRegister::Status as usize] &= !(MmuStatus::ERL as u64)
    } else {
        n64.cpu.pc = n64.cpu.mmu.regs[MmuRegister::EPC as usize];
        n64.cpu.mmu.regs[MmuRegister::Status as usize] &= !(MmuStatus::EXL as u64)
    }
    n64.cpu.branch_state.state = State::Exception;
    n64.cpu.llbit = false;
    check_pending_interrupts(n64)
}

fn execute_cp0(n64: &mut N64,opcode: u32) { n64.cpu.mmu.instrs2[(opcode & 0x3F) as usize](n64, opcode) }

fn get_control_registers(n64: &N64, index: u32) -> u64 {
    match index {
        x if x == MmuRegister::Count as u32 => n64.cpu.mmu.regs[index as usize] >> 1,
        x if x == MmuRegister::Random as u32 => set_random_register(n64),
        7 | 21 | 22 | 23 | 24 | 25 | 31 => n64.cpu.mmu.reg_latch,
        _ => n64.cpu.mmu.regs[index as usize],
    }
}

fn set_control_registers(n64: &mut N64,index: u32, mut data: u64) {
    n64.cpu.mmu.reg_latch = data;
    match index {
        x if x == MmuRegister::Count as u32 => {
            data &= 0xFFFFFFFF;
            data <<= 1;
            events::translate_events(
                n64,
                n64.cpu.mmu.regs[MmuRegister::Count as usize],
                data,
            );
            n64.cpu.mmu.regs[MmuRegister::Count as usize] = data;
            return;
        }
        x if x == MmuRegister::Wired as u32 => n64.cpu.mmu.regs[MmuRegister::Random as usize] = 31,
        x if x == MmuRegister::Compare as u32 => {
            data &= 0xFFFFFFFF;
            let current_count = (n64.cpu.mmu.regs[MmuRegister::Count as usize] >> 1) & 0xFFFFFFFF;
            let mut compare_event_diff = (data as u32).wrapping_sub(current_count as u32);

            if compare_event_diff == 0 { compare_event_diff += u32::MAX }

            create_event(n64, EventType::Compare, n64.cpu.mmu.regs[MmuRegister::Count as usize] + ((compare_event_diff as u64) << 1), compare_event);
            n64.cpu.mmu.regs[MmuRegister::Cause as usize] &= !(MmuCause::IP7 as u64);
        }
        x if x == MmuRegister::Status as u32 => {
            if data & MmuStatus::FR as u64 != n64.cpu.mmu.regs[index as usize] & MmuStatus::FR as u64 {
                fpu::set_fgr_registers(n64, data)
            }
        }
        _ => {}
    }
    memory::masked_write_64(&mut n64.cpu.mmu.regs[index as usize], data, n64.cpu.mmu.reg_write_masks[index as usize],);
    check_pending_interrupts(n64);
}

fn compare_event(n64: &mut N64) {
    n64.cpu.mmu.regs[MmuRegister::Cause as usize] &= !(MmuCause::ExccodeMask as u64);
    n64.cpu.mmu.regs[MmuRegister::Cause as usize] |= MmuCause::IP7 as u64;

    create_event(n64, EventType::Compare, n64.cpu.next_event_count + (u32::MAX as u64), compare_event);
    check_pending_interrupts(n64);
}

fn set_random_register(n64: &N64) -> u64 {
    if n64.cpu.mmu.regs[MmuRegister::Wired as usize] > 31 {
        (u64::MAX - n64.cpu.mmu.regs[MmuRegister::Count as usize]) & 0x3F
    } else {
        (u64::MAX - n64.cpu.mmu.regs[MmuRegister::Count as usize])
            % (32 - n64.cpu.mmu.regs[MmuRegister::Wired as usize])
            + n64.cpu.mmu.regs[MmuRegister::Wired as usize]
    }
}

#[derive(Copy, Clone)]
enum MmuInstruction {
    MFC0 = 0,
    DMFC0 = 1,
    MTC0 = 4,
    DMTC0 = 5,
    EXECUTECP0 = 16,
}

type MmuInstrFn = fn(&mut N64, u32);

pub const MMU_INSTRUCTIONS: [MmuInstrFn; 32] = {
    let mut instrs: [MmuInstrFn; 32] = [mmu::reserved; 32];
    
    instrs[MmuInstruction::MFC0 as usize] = mmu::mfc0 as MmuInstrFn;
    instrs[MmuInstruction::DMFC0 as usize] = mmu::dmfc0 as MmuInstrFn;
    instrs[MmuInstruction::MTC0 as usize] = mmu::mtc0 as MmuInstrFn;
    instrs[MmuInstruction::DMTC0 as usize] = mmu::dmtc0 as MmuInstrFn;
    instrs[MmuInstruction::EXECUTECP0 as usize] = mmu::execute_cp0 as MmuInstrFn;
    
    instrs
};

#[derive(Copy, Clone)]
enum MmuSecondaryInstruction {
    TLBR = 1,
    TLBWI = 2,
    TLBWR = 6,
    TLBP = 8,
    ERET = 24,
}

pub const MMU_SECONDARY_INSTRUCTIONS: [MmuInstrFn; 32] = {
    let mut instrs: [MmuInstrFn; 32] = [mmu::reserved; 32];
    
    instrs[MmuSecondaryInstruction::TLBR as usize] = mmu::tlbr;
    instrs[MmuSecondaryInstruction::TLBWI as usize] = mmu::tlbwi;
    instrs[MmuSecondaryInstruction::TLBWR as usize] = mmu::tlbwr;
    instrs[MmuSecondaryInstruction::TLBP as usize] = mmu::tlbp;
    instrs[MmuSecondaryInstruction::ERET as usize] = mmu::eret;
    
    instrs
};

const MMU_INIT_VALUES: [(MmuRegister, u64); 8] = [
    (MmuRegister::Random, 0b00000000000000000000000000011111),
    (MmuRegister::Config, 0b01110000000001101110010001100000),
    (MmuRegister::Status, 0b00000000010000000000000000000100),
    (MmuRegister::PrevID, 0b00000000000000000000101100100010),
    (MmuRegister::EPC, 0xFFFFFFFF_FFFFFFFF),
    (MmuRegister::ErrorEPC, 0xFFFFFFFF_FFFFFFFF),
    (MmuRegister::BadVAddr, 0xFFFFFFFF),
    (MmuRegister::Context, 0x7FFFF0),
];

pub fn write_mmu_registers(n64: &mut N64) {
    n64.cpu.mmu.reg_write_masks = [
        MmuMask::Index as u64,
        0, // Random, read only
        MmuMask::EntryLo as u64,
        MmuMask::EntryLo as u64,
        MmuMask::Context as u64,
        MmuMask::PageMask as u64,
        MmuMask::Wired as u64,
        0,               // 7
        0,               // BadVAddr, read only
        u32::MAX as u64, // count
        MmuMask::EntryHi as u64,
        u32::MAX as u64, // compare
        MmuMask::Status as u64,
        MmuMask::Cause as u64,
        u64::MAX, // EPC
        0,        // previd, read only
        MmuMask::Config as u64,
        MmuMask::LLAddr as u64,
        MmuMask::WatchLo as u64,
        MmuMask::WatchHi as u64,
        MmuMask::XContext as u64,
        0, // 21
        0, // 22
        0, // 23
        0, // 24
        0, // 25
        MmuMask::ParityErr as u64,
        0, // cache error
        MmuMask::TagLo as u64,
        0,        // taghi
        u64::MAX, // ErrorPC
        0,        // 31
    ];

    for &(reg, value) in MMU_INIT_VALUES.iter() { n64.cpu.mmu.regs[reg as usize] = value }

    create_event(n64, EventType::Compare, u32::MAX as u64, compare_event)
}