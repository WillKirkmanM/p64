use tracing::info;

use crate::builder::registers::Registers;
use crate::fpu::Fpu;
use crate::rcp::Rcp;
use crate::{cache, events, exceptions, tlb};

use crate::mmu::{ MMU_INSTRUCTIONS, MMU_SECONDARY_INSTRUCTIONS, write_mmu_registers, Mmu};
use crate::{mmu, fpu};

use super::mmu::{add_cycles, MmuRegister};
use super::events::EventType;
use super::{rcp, memory, N64};
use crate::cpu;



#[derive(Default, PartialEq)]
pub enum State {
    #[default]
    Step,
    Take,
    NotTaken,
    DelaySlotTaken,
    DelaySlotNotTaken,
    Discard,
    Exception,
}

#[derive(Default)]
pub struct BranchState {
    pub state: State,
    pub pc: u64,
}

pub struct Cpu {
    pub mmu: mmu::Mmu,
    pub fpu: Fpu,
    pub rcp: rcp::Rcp,
    pub branch_state: BranchState,
    pub gpr: [u64; 32],
    pub pc: u64,
    pub pc_phys: u64,
    pub lo: u64,
    pub hi: u64,
    pub running: u8,
    pub llbit: bool,
    pub clock_rate: u64,
    pub instrs: [fn(&mut N64, u32); 64],
    pub special_instrs: [fn(&mut N64, u32); 64],
    pub regimm_instrs: [fn(&mut N64, u32); 32],
    pub events: [events::Event; EventType::Count as usize],
    pub next_event_count: u64,
    pub next_event: usize,
}

impl Default for Cpu {
    fn default() -> Self {
        fn default_instruction(_device: &mut N64, _value: u32) {}

        info!("Initialising CPU...");

        let cpu = Cpu {
            mmu: Mmu {
                regs: Registers::<u64>::new().with_size(MmuRegister::RegsCount as usize),
                reg_write_masks: [0; MmuRegister::RegsCount as usize],
                reg_latch: 0,
                instrs: MMU_INSTRUCTIONS,
                instrs2: MMU_SECONDARY_INSTRUCTIONS,
                tlb_lut_w: vec![
                    tlb::TlbLookupTable {
                        address: 0,
                        cached: false,
                    };
                    0x100000
                ],
                tlb_lut_r: vec![
                    tlb::TlbLookupTable {
                        address: 0,
                        cached: false,
                    };
                    0x100000
                ],
                tlb_entries: [tlb::TlbEntry::default(); 32],
                // ..Default::default()
            },
            fpu: Fpu::default(),
            rcp: Rcp::default(),
            branch_state: BranchState {
                state: State::Step,
                pc: 0,
            },
            gpr: [0; 32],
            clock_rate: 0,
            pc: 0xBFC00000,
            pc_phys: 0,
            llbit: false,
            lo: 0,
            hi: 0,
            running: 0,
            instrs: [default_instruction; 64],
            special_instrs: [default_instruction; 64],
            regimm_instrs: [default_instruction; 32],
            events: [events::Event {
                enabled: false,
                count: u64::MAX,
                handler: events::dummy_event,
            }; EventType::Count as usize],
            next_event_count: u64::MAX,
            next_event: 0,
        };
        
        info!("CPU has Initialised");

        cpu
        
    }
}

pub fn decode_opcode(n64: &N64, opcode: u32) -> fn(&mut N64,u32) {
    match opcode >> 26 {
        0 => n64.cpu.special_instrs[(opcode & 0x3F) as usize],
        1 => n64.cpu.regimm_instrs[((opcode >> 16) & 0x1F) as usize],
        16..=18 => [&n64.cpu.mmu.instrs, &n64.cpu.fpu.instrs, &n64.cpu.rcp.instrs]
            [(opcode >> 26) as usize - 16][((opcode >> 21) & 0x1F) as usize],
        _ => n64.cpu.instrs[(opcode >> 26) as usize],
    }
}
type InstrFn = for<'a> fn(&'a mut N64, u32);

const MAIN_OPS: [(usize, InstrFn); 48] = [
    (2, j), (3, jal), (4, beq), (5, bne), (6, blez), (7, bgtz),
    (8, addi), (9, addiu), (10, slti), (11, sltiu), (12, andi),
    (13, ori), (14, xori), (15, lui), (20, beql), (21, bnel),
    (22, blezl), (23, bgtzl), (24, daddi), (25, daddiu),
    (26, ldl), (27, ldr), (32, lb), (33, lh), (34, lwl),
    (35, lw), (36, lbu), (37, lhu), (38, lwr), (39, lwu),
    (40, sb), (41, sh), (42, swl), (43, sw), (44, sdl),
    (45, sdr), (46, swr), (47, cache), (48, ll), (49, fpu::lwc1),
    (52, lld), (53, fpu::ldc1), (55, ld), (56, sc),
    (57, fpu::swc1), (60, scd), (61, fpu::sdc1), (63, sd)
];

const SPECIAL_OPS: [(usize, InstrFn); 52] = [
    (0, sll), (2, srl), (3, sra), (4, sllv), (6, srlv), (7, srav),
    (8, jr), (9, jalr), (12, mmu::syscall), (13, mmu::break_),
    (15, sync), (16, mfhi), (17, mthi), (18, mflo), (19, mtlo),
    (20, dsllv), (22, dsrlv), (23, dsrav), (24, mult), (25, multu),
    (26, div), (27, divu), (28, dmult), (29, dmultu), (30, ddiv),
    (31, ddivu), (32, add), (33, addu), (34, sub), (35, subu),
    (36, and), (37, or), (38, xor), (39, nor), (42, slt),
    (43, sltu), (44, dadd), (45, daddu), (46, dsub), (47, dsubu),
    (48, tge), (49, tgeu), (50, tlt), (51, tltu), (52, teq),
    (54, tne), (56, dsll), (58, dsrl), (59, dsra), (60, dsll32),
    (62, dsrl32), (63, dsra32)
];

const REGIMM_OPS: [(usize, InstrFn); 14] = [
    (0, bltz), (1, bgez), (2, bltzl), (3, bgezl), (8, tgei),
    (9, tgeiu), (10, tlti), (11, tltiu), (12, teqi), (14, tnei),
    (16, bltzal), (17, bgezal), (18, bltzall), (19, bgezall)
];

pub fn start_cpu(n64: &mut N64) {
    n64.cpu.clock_rate = 93750000;

    info!("Initializing CPU instructions...");
    n64.cpu.instrs = {
        let reserved_fn: InstrFn = mmu::reserved as fn(&mut N64, u32);
        let mut arr = [reserved_fn; 64];
        for &(i, f) in MAIN_OPS.iter() { arr[i] = f; }
        arr
    };
    info!("CPU instructions initialized");
    

    info!("Initializing CPU special instructions...");
    n64.cpu.special_instrs = {
        let reserved_fn: InstrFn = mmu::reserved as fn(&mut N64,u32);
        let mut arr = [reserved_fn; 64];
        for &(i, f) in SPECIAL_OPS.iter() { arr[i] = f; }
        arr
    };
    info!("CPU special instructions initialized");

    info!("Initializing CPU regimm instructions...");
    n64.cpu.regimm_instrs = {
        let reserved_fn: InstrFn = mmu::reserved as fn(&mut N64,u32);
        let mut arr = [reserved_fn; 32];
        for &(i, f) in REGIMM_OPS.iter() { arr[i] = f; }
        arr
    };
    info!("CPU regimm instructions initialized");

    write_mmu_registers(n64);
}

pub fn in_delay_slot(n64: &N64) -> bool { matches!(n64.cpu.branch_state.state, State::DelaySlotTaken | State::DelaySlotNotTaken) }
fn in_delay_slot_taken(n64: &N64) -> bool { matches!(n64.cpu.branch_state.state, State::DelaySlotTaken) }

pub fn run(n64: &mut N64) {
    n64.cpu.running = 1;
    while n64.cpu.running == 1 {
        n64.cpu.gpr[0] = 0; // gpr 0 is read only
        let (cached, err);
        (n64.cpu.pc_phys, cached, err) = memory::translate_address(
            n64,
            n64.cpu.pc,
            memory::AccessType::Read,
        );

        if err { continue; } // TLB exception}

        if cached {
            cache::icache_fetch(n64, n64.cpu.pc_phys)
        } else {
            let opcode = n64.memory.memory_map_read[(n64.cpu.pc_phys >> 16) as usize](
                n64,
                n64.cpu.pc_phys,
                memory::AccessSize::Word,
            );
            cpu::decode_opcode(n64, opcode)(n64, opcode);
        }

        match n64.cpu.branch_state.state {
            State::Discard => n64.cpu.pc += 8,
            State::DelaySlotTaken => n64.cpu.pc = n64.cpu.branch_state.pc,
            State::Exception => (),
            _ => n64.cpu.pc += 4,
        }
        
        n64.cpu.branch_state.state = match n64.cpu.branch_state.state {
            State::Take => State::DelaySlotTaken,
            State::NotTaken => State::DelaySlotNotTaken,
            _ => State::Step,
        };


        add_cycles(n64, 1);

        if n64.cpu.mmu.regs[MmuRegister::Count as usize] > n64.cpu.next_event_count {
            events::trigger_event(n64)
        }
    }
}

pub fn rd(opcode: u32) -> u32 { (opcode >> 11) & 0x1F }
pub fn rs(opcode: u32) -> u32 { (opcode >> 21) & 0x1F }
pub fn rt(opcode: u32) -> u32 { (opcode >> 16) & 0x1F }
fn sa(opcode: u32) -> u32 { (opcode >> 6) & 0x1F }
pub fn imm(opcode: u32) -> u16 { opcode as u16 }
pub fn se32(value: i32) -> u64 { value as i64 as u64 }
pub fn se16(value: i16) -> u64 { value as i64 as u64 }
fn se8(value: i8) -> u64 { value as i64 as u64 }

fn bshift<T: Into<u64>>(address: T) -> u64 { ((address.into() & 3) ^ 3) << 3 }
fn hshift<T: Into<u64>>(address: T) -> u64 { ((address.into() & 2) ^ 2) << 3 }
fn bits_below_mask<T: Into<u64>>(x: T) -> u64 { (1 << x.into()) - 1 }
fn bits_above_mask<T: Into<u64>>(x: T) -> u64 { !bits_below_mask(x) }

#[macro_export]
macro_rules! check_idle_loop {
    ($n64: expr, $condition:expr) => {
        if $condition
            && $n64.memory.fast_read[($n64.cpu.pc_phys >> 16) as usize](
                $n64,
                $n64.cpu.pc_phys + 4,
                memory::AccessSize::Word,
            ) == 0
        {
            $n64.cpu.mmu.regs[MmuRegister::Count as usize] = $n64.cpu.next_event_count
        }
    };
}

pub fn check_relative_idle_loop(n64: &mut N64,opcode: u32) { check_idle_loop!(n64, imm(opcode) as i16 == -1); }
fn check_absolute_idle_loop(n64: &mut N64,opcode: u32) { check_idle_loop!(n64, (opcode & 0x3FFFFFF) as u64 == (n64.cpu.pc_phys & 0x0FFFFFFF) >> 2); }

fn j(n64: &mut N64,opcode: u32) {
    if cpu::in_delay_slot_taken(n64) {
        return;
    }
    check_absolute_idle_loop(n64, opcode);
    n64.cpu.branch_state.state = State::Take;
    n64.cpu.branch_state.pc =
        (n64.cpu.pc + 4) & 0xFFFFFFFFF0000000 | ((opcode & 0x3FFFFFF) << 2) as u64
}

fn jal(n64: &mut N64,opcode: u32) {
    if cpu::in_delay_slot_taken(n64) {
        n64.cpu.gpr[31] = n64.cpu.branch_state.pc + 4
    } else {
        n64.cpu.gpr[31] = n64.cpu.pc + 8
    }
    if !cpu::in_delay_slot_taken(n64) {
        check_absolute_idle_loop(n64, opcode);
        n64.cpu.branch_state.state = State::Take;
        n64.cpu.branch_state.pc =
            (n64.cpu.pc + 4) & 0xFFFFFFFFF0000000 | ((opcode & 0x3FFFFFF) << 2) as u64
    } else if !cpu::in_delay_slot(n64) {
        n64.cpu.branch_state.state = State::NotTaken;
    }
}

macro_rules! branch_instruction {
    ($name:ident, $condition:expr) => {
        fn $name(n64: &mut N64,opcode: u32) {
            if $condition(n64, opcode) {
                check_relative_idle_loop(n64, opcode);
                n64.cpu.branch_state.state = State::Take;
                n64.cpu.branch_state.pc = n64.cpu.pc.wrapping_add(se16(imm(opcode) as i16) << 2) + 4;
            } else {
                n64.cpu.branch_state.state = State::NotTaken;
            }
        }
    };
}

branch_instruction!(beq, |n64: &mut N64,opcode| n64.cpu.gpr[rs(opcode) as usize] == n64.cpu.gpr[rt(opcode) as usize]);
branch_instruction!(bne, |n64: &mut N64,opcode| n64.cpu.gpr[rs(opcode) as usize] != n64.cpu.gpr[rt(opcode) as usize]);
branch_instruction!(blez, |n64: &mut N64,opcode| n64.cpu.gpr[rs(opcode) as usize] as i64 <= 0); 
branch_instruction!(bgtz, |n64: &mut N64,opcode| n64.cpu.gpr[rs(opcode) as usize] as i64 > 0);


macro_rules! impl_immediate_op {
    ($name:ident, $op:expr) => {
        fn $name(n64: &mut N64,opcode: u32) {
            n64.cpu.gpr[rt(opcode) as usize] = $op(
                n64.cpu.gpr[rs(opcode) as usize],
                imm(opcode)
            );
        }
    };
}

impl_immediate_op!(addi, |rs, imm| se32(rs as i32).wrapping_add(se16(imm as i16)));
impl_immediate_op!(addiu, |rs, imm| se32(rs as i32).wrapping_add(se16(imm as i16)));
impl_immediate_op!(slti, |rs, imm| ((rs as i64) < (imm as i16 as i64)) as u64);
impl_immediate_op!(sltiu, |rs, imm| (rs < se16(imm as i16)) as u64);

fn andi(n64: &mut N64,opcode: u32) { n64.cpu.gpr[rt(opcode) as usize] = n64.cpu.gpr[rs(opcode) as usize] & imm(opcode) as u64 }
fn ori(n64: &mut N64,opcode: u32) { n64.cpu.gpr[rt(opcode) as usize] = n64.cpu.gpr[rs(opcode) as usize] | imm(opcode) as u64 }
fn xori(n64: &mut N64,opcode: u32) { n64.cpu.gpr[rt(opcode) as usize] = n64.cpu.gpr[rs(opcode) as usize] ^ imm(opcode) as u64 }
fn lui(n64: &mut N64,opcode: u32) { n64.cpu.gpr[rt(opcode) as usize] = se32(((imm(opcode) as u32) << 16) as i32) }

fn beql(n64: &mut N64,opcode: u32) {
    if n64.cpu.gpr[rs(opcode) as usize] == n64.cpu.gpr[rt(opcode) as usize] {
        check_relative_idle_loop(n64, opcode);
        n64.cpu.branch_state.state = State::Take;
        n64.cpu.branch_state.pc = n64.cpu.pc.wrapping_add(se16(imm(opcode) as i16) << 2) + 4;
    } else {
        n64.cpu.branch_state.state = State::Discard;
    }
}

fn bnel(n64: &mut N64,opcode: u32) {
    if n64.cpu.gpr[rs(opcode) as usize] != n64.cpu.gpr[rt(opcode) as usize] {
        check_relative_idle_loop(n64, opcode);
        n64.cpu.branch_state.state = State::Take;
        n64.cpu.branch_state.pc = n64.cpu.pc.wrapping_add(se16(imm(opcode) as i16) << 2) + 4;
    } else {
        n64.cpu.branch_state.state = State::Discard;
    }
}

fn blezl(n64: &mut N64,opcode: u32) {
    if n64.cpu.gpr[rs(opcode) as usize] as i64 <= 0 {
        check_relative_idle_loop(n64, opcode);
        n64.cpu.branch_state.state = State::Take;
        n64.cpu.branch_state.pc = n64.cpu.pc.wrapping_add(se16(imm(opcode) as i16) << 2) + 4;
    } else {
        n64.cpu.branch_state.state = State::Discard;
    }
}

fn bgtzl(n64: &mut N64,opcode: u32) {
    if n64.cpu.gpr[rs(opcode) as usize] as i64 > 0 {
        check_relative_idle_loop(n64, opcode);
        n64.cpu.branch_state.state = State::Take;
        n64.cpu.branch_state.pc = n64.cpu.pc.wrapping_add(se16(imm(opcode) as i16) << 2) + 4;
    } else {
        n64.cpu.branch_state.state = State::Discard;
    }
}

fn daddi(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[rt(opcode) as usize] =
        n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16))
}

fn daddiu(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[rt(opcode) as usize] =
        n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16))
}

fn ldl(n64: &mut N64,opcode: u32) {
    let addr = n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16));
    let n = addr & 7;
    let shift = 8 * n;

    let mask = bits_below_mask(8 * n);

    let (mut phys_address, cached, err) =
        memory::translate_address(n64, addr, memory::AccessType::Read);
    if err {
        return;
    }
    phys_address &= !7;

    let mut w = [0; 2];
    w[0] = memory::read_data(
        n64,
        phys_address,
        memory::AccessSize::Dword,
        cached,
    );
    w[1] = memory::read_data(
        n64,
        phys_address + 4,
        memory::AccessSize::Dword,
        cached,
    );
    n64.cpu.gpr[rt(opcode) as usize] = (n64.cpu.gpr[rt(opcode) as usize] & mask)
        | (((((w[0]) as u64) << 32) | (w[1]) as u64) << shift)
}

fn ldr(n64: &mut N64,opcode: u32) {
    let addr = n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16));
    let n = addr & 7;
    let shift = 8 * (7 - n);

    let mask = if n == 7 {
        0
    } else {
        bits_above_mask(8 * (n + 1))
    };

    let (mut phys_address, cached, err) =
        memory::translate_address(n64, addr, memory::AccessType::Read);
    if err {
        return;
    }
    phys_address &= !7;

    let mut w = [0; 2];
    w[0] = memory::read_data(
        n64,
        phys_address,
        memory::AccessSize::Dword,
        cached,
    );
    w[1] = memory::read_data(
        n64,
        phys_address + 4,
        memory::AccessSize::Dword,
        cached,
    );
    n64.cpu.gpr[rt(opcode) as usize] = (n64.cpu.gpr[rt(opcode) as usize] & mask)
        | (((((w[0]) as u64) << 32) | (w[1]) as u64) >> shift)
}

fn lb(n64: &mut N64,opcode: u32) {
    let (phys_address, cached, err) = memory::translate_address(
        n64,
        n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16)),
        memory::AccessType::Read,
    );
    if err {
        return;
    }

    let shift = bshift(phys_address);
    n64.cpu.gpr[rt(opcode) as usize] = se8((memory::read_data(
        n64,
        phys_address & !0x3,
        memory::AccessSize::Word,
        cached,
    ) >> shift) as u8 as i8)
}

fn lh(n64: &mut N64,opcode: u32) {
    let (phys_address, cached, err) = memory::translate_address(
        n64,
        n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16)),
        memory::AccessType::Read,
    );
    if err {
        return;
    }

    let shift = hshift(phys_address);
    n64.cpu.gpr[rt(opcode) as usize] = se16(
        (memory::read_data(
            n64,
            phys_address & !0x3,
            memory::AccessSize::Word,
            cached,
        ) >> shift) as u16 as i16,
    )
}

fn lwl(n64: &mut N64,opcode: u32) {
    let addr = n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16));
    let n = addr & 3;
    let shift = 8 * n;

    let mask = bits_below_mask(8 * n) as u32;

    let (mut phys_address, cached, err) =
        memory::translate_address(n64, addr, memory::AccessType::Read);
    if err {
        return;
    }
    phys_address &= !3;

    n64.cpu.gpr[rt(opcode) as usize] = se32(
        ((n64.cpu.gpr[rt(opcode) as usize] as u32) & mask
            | memory::read_data(
                n64,
                phys_address,
                memory::AccessSize::Word,
                cached,
            ) << shift) as i32,
    )
}

fn lw(n64: &mut N64,opcode: u32) {
    let (phys_address, cached, err) = memory::translate_address(
        n64,
        n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16)),
        memory::AccessType::Read,
    );
    if err {
        return;
    }

    n64.cpu.gpr[rt(opcode) as usize] = se32(memory::read_data(
        n64,
        phys_address,
        memory::AccessSize::Word,
        cached,
    ) as i32)
}

fn lbu(n64: &mut N64,opcode: u32) {
    let (phys_address, cached, err) = memory::translate_address(
        n64,
        n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16)),
        memory::AccessType::Read,
    );
    if err {
        return;
    }

    let shift = bshift(phys_address);
    n64.cpu.gpr[rt(opcode) as usize] = (memory::read_data(
        n64,
        phys_address & !0x3,
        memory::AccessSize::Word,
        cached,
    ) >> shift) as u8 as u64
}

fn lhu(n64: &mut N64,opcode: u32) {
    let (phys_address, cached, err) = memory::translate_address(
        n64,
        n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16)),
        memory::AccessType::Read,
    );
    if err {
        return;
    }

    let shift = hshift(phys_address);
    n64.cpu.gpr[rt(opcode) as usize] = (memory::read_data(
        n64,
        phys_address & !0x3,
        memory::AccessSize::Word,
        cached,
    ) >> shift) as u16 as u64
}

fn lwr(n64: &mut N64,opcode: u32) {
    let addr = n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16));
    let n = addr & 3;
    let shift = 8 * (3 - n);

    let mask = if n == 3 {
        0
    } else {
        bits_above_mask(8 * (n + 1))
    };

    let (mut phys_address, cached, err) =
        memory::translate_address(n64, addr, memory::AccessType::Read);
    if err {
        return;
    }
    phys_address &= !3;

    n64.cpu.gpr[rt(opcode) as usize] = se32(
        ((n64.cpu.gpr[rt(opcode) as usize] as u32) & (mask as u32)
            | memory::read_data(
                n64,
                phys_address,
                memory::AccessSize::Word,
                cached,
            ) >> shift) as i32,
    )
}

fn lwu(n64: &mut N64,opcode: u32) {
    let (phys_address, cached, err) = memory::translate_address(
        n64,
        n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16)),
        memory::AccessType::Read,
    );
    if err {
        return;
    }

    n64.cpu.gpr[rt(opcode) as usize] = memory::read_data(
        n64,
        phys_address,
        memory::AccessSize::Word,
        cached,
    ) as u64
}

fn sb(n64: &mut N64,opcode: u32) {
    let (phys_address, cached, err) = memory::translate_address(
        n64,
        n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16)),
        memory::AccessType::Write,
    );
    if err {
        return;
    }

    let shift = bshift(phys_address);

    memory::write_data(
        n64,
        phys_address & !0x3,
        (n64.cpu.gpr[rt(opcode) as usize] as u32) << shift,
        0xFF << shift,
        cached,
    )
}

fn sh(n64: &mut N64,opcode: u32) {
    let (phys_address, cached, err) = memory::translate_address(
        n64,
        n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16)),
        memory::AccessType::Write,
    );
    if err {
        return;
    }

    let shift = hshift(phys_address);

    memory::write_data(
        n64,
        phys_address & !0x3,
        (n64.cpu.gpr[rt(opcode) as usize] as u32) << shift,
        0xFFFF << shift,
        cached,
    )
}

fn swl(n64: &mut N64,opcode: u32) {
    let addr = n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16));
    let n = addr & 3;
    let shift = 8 * n;

    let mask = if n == 0 {
        u32::MAX
    } else {
        bits_below_mask(8 * (4 - n)) as u32
    };

    let (mut phys_address, cached, err) =
        memory::translate_address(n64, addr, memory::AccessType::Write);
    if err {
        return;
    }
    phys_address &= !3;

    memory::write_data(
        n64,
        phys_address,
        (n64.cpu.gpr[rt(opcode) as usize] >> shift) as u32,
        mask,
        cached,
    )
}

fn sw(n64: &mut N64,opcode: u32) {
    let (phys_address, cached, err) = memory::translate_address(
        n64,
        n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16)),
        memory::AccessType::Write,
    );
    if err {
        return;
    }

    memory::write_data(
        n64,
        phys_address,
        n64.cpu.gpr[rt(opcode) as usize] as u32,
        0xFFFFFFFF,
        cached,
    )
}

fn sdl(n64: &mut N64,opcode: u32) {
    let addr = n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16));
    let n = addr & 7;
    let shift = 8 * n;

    let mask = if n == 0 {
        u64::MAX
    } else {
        bits_below_mask(8 * (8 - n))
    };

    let (mut phys_address, cached, err) =
        memory::translate_address(n64, addr, memory::AccessType::Write);
    if err {
        return;
    }
    phys_address &= !7;

    let value = n64.cpu.gpr[rt(opcode) as usize] >> shift;
    memory::write_data(
        n64,
        phys_address,
        (value >> 32) as u32,
        (mask >> 32) as u32,
        cached,
    );
    memory::write_data(n64, phys_address + 4, value as u32, mask as u32, cached)
}

fn sdr(n64: &mut N64,opcode: u32) {
    let addr = n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16));
    let n = addr & 7;
    let shift = 8 * (7 - n);

    let mask = bits_above_mask(8 * (7 - n));

    let (mut phys_address, cached, err) =
        memory::translate_address(n64, addr, memory::AccessType::Write);
    if err {
        return;
    }
    phys_address &= !7;

    let value = n64.cpu.gpr[rt(opcode) as usize] << shift;
    memory::write_data(
        n64,
        phys_address,
        (value >> 32) as u32,
        (mask >> 32) as u32,
        cached,
    );
    memory::write_data(n64, phys_address + 4, value as u32, mask as u32, cached)
}

fn swr(n64: &mut N64,opcode: u32) {
    let addr = n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16));
    let n = addr & 3;
    let shift = 8 * (3 - n);

    let mask = bits_above_mask(8 * (3 - n)) as u32;

    let (mut phys_address, cached, err) =
        memory::translate_address(n64, addr, memory::AccessType::Write);
    if err {
        return;
    }
    phys_address &= !3;

    memory::write_data(
        n64,
        phys_address,
        (n64.cpu.gpr[rt(opcode) as usize] << shift) as u32,
        mask,
        cached,
    )
}

fn cache(n64: &mut N64,opcode: u32) {
    let (phys_address, _, err) = memory::translate_address(
        n64,
        n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16)),
        memory::AccessType::Read,
    );
    if err {
        return;
    }

    let dcache_line = ((phys_address >> 4) & 0x1FF) as usize;
    let icache_line = ((phys_address >> 5) & 0x1FF) as usize;
    match rt(opcode) {
        0x00 => {
            //icache index invalidate
            n64.memory.icache[icache_line].valid = false
        }
        0x04 => {
            //icache load tag
            let mut valid = 0;
            if n64.memory.icache[icache_line].valid {
                valid = 1
            }
            memory::masked_write_64(
                &mut n64.cpu.mmu.regs[MmuRegister::TagLo as usize],
                valid << 7,
                0x80,
            );
            memory::masked_write_64(
                &mut n64.cpu.mmu.regs[MmuRegister::TagLo as usize],
                0,
                0x40,
            );
            memory::masked_write_64(
                &mut n64.cpu.mmu.regs[MmuRegister::TagLo as usize],
                (n64.memory.icache[icache_line].tag >> 4) as u64,
                0xFFFFF00,
            )
        }
        0x08 => {
            //icache store tag
            n64.memory.icache[icache_line].valid =
                (n64.cpu.mmu.regs[MmuRegister::TagLo as usize] & 0x80) >> 7 != 0;
            n64.memory.icache[icache_line].tag =
                ((n64.cpu.mmu.regs[MmuRegister::TagLo as usize] & 0xFFFFF00) << 4)
                    as u32
        }
        0x10 => {
            //icache hit invalidate
            if cache::icache_hit(n64, icache_line, phys_address) {
                n64.memory.icache[icache_line].valid = false
            }
        }
        0x14 => {
            //icache fill
            cache::icache_fill(n64, icache_line, phys_address)
        }
        0x18 => {
            //icache hit write back
            if cache::icache_hit(n64, icache_line, phys_address) {
                cache::icache_writeback(n64, icache_line)
            }
        }
        0x01 => {
            //dcache index write back invalidate
            if n64.memory.dcache[dcache_line].dirty && n64.memory.dcache[dcache_line].valid {
                cache::dcache_writeback(n64, dcache_line)
            }
            n64.memory.dcache[dcache_line].valid = false
        }
        0x05 => {
            //dcache index load tag
            let mut valid = 0;
            let mut dirty = 0;
            if n64.memory.dcache[dcache_line].valid {
                valid = 1
            }
            if n64.memory.dcache[dcache_line].dirty {
                dirty = 1
            }
            memory::masked_write_64(
                &mut n64.cpu.mmu.regs[MmuRegister::TagLo as usize],
                valid << 7,
                0x80,
            );
            memory::masked_write_64(
                &mut n64.cpu.mmu.regs[MmuRegister::TagLo as usize],
                dirty << 6,
                0x40,
            );
            memory::masked_write_64(
                &mut n64.cpu.mmu.regs[MmuRegister::TagLo as usize],
                (n64.memory.dcache[dcache_line].tag >> 4) as u64,
                0xFFFFF00,
            )
        }
        0x09 => {
            //dcache index store tag
            n64.memory.dcache[dcache_line].valid =
                (n64.cpu.mmu.regs[MmuRegister::TagLo as usize] & 0x80) >> 7 != 0;
            n64.memory.dcache[dcache_line].dirty =
                (n64.cpu.mmu.regs[MmuRegister::TagLo as usize] & 0x40) >> 6 != 0;
            n64.memory.dcache[dcache_line].tag =
                ((n64.cpu.mmu.regs[MmuRegister::TagLo as usize] & 0xFFFFF00) << 4)
                    as u32
        }
        0x0D => {
            //dcache create dirty exclusive
            if !cache::dcache_hit(n64, dcache_line, phys_address)
                && n64.memory.dcache[dcache_line].dirty
            {
                cache::dcache_writeback(n64, dcache_line)
            }
            n64.memory.dcache[dcache_line].tag = (phys_address & !0xFFF) as u32;
            n64.memory.dcache[dcache_line].valid = true;
            n64.memory.dcache[dcache_line].dirty = true
        }
        0x11 => {
            //dcache hit invalidate
            if cache::dcache_hit(n64, dcache_line, phys_address) {
                n64.memory.dcache[dcache_line].valid = false;
                n64.memory.dcache[dcache_line].dirty = false
            }
        }
        0x15 => {
            //dcache hit write back invalidate
            if cache::dcache_hit(n64, dcache_line, phys_address) {
                if n64.memory.dcache[dcache_line].dirty {
                    cache::dcache_writeback(n64, dcache_line)
                }
                n64.memory.dcache[dcache_line].valid = false
            }
        }
        0x19 => {
            //dcache hit write back
            if cache::dcache_hit(n64, dcache_line, phys_address)
                && n64.memory.dcache[dcache_line].dirty
            {
                cache::dcache_writeback(n64, dcache_line)
            }
        }
        _ => {
            panic!("unknown cache code {:#01x}", rt(opcode))
        }
    }
}

fn ll(n64: &mut N64,opcode: u32) {
    let (phys_address, cached, err) = memory::translate_address(
        n64,
        n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16)),
        memory::AccessType::Read,
    );
    if err {
        return;
    }

    n64.cpu.gpr[rt(opcode) as usize] = se32(memory::read_data(
        n64,
        phys_address,
        memory::AccessSize::Word,
        cached,
    ) as i32);
    n64.cpu.llbit = true;
    n64.cpu.mmu.regs[MmuRegister::LLAddr as usize] = phys_address >> 4
}

fn lld(n64: &mut N64,opcode: u32) {
    let (phys_address, cached, err) = memory::translate_address(
        n64,
        n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16)),
        memory::AccessType::Read,
    );
    if err {
        return;
    }

    let mut w = [0; 2];
    w[0] = memory::read_data(
        n64,
        phys_address,
        memory::AccessSize::Dword,
        cached,
    );
    w[1] = memory::read_data(
        n64,
        phys_address + 4,
        memory::AccessSize::Dword,
        cached,
    );
    n64.cpu.gpr[rt(opcode) as usize] = ((w[0] as u64) << 32) | (w[1]) as u64;

    n64.cpu.llbit = true;
    n64.cpu.mmu.regs[MmuRegister::LLAddr as usize] = phys_address >> 4
}

fn ld(n64: &mut N64,opcode: u32) {
    let (phys_address, cached, err) = memory::translate_address(
        n64,
        n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16)),
        memory::AccessType::Read,
    );
    if err {
        return;
    }

    let mut w = [0; 2];
    w[0] = memory::read_data(
        n64,
        phys_address,
        memory::AccessSize::Dword,
        cached,
    );
    w[1] = memory::read_data(
        n64,
        phys_address + 4,
        memory::AccessSize::Dword,
        cached,
    );
    n64.cpu.gpr[rt(opcode) as usize] = ((w[0] as u64) << 32) | (w[1]) as u64
}

fn sc(n64: &mut N64,opcode: u32) {
    if n64.cpu.llbit {
        let (phys_address, cached, err) = memory::translate_address(
            n64,
            n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16)),
            memory::AccessType::Write,
        );
        if err {
            return;
        }

        n64.cpu.llbit = false;
        memory::write_data(
            n64,
            phys_address,
            n64.cpu.gpr[rt(opcode) as usize] as u32,
            0xFFFFFFFF,
            cached,
        );
        n64.cpu.gpr[rt(opcode) as usize] = 1
    } else {
        n64.cpu.gpr[rt(opcode) as usize] = 0
    }
}

fn scd(n64: &mut N64,opcode: u32) {
    if n64.cpu.llbit {
        let (phys_address, cached, err) = memory::translate_address(
            n64,
            n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16)),
            memory::AccessType::Write,
        );
        if err {
            return;
        }

        n64.cpu.llbit = false;
        memory::write_data(
            n64,
            phys_address,
            (n64.cpu.gpr[rt(opcode) as usize] >> 32) as u32,
            0xFFFFFFFF,
            cached,
        );
        memory::write_data(
            n64,
            phys_address + 4,
            n64.cpu.gpr[rt(opcode) as usize] as u32,
            0xFFFFFFFF,
            cached,
        );
        n64.cpu.gpr[rt(opcode) as usize] = 1
    } else {
        n64.cpu.gpr[rt(opcode) as usize] = 0
    }
}

fn sd(n64: &mut N64,opcode: u32) {
    let (phys_address, cached, err) = memory::translate_address(
        n64,
        n64.cpu.gpr[rs(opcode) as usize].wrapping_add(se16(imm(opcode) as i16)),
        memory::AccessType::Write,
    );
    if err {
        return;
    }

    memory::write_data(
        n64,
        phys_address,
        (n64.cpu.gpr[rt(opcode) as usize] >> 32) as u32,
        0xFFFFFFFF,
        cached,
    );
    memory::write_data(
        n64,
        phys_address + 4,
        n64.cpu.gpr[rt(opcode) as usize] as u32,
        0xFFFFFFFF,
        cached,
    )
}

fn sll(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[rd(opcode) as usize] =
        se32(((n64.cpu.gpr[rt(opcode) as usize] as u32) << sa(opcode)) as i32)
}

fn srl(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[rd(opcode) as usize] =
        se32(((n64.cpu.gpr[rt(opcode) as usize] as u32) >> sa(opcode)) as i32)
}

fn sra(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[rd(opcode) as usize] =
        se32(((n64.cpu.gpr[rt(opcode) as usize] as i64) >> sa(opcode)) as i32)
}

fn sllv(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[rd(opcode) as usize] = se32(
        ((n64.cpu.gpr[rt(opcode) as usize] as u32)
            << (n64.cpu.gpr[rs(opcode) as usize] as u32 & 31)) as i32,
    )
}

fn srlv(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[rd(opcode) as usize] = se32(
        ((n64.cpu.gpr[rt(opcode) as usize] as u32)
            >> (n64.cpu.gpr[rs(opcode) as usize] as u32 & 31)) as i32,
    )
}

fn srav(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[rd(opcode) as usize] = se32(
        ((n64.cpu.gpr[rt(opcode) as usize] as i64) >> (n64.cpu.gpr[rs(opcode) as usize] & 31))
            as i32,
    )
}

fn jr(n64: &mut N64,opcode: u32) {
    if !cpu::in_delay_slot_taken(n64) {
        check_relative_idle_loop(n64, opcode);
        n64.cpu.branch_state.state = State::Take;
        n64.cpu.branch_state.pc = n64.cpu.gpr[rs(opcode) as usize];
    } else if !cpu::in_delay_slot(n64) {
        n64.cpu.branch_state.state = State::NotTaken;
    }
}

fn jalr(n64: &mut N64,opcode: u32) {
    let in_delay_slot_taken = cpu::in_delay_slot_taken(n64);

    if !in_delay_slot_taken {
        check_relative_idle_loop(n64, opcode);
        n64.cpu.branch_state.state = State::Take;
        n64.cpu.branch_state.pc = n64.cpu.gpr[rs(opcode) as usize];
    } else if !cpu::in_delay_slot(n64) {
        n64.cpu.branch_state.state = State::NotTaken;
    }

    if in_delay_slot_taken {
        n64.cpu.gpr[rd(opcode) as usize] = n64.cpu.branch_state.pc + 4
    } else {
        n64.cpu.gpr[rd(opcode) as usize] = n64.cpu.pc + 8
    }
}

fn sync(_device: &mut N64, _opcode: u32) {}

fn mfhi(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[rd(opcode) as usize] = n64.cpu.hi
}

fn mthi(n64: &mut N64,opcode: u32) {
    n64.cpu.hi = n64.cpu.gpr[rs(opcode) as usize]
}

fn mflo(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[rd(opcode) as usize] = n64.cpu.lo
}

fn mtlo(n64: &mut N64,opcode: u32) {
    n64.cpu.lo = n64.cpu.gpr[rs(opcode) as usize]
}

fn dsllv(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[rd(opcode) as usize] =
        n64.cpu.gpr[rt(opcode) as usize] << (n64.cpu.gpr[rs(opcode) as usize] & 63)
}

fn dsrlv(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[rd(opcode) as usize] =
        n64.cpu.gpr[rt(opcode) as usize] >> (n64.cpu.gpr[rs(opcode) as usize] & 63)
}

fn dsrav(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[rd(opcode) as usize] = ((n64.cpu.gpr[rt(opcode) as usize] as i64)
        >> (n64.cpu.gpr[rs(opcode) as usize] & 63))
        as u64
}

fn div(n64: &mut N64,opcode: u32) {
    if (n64.cpu.gpr[rt(opcode) as usize]) as i32 != 0 {
        n64.cpu.lo = se32(
            (n64.cpu.gpr[rs(opcode) as usize] as i32)
                .wrapping_div(n64.cpu.gpr[rt(opcode) as usize] as i32),
        );
        n64.cpu.hi = se32(
            (n64.cpu.gpr[rs(opcode) as usize] as i32)
                .wrapping_rem(n64.cpu.gpr[rt(opcode) as usize] as i32),
        );
    } else {
        if (n64.cpu.gpr[rs(opcode) as usize] as i32) < 0 {
            n64.cpu.lo = 1
        } else {
            n64.cpu.lo = u64::MAX
        }
        n64.cpu.hi = se32(n64.cpu.gpr[rs(opcode) as usize] as i32)
    }

    add_cycles(n64, 36)
}

fn divu(n64: &mut N64,opcode: u32) {
    if (n64.cpu.gpr[rt(opcode) as usize]) as u32 != 0 {
        n64.cpu.lo = se32(
            ((n64.cpu.gpr[rs(opcode) as usize] as u32)
                / (n64.cpu.gpr[rt(opcode) as usize] as u32)) as i32,
        );
        n64.cpu.hi = se32(
            ((n64.cpu.gpr[rs(opcode) as usize] as u32)
                % (n64.cpu.gpr[rt(opcode) as usize] as u32)) as i32,
        );
    } else {
        n64.cpu.lo = u64::MAX;
        n64.cpu.hi = se32(n64.cpu.gpr[rs(opcode) as usize] as u32 as i32)
    }

    add_cycles(n64, 36)
}

fn mult(n64: &mut N64,opcode: u32) {
    let result = ((n64.cpu.gpr[rs(opcode) as usize]) as i32 as i64)
        .wrapping_mul(n64.cpu.gpr[rt(opcode) as usize] as i32 as i64);

    n64.cpu.lo = se32((result) as i32);
    n64.cpu.hi = se32((result >> 32) as i32);

    add_cycles(n64, 4);
}

fn multu(n64: &mut N64,opcode: u32) {
    let result = (n64.cpu.gpr[rs(opcode) as usize] as u32 as u64)
        .wrapping_mul(n64.cpu.gpr[rt(opcode) as usize] as u32 as u64);

    n64.cpu.lo = se32((result) as u32 as i32);
    n64.cpu.hi = se32((result >> 32) as u32 as i32);

    add_cycles(n64, 4);
}


fn dmult(n64: &mut N64,opcode: u32) {
    let result = ((n64.cpu.gpr[rs(opcode) as usize] as i64 as i128)
        * (n64.cpu.gpr[rt(opcode) as usize] as i64 as i128)) as u128;
    n64.cpu.lo = result as u64;
    n64.cpu.hi = (result >> 64) as u64;
    add_cycles(n64, 7)
}

fn dmultu(n64: &mut N64,opcode: u32) {
    let result = (n64.cpu.gpr[rs(opcode) as usize] as u128)
        * (n64.cpu.gpr[rt(opcode) as usize] as u128);
    n64.cpu.lo = result as u64;
    n64.cpu.hi = (result >> 64) as u64;
    add_cycles(n64, 7)
}

fn ddiv(n64: &mut N64,opcode: u32) {
    if n64.cpu.gpr[rt(opcode) as usize] as i64 != 0 {
        n64.cpu.lo = (((n64.cpu.gpr[rs(opcode) as usize] as i64) as i128)
            / (n64.cpu.gpr[rt(opcode) as usize] as i64) as i128) as u64;
        n64.cpu.hi = (((n64.cpu.gpr[rs(opcode) as usize] as i64) as i128)
            % (n64.cpu.gpr[rt(opcode) as usize] as i64) as i128) as u64;
    } else {
        if ((n64.cpu.gpr[rs(opcode) as usize]) as i64) < 0 {
            n64.cpu.lo = 1
        } else {
            n64.cpu.lo = u64::MAX
        }
        n64.cpu.hi = n64.cpu.gpr[rs(opcode) as usize]
    }
    add_cycles(n64, 68);
}

fn ddivu(n64: &mut N64,opcode: u32) {
    if n64.cpu.gpr[rt(opcode) as usize] != 0 {
        n64.cpu.lo = n64.cpu.gpr[rs(opcode) as usize] / n64.cpu.gpr[rt(opcode) as usize];
        n64.cpu.hi = n64.cpu.gpr[rs(opcode) as usize] % n64.cpu.gpr[rt(opcode) as usize]
    } else {
        n64.cpu.lo = u64::MAX;
        n64.cpu.hi = n64.cpu.gpr[rs(opcode) as usize]
    }
    add_cycles(n64, 68);
}

macro_rules! cpu_arithmetic_op {
    ($name:ident, $op:tt) => {
        fn $name(n64: &mut N64,opcode: u32) {
            n64.cpu.gpr[rd(opcode) as usize] = se32(
                (n64.cpu.gpr[rs(opcode) as usize] as u32)
                    .$op(n64.cpu.gpr[rt(opcode) as usize] as u32) as i32,
            )
        }
    };
}

// Generate all arithmetic operations
cpu_arithmetic_op!(add, wrapping_add);
cpu_arithmetic_op!(addu, wrapping_add);
cpu_arithmetic_op!(sub, wrapping_sub);
cpu_arithmetic_op!(subu, wrapping_sub);

macro_rules! impl_bitwise_op {
    ($name:ident, $op:tt) => {
        fn $name(n64: &mut N64,opcode: u32) {
            n64.cpu.gpr[rd(opcode) as usize] = 
                n64.cpu.gpr[rs(opcode) as usize] $op n64.cpu.gpr[rt(opcode) as usize]
        }
    };
}

impl_bitwise_op!(and, &);
impl_bitwise_op!(or, |);
impl_bitwise_op!(xor, ^);

// NOR needs special handling due to the NOT operation
fn nor(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[rd(opcode) as usize] =
        !(n64.cpu.gpr[rs(opcode) as usize] | n64.cpu.gpr[rt(opcode) as usize])
}

fn slt(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[rd(opcode) as usize] = ((n64.cpu.gpr[rs(opcode) as usize] as i64)
        < (n64.cpu.gpr[rt(opcode) as usize] as i64))
        as u64
}

fn sltu(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[rd(opcode) as usize] =
        (n64.cpu.gpr[rs(opcode) as usize] < n64.cpu.gpr[rt(opcode) as usize]) as u64
}

macro_rules! def_d_arithmetic {
    ($name:ident, $op:tt) => {
        fn $name(n64: &mut N64,opcode: u32) {
            n64.cpu.gpr[rd(opcode) as usize] = 
                n64.cpu.gpr[rs(opcode) as usize].$op(n64.cpu.gpr[rt(opcode) as usize])
        }
    };
}

// Define arithmetic instructions
def_d_arithmetic!(dadd, wrapping_add);
def_d_arithmetic!(daddu, wrapping_add);
def_d_arithmetic!(dsub, wrapping_sub);
def_d_arithmetic!(dsubu, wrapping_sub);

macro_rules! def_trap_reg {
    ($name:ident, $op:tt) => {
        fn $name(n64: &mut N64,opcode: u32) {
            if (n64.cpu.gpr[rs(opcode) as usize] as i64) $op (n64.cpu.gpr[rt(opcode) as usize] as i64) {
                exceptions::trap_exception(n64)
            }
        }
    };
    ($name:ident, $op:tt, unsigned) => {
        fn $name(n64: &mut N64,opcode: u32) {
            if n64.cpu.gpr[rs(opcode) as usize] $op n64.cpu.gpr[rt(opcode) as usize] {
                exceptions::trap_exception(n64)
            }
        }
    };
}

// Define trap instructions using a macro
def_trap_reg!(tge, >=);
def_trap_reg!(tgeu, >=, unsigned);
def_trap_reg!(tlt, <);
def_trap_reg!(tltu, <, unsigned);
def_trap_reg!(teq, ==, unsigned);
def_trap_reg!(tne, !=, unsigned);

fn dsll(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[rd(opcode) as usize] = n64.cpu.gpr[rt(opcode) as usize] << sa(opcode)
}

fn dsrl(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[rd(opcode) as usize] = n64.cpu.gpr[rt(opcode) as usize] >> sa(opcode)
}

fn dsra(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[rd(opcode) as usize] =
        ((n64.cpu.gpr[rt(opcode) as usize] as i64) >> sa(opcode)) as u64
}

fn dsll32(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[rd(opcode) as usize] = n64.cpu.gpr[rt(opcode) as usize] << (32 + sa(opcode))
}

fn dsrl32(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[rd(opcode) as usize] = n64.cpu.gpr[rt(opcode) as usize] >> (32 + sa(opcode))
}

fn dsra32(n64: &mut N64,opcode: u32) {
    n64.cpu.gpr[rd(opcode) as usize] =
        ((n64.cpu.gpr[rt(opcode) as usize] as i64) >> (32 + sa(opcode))) as u64
}

macro_rules! branch_instruction {
    ($name:ident, $cmp:expr, $not_taken_state:expr) => {
        fn $name(n64: &mut N64,opcode: u32) {
            if $cmp(n64.cpu.gpr[rs(opcode) as usize] as i64) {
                check_relative_idle_loop(n64, opcode);
                n64.cpu.branch_state.state = State::Take;
                n64.cpu.branch_state.pc = n64.cpu.pc.wrapping_add(se16(imm(opcode) as i16) << 2) + 4;
            } else {
                n64.cpu.branch_state.state = $not_taken_state;
            }
        }
    };
}

// Define branch instructions using a macro
branch_instruction!(bltz, |x| x < 0, State::NotTaken);
branch_instruction!(bgez, |x| x >= 0, State::NotTaken);
branch_instruction!(bltzl, |x| x < 0, State::Discard);
branch_instruction!(bgezl, |x| x >= 0, State::Discard);

macro_rules! def_trap_imm {
    ($name:ident, $op:tt) => {
        fn $name(n64: &mut N64,opcode: u32) {
            if (n64.cpu.gpr[rs(opcode) as usize] as i64) $op (imm(opcode) as i16 as i64) {
                exceptions::trap_exception(n64)
            }
        }
    };
    ($name:ident, $op:tt, unsigned) => {
        fn $name(n64: &mut N64,opcode: u32) {
            if n64.cpu.gpr[rs(opcode) as usize] $op se16(imm(opcode) as i16) {
                exceptions::trap_exception(n64)
            }
        }
    };
}

// Define all trap immediate instructions
def_trap_imm!(tgei, >=);
def_trap_imm!(tgeiu, >=, unsigned);
def_trap_imm!(tlti, <);
def_trap_imm!(tltiu, <, unsigned);
def_trap_imm!(teqi, ==);
def_trap_imm!(tnei, !=);

macro_rules! impl_branch_and_link {
    ($name:ident, $condition:expr, $discard_on_false:literal) => {
        fn $name(n64: &mut N64,opcode: u32) {
            let rs_val = n64.cpu.gpr[rs(opcode) as usize] as i64;
            let in_delay_slot = cpu::in_delay_slot(n64);
            
            if $condition(rs_val) {
                check_relative_idle_loop(n64, opcode);
                n64.cpu.branch_state.state = State::Take;
                n64.cpu.branch_state.pc = n64.cpu.pc.wrapping_add(se16(imm(opcode) as i16) << 2) + 4;
            } else {
                n64.cpu.branch_state.state = if $discard_on_false {
                    State::Discard
                } else {
                    State::NotTaken
                };
            }

            n64.cpu.gpr[31] = if in_delay_slot {
                n64.cpu.branch_state.pc + 4
            } else {
                n64.cpu.pc + 8
            };
        }
    };
}

// Define the branch conditions
const BLTZ: fn(i64) -> bool = |x| x < 0;
const BGEZ: fn(i64) -> bool = |x| x >= 0;

// Implement the instructions
impl_branch_and_link!(bltzal, BLTZ, false);
impl_branch_and_link!(bgezal, BGEZ, false);
impl_branch_and_link!(bltzall, BLTZ, true);
impl_branch_and_link!(bgezall, BGEZ, true);
