use super::builder::memory::Memory;
use super::cpu::rt;
use super::mips::set_rcp_interrupt;
use super::rdram::rdram_calculate_cycles;
use crate::events::remove_event;
use crate::mips;
use crate::mips::{clear_rcp_interrupt, schedule_rcp_interrupt, MI_INTR_SP};
use crate::rdp;
use crate::rsp;
use std::arch::x86_64::*;

use super::events::{create_event, EventType};
use super::mmu::reserved;
use super::{memory, N64};

#[derive(PartialEq, Copy, Clone, Default)]
pub enum DmaDir {
    #[default]
    None,
    Write,
    Read,
}

#[derive(Copy, Clone, Default)]
pub struct RspDma {
    pub dir: DmaDir,
    pub length: u32,
    pub memaddr: u32,
    pub dramaddr: u32,
}

// Type aliases for clarity
type InstrFn = fn(&mut N64, u32);
type SimdRegister = std::arch::x86_64::__m128i;

/// Reality Signal Processor (RSP) - Nintendo 64's vector processing unit
///
/// General register state
pub struct RegisterState {
    pub regs: Registers,
    pub regs2: Registers,
    pub gpr: [u32; 32],
    pub vpr: [u128; 32],
}

/// Memory and DMA state
pub struct MemoryState {
    pub mem: Memory,
    pub fifo: [RspDma; 2],
}

/// Instruction state
pub struct InstructionState {
    pub instructions: [Instructions; 0x1000 / 4],
    pub last_instruction_type: InstructionType,
    pub instruction_type: InstructionType,
}

/// Processor execution state
pub struct ProcessorState {
    pub pipeline_full: bool,
    pub branch_state: BranchState,
    pub broken: bool,
    pub halted: bool,
    pub sync_point: bool,
    pub cycle_counter: u64,
}

/// Vector operation state
pub struct VectorState {
    pub shuffle: [SimdRegister; 16],
    pub reciprocals: [u16; 512],
    pub inverse_square_roots: [u16; 512],
}

/// Vector control registers
pub struct VectorControlState {
    pub vcol: SimdRegister,
    pub vcoh: SimdRegister,
    pub vccl: SimdRegister,
    pub vcch: SimdRegister,
    pub vce: SimdRegister,
}

/// Accumulator registers
pub struct AccumulatorState {
    pub accl: SimdRegister,
    pub accm: SimdRegister,
    pub acch: SimdRegister,
}

/// Division unit state
pub struct DivisionState {
    pub divdp: bool,
    pub divin: i16,
    pub divout: i16,
}

/// Main RSP struct using composed state
pub struct Rsp {
    pub registers: RegisterState,
    pub memory: MemoryState,
    pub instruction: InstructionState,
    pub processor: ProcessorState,
    pub vector: VectorState,
    pub vector_control: VectorControlState,
    pub accumulator: AccumulatorState,
    pub division: DivisionState,
    pub instruction_tables: InstructionTables,
}

/// Instruction lookup tables for different RSP operation types
pub struct InstructionTables {
    /// Special instructions (SPECIAL opcode)
    pub special_table: [InstrFn; 64],

    /// Register immediate instructions (REGIMM opcode)
    pub register_immediate_table: [InstrFn; 32],

    /// Memory Management Unit instructions
    pub memory_management_table: [InstrFn; 32],

    /// Coprocessor 2 instructions
    pub rsp_table: [InstrFn; 32],

    /// Load Word Coprocessor 2 instructions
    pub load_word_rsp_table: [InstrFn; 32],

    /// Store Word Coprocessor 2 instructions
    pub store_word_rsp_table: [InstrFn; 32],

    /// Primary instruction table
    pub primary_table: [InstrFn; 64],

    /// Vector instruction table
    pub vector_table: [InstrFn; 64],
}

impl Default for InstructionTables {
    fn default() -> Self {
        let default_instruction: InstrFn = |_n64, _insn| {};

        Self {
            special_table: [default_instruction; 64],
            register_immediate_table: [default_instruction; 32],
            memory_management_table: [default_instruction; 32],
            rsp_table: [default_instruction; 32],
            load_word_rsp_table: [default_instruction; 32],
            store_word_rsp_table: [default_instruction; 32],
            primary_table: [default_instruction; 64],
            vector_table: [default_instruction; 64],
        }
    }
}

impl Default for RegisterState {
    fn default() -> Self {
        Self {
            regs: Registers::new()
                .with_size(SpReg::RegsCount as usize)
                .build(),
            regs2: Registers::new()
                .with_size(SpReg2::Regs2Count as usize)
                .build(),
            gpr: [0; 32],
            vpr: [0; 32],
        }
    }
}

impl Default for MemoryState {
    fn default() -> Self {
        Self {
            mem: Memory::new().with_size(0x2000).build(),
            fifo: [RspDma::default(); 2],
        }
    }
}

impl Default for InstructionState {
    fn default() -> Self {
        Self {
            instructions: [Instructions::default(); 0x1000 / 4],
            last_instruction_type: InstructionType::default(),
            instruction_type: InstructionType::default(),
        }
    }
}

impl Default for ProcessorState {
    fn default() -> Self {
        Self {
            pipeline_full: false,
            branch_state: BranchState::default(),
            broken: false,
            halted: false,
            sync_point: false,
            cycle_counter: 0,
        }
    }
}

impl Default for VectorState {
    fn default() -> Self {
        Self {
            shuffle: [unsafe { std::mem::zeroed() }; 16],
            reciprocals: [0; 512],
            inverse_square_roots: [0; 512],
        }
    }
}

impl Default for VectorControlState {
    fn default() -> Self {
        Self {
            vcol: unsafe { std::mem::zeroed() },
            vcoh: unsafe { std::mem::zeroed() },
            vccl: unsafe { std::mem::zeroed() },
            vcch: unsafe { std::mem::zeroed() },
            vce: unsafe { std::mem::zeroed() },
        }
    }
}

impl Default for AccumulatorState {
    fn default() -> Self {
        Self {
            accl: unsafe { std::mem::zeroed() },
            accm: unsafe { std::mem::zeroed() },
            acch: unsafe { std::mem::zeroed() },
        }
    }
}

impl Default for DivisionState {
    fn default() -> Self {
        Self {
            divdp: false,
            divin: 0,
            divout: 0,
        }
    }
}

impl Default for Rsp {
    fn default() -> Self {
        let mut rsp = Rsp {
            registers: RegisterState::default(),
            memory: MemoryState::default(),
            instruction: InstructionState::default(),
            processor: ProcessorState::default(),
            vector: VectorState::default(),
            vector_control: VectorControlState::default(),
            accumulator: AccumulatorState::default(),
            division: DivisionState::default(),
            instruction_tables: InstructionTables::default(),
        };

        rsp.registers.regs[SpReg::Status as usize] = 1;

        rsp
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub enum CpuState {
    #[default]
    Step,
    Take,
    NotTaken,
    DelaySlotTaken,
    DelaySlotNotTaken,
}

#[derive(Copy, Clone, Default)]
pub struct BranchState {
    pub state: CpuState,
    pub pc: u32,
}

#[derive(Copy, Clone)]
pub struct Instructions {
    pub func: fn(&mut N64, u32),
    pub opcode: u32,
}

impl Default for Instructions {
    fn default() -> Self {
        fn default_func(_device: &mut N64, _value: u32) {}

        Instructions {
            func: default_func,
            opcode: 0,
        }
    }
}
#[derive(PartialEq, Copy, Clone, Default)]
pub enum InstructionType {
    #[default]
    Su,
    Vu,
}

#[derive(Debug)]
pub enum OpcodeType {
    Special,
    RegImm,
    MMU,
    Cop2,
    Lwc2,
    Swc2,
    Other(u8),
}

impl From<u32> for OpcodeType {
    fn from(opcode: u32) -> Self {
        match opcode >> 26 {
            0 => OpcodeType::Special,
            1 => OpcodeType::RegImm,
            16 => OpcodeType::MMU,
            18 => OpcodeType::Cop2,
            50 => OpcodeType::Lwc2,
            58 => OpcodeType::Swc2,
            other => OpcodeType::Other(other as u8),
        }
    }
}

fn in_delay_slot(n64: &N64) -> bool {
    matches!(
        n64.rsp.processor.branch_state.state,
        CpuState::DelaySlotTaken | CpuState::DelaySlotNotTaken
    )
}

fn in_delay_slot_taken(n64: &N64) -> bool {
    n64.rsp.processor.branch_state.state == CpuState::DelaySlotTaken
}

fn run(n64: &mut N64) -> u64 {
    n64.rsp.processor.broken = false;
    n64.rsp.processor.cycle_counter = 0;

    while !n64.rsp.processor.sync_point {
        n64.rsp.instruction.instruction_type = InstructionType::Su;
        n64.rsp.registers.gpr[0] = 0;

        let pc = n64.rsp.registers.regs2[SpReg2::Pc as usize] / 4;
        let instruction = n64.rsp.instruction.instructions[pc as usize];
        (instruction.func)(n64, instruction.opcode);

        let pc_reg = &mut n64.rsp.registers.regs2[SpReg2::Pc as usize];
        match n64.rsp.processor.branch_state.state {
            CpuState::Step => *pc_reg += 4,
            CpuState::Take => {
                *pc_reg += 4;
                n64.rsp.processor.branch_state.state = CpuState::DelaySlotTaken;
            }
            CpuState::NotTaken => {
                *pc_reg += 4;
                n64.rsp.processor.branch_state.state = CpuState::DelaySlotNotTaken;
            }
            CpuState::DelaySlotTaken => {
                *pc_reg = n64.rsp.processor.branch_state.pc;
                n64.rsp.processor.branch_state.state = CpuState::Step;
            }
            CpuState::DelaySlotNotTaken => {
                *pc_reg += 4;
                n64.rsp.processor.branch_state.state = CpuState::Step;
            }
        }

        *pc_reg &= 0xFFC;

        if n64.rsp.instruction.instruction_type == n64.rsp.instruction.instruction_type {
            n64.rsp.processor.cycle_counter += 1;
            n64.rsp.processor.pipeline_full = false;
        } else {
            n64.rsp.instruction.instruction_type = n64.rsp.instruction.instruction_type;
            if n64.rsp.processor.pipeline_full {
                n64.rsp.processor.cycle_counter += 1;
                n64.rsp.processor.pipeline_full = false;
            } else {
                n64.rsp.processor.pipeline_full = true;
            }
        }

        if n64.rsp.processor.broken {
            break;
        }
    }

    (n64.rsp.processor.cycle_counter as f64 * 1.5) as u64
}

fn decode_opcode(n64: &N64, opcode: u32) -> fn(&mut N64, u32) {
    match OpcodeType::from(opcode) {
        OpcodeType::Special => n64.rsp.instruction_tables.special_table[(opcode & 0x3F) as usize], // SPECIAL
        OpcodeType::RegImm => {
            n64.rsp.instruction_tables.register_immediate_table[((opcode >> 16) & 0x1F) as usize]
        } // REGIMM
        OpcodeType::MMU => {
            n64.rsp.instruction_tables.memory_management_table[((opcode >> 21) & 0x1F) as usize]
        } // MMU
        OpcodeType::Cop2 => n64.rsp.instruction_tables.rsp_table[((opcode >> 21) & 0x1F) as usize], // RCP
        OpcodeType::Lwc2 => {
            n64.rsp.instruction_tables.load_word_rsp_table[((opcode >> 11) & 0x1F) as usize]
        } // LWC2
        OpcodeType::Swc2 => {
            n64.rsp.instruction_tables.store_word_rsp_table[((opcode >> 11) & 0x1F) as usize]
        } // SWC2
        OpcodeType::Other(op) => n64.rsp.instruction_tables.primary_table[op as usize],
    }
}

pub fn start_rsp(n64: &mut N64) {
    n64.rsp.vector.reciprocals[0] = u16::MAX;

    for index in 1..512 {
        let a = (index + 512) as u64;
        let b = (1_u64 << 34) / a;
        n64.rsp.vector.reciprocals[index] = ((b + 1) >> 8) as u16;
    }

    for index in 0..512 {
        let shift = if index % 2 == 1 { 1 } else { 0 };
        let a = ((index + 512) >> shift) as u64;
        let mut b = (1 << 17) as u64;
        // Find the largest b where b < 1.0 / sqrt(a)
        while a * (b + 1) * (b + 1) < (1_u64 << 44) {
            b += 1;
        }
        n64.rsp.vector.inverse_square_roots[index] = (b >> 1) as u16;
    }

    n64.rsp.vector.shuffle = unsafe {
        let mut shuffle = [_mm_set_epi8(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0); 16];

        // Vector
        shuffle[0] = _mm_set_epi8(15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0);
        shuffle[1] = _mm_set_epi8(15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0);

        // Scalar quarter
        shuffle[2] = _mm_set_epi8(15, 14, 15, 14, 11, 10, 11, 10, 7, 6, 7, 6, 3, 2, 3, 2);
        shuffle[3] = _mm_set_epi8(13, 12, 13, 12, 9, 8, 9, 8, 5, 4, 5, 4, 1, 0, 1, 0);

        // Scalar half
        for i in 0..4 {
            shuffle[4 + i] = _mm_set_epi8(
                (15 - 2 * i) as i8,
                (14 - 2 * i) as i8,
                (15 - 2 * i) as i8,
                (14 - 2 * i) as i8,
                (15 - 2 * i) as i8,
                (14 - 2 * i) as i8,
                (15 - 2 * i) as i8,
                (14 - 2 * i) as i8,
                (7 - 2 * i) as i8,
                (6 - 2 * i) as i8,
                (7 - 2 * i) as i8,
                (6 - 2 * i) as i8,
                (7 - 2 * i) as i8,
                (6 - 2 * i) as i8,
                (7 - 2 * i) as i8,
                (6 - 2 * i) as i8,
            );
        }

        // Scalar whole
        for i in 0..8 {
            shuffle[8 + i] = _mm_set_epi8(
                (15 - 2 * i) as i8,
                (14 - 2 * i) as i8,
                (15 - 2 * i) as i8,
                (14 - 2 * i) as i8,
                (15 - 2 * i) as i8,
                (14 - 2 * i) as i8,
                (15 - 2 * i) as i8,
                (14 - 2 * i) as i8,
                (15 - 2 * i) as i8,
                (14 - 2 * i) as i8,
                (15 - 2 * i) as i8,
                (14 - 2 * i) as i8,
                (15 - 2 * i) as i8,
                (14 - 2 * i) as i8,
                (15 - 2 * i) as i8,
                (14 - 2 * i) as i8,
            );
        }

        shuffle
    };

    // Initialize all instructions to reserved by default
    n64.rsp.instruction_tables.primary_table = [reserved; 64];

    // Set specific instructions
    n64.rsp.instruction_tables.primary_table[2] = j;
    n64.rsp.instruction_tables.primary_table[3] = jal;
    n64.rsp.instruction_tables.primary_table[4] = beq;
    n64.rsp.instruction_tables.primary_table[5] = bne;
    n64.rsp.instruction_tables.primary_table[6] = blez;
    n64.rsp.instruction_tables.primary_table[7] = bgtz;
    n64.rsp.instruction_tables.primary_table[8] = addi;
    n64.rsp.instruction_tables.primary_table[9] = addiu;
    n64.rsp.instruction_tables.primary_table[10] = slti;
    n64.rsp.instruction_tables.primary_table[11] = sltiu;
    n64.rsp.instruction_tables.primary_table[12] = andi;
    n64.rsp.instruction_tables.primary_table[13] = ori;
    n64.rsp.instruction_tables.primary_table[14] = xori;
    n64.rsp.instruction_tables.primary_table[15] = lui;
    n64.rsp.instruction_tables.primary_table[32] = lb;
    n64.rsp.instruction_tables.primary_table[33] = lh;
    n64.rsp.instruction_tables.primary_table[35] = lw;
    n64.rsp.instruction_tables.primary_table[36] = lbu;
    n64.rsp.instruction_tables.primary_table[37] = lhu;
    n64.rsp.instruction_tables.primary_table[39] = lwu;
    n64.rsp.instruction_tables.primary_table[40] = sb;
    n64.rsp.instruction_tables.primary_table[41] = sh;
    n64.rsp.instruction_tables.primary_table[43] = sw;

    // Initialize all special instructions to reserved by default
    n64.rsp.instruction_tables.special_table = [reserved; 64];

    // Set specific special instructions
    let special_instrs = [
        (0, sll as fn(&mut N64, _)),
        (2, srl as fn(&mut N64, _)),
        (3, sra as fn(&mut N64, _)),
        (4, sllv as fn(&mut N64, _)),
        (6, srlv as fn(&mut N64, _)),
        (7, srav as fn(&mut N64, _)),
        (8, jr as fn(&mut N64, _)),
        (9, jalr as fn(&mut N64, _)),
        (13, break_ as fn(&mut N64, _)),
        (32, add as fn(&mut N64, _)),
        (33, addu as fn(&mut N64, _)),
        (34, sub as fn(&mut N64, _)),
        (35, subu as fn(&mut N64, _)),
        (36, and as fn(&mut N64, _)),
        (37, or as fn(&mut N64, _)),
        (38, xor as fn(&mut N64, _)),
        (39, nor as fn(&mut N64, _)),
        (42, slt as fn(&mut N64, _)),
        (43, sltu as fn(&mut N64, _)),
    ];

    for (index, instr) in special_instrs.iter() {
        n64.rsp.instruction_tables.special_table[*index] = *instr;
    }

    // Initialize all regimm instructions to reserved by default
    n64.rsp.instruction_tables.register_immediate_table = [reserved; 32];

    // Set specific regimm instructions
    let regimm_instrs: [(usize, fn(&mut N64, _)); 4] =
        [(0, bltz), (1, bgez), (16, bltzal), (17, bgezal)];

    for (index, instr) in regimm_instrs.iter() {
        n64.rsp.instruction_tables.register_immediate_table[*index] = *instr;
    }

    n64.rsp.instruction_tables.memory_management_table = [reserved; 32];

    n64.rsp.instruction_tables.memory_management_table[0] = mfc0;
    n64.rsp.instruction_tables.memory_management_table[4] = mtc0;

    n64.rsp.instruction_tables.rsp_table = [reserved; 32];

    n64.rsp.instruction_tables.rsp_table[0] = mfc2;
    n64.rsp.instruction_tables.rsp_table[2] = cfc2;
    n64.rsp.instruction_tables.rsp_table[4] = mtc2;
    n64.rsp.instruction_tables.rsp_table[6] = ctc2;
    for i in 16..32 {
        n64.rsp.instruction_tables.rsp_table[i] = execute_vec;
    }

    n64.rsp.instruction_tables.load_word_rsp_table = [reserved; 32];

    let lwc2_instrs: [(usize, fn(&mut N64, _)); 12] = [
        (0, lbv),
        (1, lsv),
        (2, llv),
        (3, ldv),
        (4, lqv),
        (5, lrv),
        (6, lpv),
        (7, luv),
        (8, lhv),
        (9, lfv),
        (10, lwv),
        (11, ltv),
    ];

    for (index, instr) in lwc2_instrs.iter() {
        n64.rsp.instruction_tables.load_word_rsp_table[*index] = *instr;
    }

    n64.rsp.instruction_tables.store_word_rsp_table = [reserved; 32];

    let swc2_instrs: [(usize, fn(&mut N64, _)); 12] = [
        (0, sbv),
        (1, ssv),
        (2, slv),
        (3, sdv),
        (4, sqv),
        (5, srv),
        (6, spv),
        (7, suv),
        (8, shv),
        (9, sfv),
        (10, swv),
        (11, stv),
    ];

    for (index, instr) in swc2_instrs.iter() {
        n64.rsp.instruction_tables.store_word_rsp_table[*index] = *instr
    }

    n64.rsp.instruction_tables.vector_table = [vzero; 64];

    // Set specific vec instructions
    let vec_instrs: [(usize, fn(&mut N64, _)); 46] = [
        (0, vmulf),
        (1, vmulu),
        (2, vrndp),
        (3, vmulq),
        (4, vmudl),
        (5, vmudm),
        (6, vmudn),
        (7, vmudh),
        (8, vmacf),
        (9, vmacu),
        (10, vrndn),
        (11, vmacq),
        (12, vmadl),
        (13, vmadm),
        (14, vmadn),
        (15, vmadh),
        (16, vadd),
        (17, vsub),
        (18, vzero),
        (19, vabs),
        (20, vaddc),
        (21, vsubc),
        (29, vsar),
        (32, vlt),
        (33, veq),
        (34, vne),
        (35, vge),
        (36, vcl),
        (37, vch),
        (38, vcr),
        (39, vmrg),
        (40, vand),
        (41, vnand),
        (42, vor),
        (43, vnor),
        (44, vxor),
        (45, vnxor),
        (48, vrcp),
        (49, vrcpl),
        (50, vrcph),
        (51, vmov),
        (52, vrsq),
        (53, vrsql),
        (54, vrsqh),
        (55, vnop),
        (63, vnop),
    ];

    for (index, instr) in vec_instrs.iter() {
        n64.rsp.instruction_tables.vector_table[*index] = *instr
    }
}

use super::{builder::registers::Registers, mmu::MmuRegister};

#[repr(u32)]
pub enum SpReg {
    MemAddr = 0,
    DramAddr = 1,
    RdLen = 2,
    WrLen = 3,
    Status = 4,
    DmaFull = 5,
    DmaBusy = 6,
    Semaphore = 7,
    RegsCount = 8,
}

#[repr(u32)]
pub enum SpReg2 {
    Pc = 0,
    // Ibist = 1,
    Regs2Count = 2,
}

#[repr(u32)]
pub enum SpStatus {
    Halt = 1 << 0,
    Broke = 1 << 1,
    DmaBusy = 1 << 2,
    DmaFull = 1 << 3,
    // IoFull = 1 << 4,
    Sstep = 1 << 5,
    IntrBreak = 1 << 6,
    Sig0 = 1 << 7,
    Sig1 = 1 << 8,
    Sig2 = 1 << 9,
    Sig3 = 1 << 10,
    Sig4 = 1 << 11,
    Sig5 = 1 << 12,
    Sig6 = 1 << 13,
    Sig7 = 1 << 14,
}

#[repr(u32)]
pub enum SpStatusWrite {
    ClrHalt = 1 << 0,
    SetHalt = 1 << 1,
    ClrBroke = 1 << 2,
    ClrIntr = 1 << 3,
    SetIntr = 1 << 4,
    ClrSstep = 1 << 5,
    SetSstep = 1 << 6,
    ClrIntrBreak = 1 << 7,
    SetIntrBreak = 1 << 8,
    ClrSig0 = 1 << 9,
    SetSig0 = 1 << 10,
    ClrSig1 = 1 << 11,
    SetSig1 = 1 << 12,
    ClrSig2 = 1 << 13,
    SetSig2 = 1 << 14,
    ClrSig3 = 1 << 15,
    SetSig3 = 1 << 16,
    ClrSig4 = 1 << 17,
    SetSig4 = 1 << 18,
    ClrSig5 = 1 << 19,
    SetSig5 = 1 << 20,
    ClrSig6 = 1 << 21,
    SetSig6 = 1 << 22,
    ClrSig7 = 1 << 23,
    SetSig7 = 1 << 24,
}

pub const RSP_MEM_MASK: usize = 0x1FFF;

pub fn read_mem_fast(n64: &mut N64, address: u64, _access_size: memory::AccessSize) -> u32 {
    let masked_address = address as usize & RSP_MEM_MASK;
    u32::from_be_bytes(
        n64.rsp.memory.mem[masked_address..masked_address + 4]
            .try_into()
            .unwrap(),
    )
}

pub fn read_mem(n64: &mut N64, address: u64, _access_size: memory::AccessSize) -> u32 {
    let masked_address = address as usize & RSP_MEM_MASK;
    u32::from_be_bytes(
        n64.rsp.memory.mem[masked_address..masked_address + 4]
            .try_into()
            .unwrap(),
    )
}

pub fn write_mem(n64: &mut N64, address: u64, value: u32, _mask: u32) {
    let masked_address = address as usize & RSP_MEM_MASK;
    let mut data = u32::from_be_bytes(
        n64.rsp.memory.mem[masked_address..masked_address + 4]
            .try_into()
            .unwrap(),
    );
    memory::masked_write_32(&mut data, value, 0xFFFFFFFF);
    n64.rsp.memory.mem[masked_address..masked_address + 4].copy_from_slice(&data.to_be_bytes());

    if masked_address & 0x1000 != 0 {
        n64.rsp.instruction.instructions[(masked_address & ADDR_MASK as usize) / 4].func =
            rsp::decode_opcode(n64, data);
        n64.rsp.instruction.instructions[(masked_address & ADDR_MASK as usize) / 4].opcode = data;
    }
}

fn do_dma(n64: &mut N64, dma: RspDma) {
    let l = dma.length;
    let length = ((l & ADDR_MASK) | 7) + 1;
    let count = ((l >> 12) & 0xff) + 1;
    let skip = (l >> 20) & 0xff8;
    let mut mem_addr = dma.memaddr & 0xff8;
    let mut dram_addr = dma.dramaddr & 0xfffff8;
    let offset = dma.memaddr & 0x1000;

    let read_data = |n64: &N64, addr: usize, len: usize| -> u32 {
        u32::from_be_bytes(n64.rsp.memory.mem[addr..addr + len].try_into().unwrap())
    };

    let write_data = |n64: &mut N64, addr: usize, data: u32| {
        n64.rdram.mem[addr..addr + 4].copy_from_slice(&data.to_ne_bytes())
    };

    let mut process_dma = |n64: &mut N64, dir: DmaDir| {
        for _ in 0..count {
            for _ in (0..length).step_by(4) {
                let rsp_addr = (offset + (mem_addr & ADDR_MASK)) as usize;
                let wrapped_addr = (dram_addr as usize) & (n64.rdram.mem.len() - 1);

                if dir == DmaDir::Read {
                    let data = read_data(n64, rsp_addr, 4);
                    write_data(n64, wrapped_addr, data);
                } else {
                    let data = u32::from_ne_bytes(
                        n64.rdram.mem[wrapped_addr..wrapped_addr + 4]
                            .try_into()
                            .unwrap(),
                    );
                    if offset != 0 {
                        let instr_idx = ((mem_addr & ADDR_MASK) / 4) as usize;
                        n64.rsp.instruction.instructions[instr_idx].func =
                            rsp::decode_opcode(n64, data);
                        n64.rsp.instruction.instructions[instr_idx].opcode = data;
                    }
                    n64.rsp.memory.mem[rsp_addr..rsp_addr + 4].copy_from_slice(&data.to_be_bytes());
                }

                mem_addr += 4;
                dram_addr += 4;
            }
            dram_addr += skip;
        }
    };

    process_dma(n64, dma.dir);

    n64.rsp.registers.regs[SpReg::MemAddr as usize] =
        (mem_addr & ADDR_MASK) + (dma.memaddr & 0x1000);
    n64.rsp.registers.regs[SpReg::DramAddr as usize] = dram_addr;
    n64.rsp.registers.regs[SpReg::RdLen as usize] = 0xff8;
    n64.rsp.registers.regs[SpReg::WrLen as usize] = 0xff8;

    create_event(
        n64,
        EventType::SPDma,
        n64.cpu.mmu.regs[MmuRegister::Count as usize]
            + rdram_calculate_cycles((count * length) as u64)
            + 9,
        rsp::fifo_pop,
    );
}

fn fifo_push(n64: &mut N64, dir: DmaDir) {
    if n64.rsp.registers.regs[SpReg::DmaFull as usize] != 0 {
        panic!("RSP DMA already full")
    }

    if n64.rsp.registers.regs[SpReg::DmaBusy as usize] != 0 {
        n64.rsp.memory.fifo[1].dir = dir;
        if dir == DmaDir::Read {
            n64.rsp.memory.fifo[1].length = n64.rsp.registers.regs[SpReg::WrLen as usize]
        } else {
            n64.rsp.memory.fifo[1].length = n64.rsp.registers.regs[SpReg::RdLen as usize]
        }
        n64.rsp.memory.fifo[1].memaddr = n64.rsp.registers.regs[SpReg::MemAddr as usize];
        n64.rsp.memory.fifo[1].dramaddr = n64.rsp.registers.regs[SpReg::DramAddr as usize];
        n64.rsp.registers.regs[SpReg::DmaFull as usize] = 1;
        n64.rsp.registers.regs[SpReg::Status as usize] |= SpStatus::DmaFull as u32;
    } else {
        n64.rsp.memory.fifo[0].dir = dir;
        if dir == DmaDir::Read {
            n64.rsp.memory.fifo[0].length = n64.rsp.registers.regs[SpReg::WrLen as usize]
        } else {
            n64.rsp.memory.fifo[0].length = n64.rsp.registers.regs[SpReg::RdLen as usize]
        }
        n64.rsp.memory.fifo[0].memaddr = n64.rsp.registers.regs[SpReg::MemAddr as usize];
        n64.rsp.memory.fifo[0].dramaddr = n64.rsp.registers.regs[SpReg::DramAddr as usize];
        n64.rsp.registers.regs[SpReg::DmaBusy as usize] = 1;
        n64.rsp.registers.regs[SpReg::Status as usize] |= SpStatus::DmaBusy as u32;

        do_dma(n64, n64.rsp.memory.fifo[0])
    }
}

fn fifo_pop(n64: &mut N64) {
    if n64.rsp.registers.regs[SpReg::DmaFull as usize] != 0 {
        n64.rsp.memory.fifo[0].dir = n64.rsp.memory.fifo[1].dir;
        n64.rsp.memory.fifo[0].length = n64.rsp.memory.fifo[1].length;
        n64.rsp.memory.fifo[0].memaddr = n64.rsp.memory.fifo[1].memaddr;
        n64.rsp.memory.fifo[0].dramaddr = n64.rsp.memory.fifo[1].dramaddr;
        n64.rsp.registers.regs[SpReg::DmaFull as usize] = 0;
        n64.rsp.registers.regs[SpReg::Status as usize] &= !(SpStatus::DmaFull as u32);

        do_dma(n64, n64.rsp.memory.fifo[0])
    } else {
        n64.rsp.registers.regs[SpReg::DmaBusy as usize] = 0;
        n64.rsp.registers.regs[SpReg::Status as usize] &= !(SpStatus::DmaBusy as u32);
    }
}

pub fn read_regs(n64: &mut N64, address: u64, _access_size: memory::AccessSize) -> u32 {
    let reg = (address & 0xFFFF) >> 2;
    match reg as u32 {
        x if x == SpReg::Semaphore as u32 => {
            let value = n64.rsp.registers.regs[reg as usize];
            n64.rsp.registers.regs[reg as usize] = 1;
            value
        }
        _ => n64.rsp.registers.regs[reg as usize],
    }
}

pub fn write_regs(n64: &mut N64, address: u64, value: u32, mask: u32) {
    let reg = (address & 0xFFFF) >> 2;
    match reg as u32 {
        x if x == SpReg::Status as u32 => update_sp_status(n64, value),
        x if x == SpReg::RdLen as u32 => {
            memory::masked_write_32(&mut n64.rsp.registers.regs[reg as usize], value, mask);
            fifo_push(n64, DmaDir::Write)
        }
        x if x == SpReg::WrLen as u32 => {
            memory::masked_write_32(&mut n64.rsp.registers.regs[reg as usize], value, mask);
            fifo_push(n64, DmaDir::Read)
        }
        x if x == SpReg::Semaphore as u32 => {
            memory::masked_write_32(&mut n64.rsp.registers.regs[reg as usize], 0, mask)
        }
        _ => {
            memory::masked_write_32(&mut n64.rsp.registers.regs[reg as usize], value, mask);
        }
    }
}

pub fn read_regs2(n64: &mut N64, address: u64, _access_size: memory::AccessSize) -> u32 {
    n64.rsp.registers.regs2[((address & 0xFFFF) >> 2) as usize]
}

pub fn write_regs2(n64: &mut N64, address: u64, value: u32, mask: u32) {
    let reg = (address & 0xFFFF) >> 2;
    match reg as u32 {
        x if x == SpReg2::Pc as u32 => {
            memory::masked_write_32(
                &mut n64.rsp.registers.regs2[reg as usize],
                value & 0xFFC,
                mask,
            );
        }
        _ => memory::masked_write_32(&mut n64.rsp.registers.regs2[reg as usize], value, mask),
    }
}

fn update_sp_status(n64: &mut N64, w: u32) {
    let was_halted = n64.rsp.registers.regs[SpReg::Status as usize] & SpStatus::Halt as u32 != 0;

    fn update_status(n64: &mut N64, w: u32, clr: u32, set: u32, status: u32) {
        if w & clr != 0 && w & set == 0 {
            n64.rsp.registers.regs[SpReg::Status as usize] &= !status
        }
        if w & set != 0 && w & clr == 0 {
            n64.rsp.registers.regs[SpReg::Status as usize] |= status
        }
    }

    update_status(
        n64,
        w,
        SpStatusWrite::ClrHalt as u32,
        SpStatusWrite::SetHalt as u32,
        SpStatus::Halt as u32,
    );
    if w & SpStatusWrite::SetHalt as u32 != 0 && w & SpStatusWrite::ClrHalt as u32 == 0 {
        remove_event(n64, EventType::SP)
    }
    if w & SpStatusWrite::ClrBroke as u32 != 0 {
        n64.rsp.registers.regs[SpReg::Status as usize] &= !(SpStatus::Broke as u32)
    }
    if w & SpStatusWrite::ClrIntr as u32 != 0 && w & SpStatusWrite::SetIntr as u32 == 0 {
        clear_rcp_interrupt(n64, MI_INTR_SP)
    }
    if w & SpStatusWrite::SetIntr as u32 != 0 && w & SpStatusWrite::ClrIntr as u32 == 0 {
        schedule_rcp_interrupt(n64, MI_INTR_SP)
    }

    update_status(
        n64,
        w,
        SpStatusWrite::ClrSstep as u32,
        SpStatusWrite::SetSstep as u32,
        SpStatus::Sstep as u32,
    );
    update_status(
        n64,
        w,
        SpStatusWrite::ClrIntrBreak as u32,
        SpStatusWrite::SetIntrBreak as u32,
        SpStatus::IntrBreak as u32,
    );
    update_status(
        n64,
        w,
        SpStatusWrite::ClrSig0 as u32,
        SpStatusWrite::SetSig0 as u32,
        SpStatus::Sig0 as u32,
    );
    update_status(
        n64,
        w,
        SpStatusWrite::ClrSig1 as u32,
        SpStatusWrite::SetSig1 as u32,
        SpStatus::Sig1 as u32,
    );
    update_status(
        n64,
        w,
        SpStatusWrite::ClrSig2 as u32,
        SpStatusWrite::SetSig2 as u32,
        SpStatus::Sig2 as u32,
    );
    update_status(
        n64,
        w,
        SpStatusWrite::ClrSig3 as u32,
        SpStatusWrite::SetSig3 as u32,
        SpStatus::Sig3 as u32,
    );
    update_status(
        n64,
        w,
        SpStatusWrite::ClrSig4 as u32,
        SpStatusWrite::SetSig4 as u32,
        SpStatus::Sig4 as u32,
    );
    update_status(
        n64,
        w,
        SpStatusWrite::ClrSig5 as u32,
        SpStatusWrite::SetSig5 as u32,
        SpStatus::Sig5 as u32,
    );
    update_status(
        n64,
        w,
        SpStatusWrite::ClrSig6 as u32,
        SpStatusWrite::SetSig6 as u32,
        SpStatus::Sig6 as u32,
    );
    update_status(
        n64,
        w,
        SpStatusWrite::ClrSig7 as u32,
        SpStatusWrite::SetSig7 as u32,
        SpStatus::Sig7 as u32,
    );

    if n64.rsp.registers.regs[SpReg::Status as usize] & SpStatus::Halt as u32 == 0 && was_halted {
        n64.rsp.processor.broken = false;
        n64.rsp.processor.halted = false;
        n64.rsp.processor.sync_point = false;
        do_task(n64);
    }
}

fn do_task(n64: &mut N64) {
    let timer = rsp::run(n64);

    create_event(
        n64,
        EventType::SP,
        n64.cpu.mmu.regs[MmuRegister::Count as usize] + timer,
        rsp_event,
    )
}

fn rsp_event(n64: &mut N64) {
    if n64.rsp.processor.broken {
        n64.rsp.registers.regs[SpReg::Status as usize] |=
            SpStatus::Halt as u32 | SpStatus::Broke as u32;

        if n64.rsp.registers.regs[SpReg::Status as usize] & SpStatus::IntrBreak as u32 != 0 {
            set_rcp_interrupt(n64, mips::MI_INTR_SP)
        }
        return;
    }
    if n64.rsp.processor.halted {
        n64.rsp.registers.regs[SpReg::Status as usize] |= SpStatus::Halt as u32;
        return;
    }
    n64.rsp.processor.sync_point = false;
    do_task(n64)
}

/*
    RPS INSTRUCTIONS
*/

// Define a macro for field extraction
macro_rules! field {
    ($name:ident, $shift:expr, $mask:expr) => {
        #[inline(always)]
        fn $name(opcode: u32) -> u32 {
            (opcode >> $shift) & $mask
        }
    };
}

// Special cases for different return types
macro_rules! field_u16 {
    ($name:ident) => {
        #[inline(always)]
        fn $name(opcode: u32) -> u16 {
            opcode as u16
        }
    };
}

macro_rules! field_u8 {
    ($name:ident, $shift:expr, $mask:expr) => {
        #[inline(always)]
        fn $name(opcode: u32) -> u8 {
            ((opcode >> $shift) & $mask) as u8
        }
    };
}

// Define the instruction fields
field!(rd, 11, 0x1F);
field!(rs, 21, 0x1F);
field!(sa, 6, 0x1F);
field_u16!(imm);
field_u8!(voffset, 0, 0x7F);
field_u8!(velement, 7, 0xF);

// Keep se16 as is since it's a special case
#[inline(always)]
fn se16(value: i16) -> u32 {
    value as i32 as u32
}

fn sign_extend_7bit_offset(offset: u8, shift_amount: u32) -> u32 {
    let soffset = (((offset << 1) & 0x80) | offset) as i8;

    (((soffset) as i32) as u32) << shift_amount
}

fn modify_vpr_byte(vpr: &mut u128, value: u8, element: u8) {
    let pos = 15 - (element & 15);
    let mask = 0xFF << (pos * 8);
    *vpr &= !mask;
    *vpr |= (value as u128) << (pos * 8);
}

fn get_vpr_byte(vpr: u128, element: u8) -> u8 {
    let pos = 15 - (element & 15);
    (vpr >> (pos * 8)) as u8
}

fn modify_vpr_element(vpr: &mut u128, value: u16, element: u8) {
    let pos = 7 - (element & 7);
    let mask = 0xFFFF << (pos * 16);
    *vpr &= !mask;
    *vpr |= (value as u128) << (pos * 16);
}

fn get_vpr_element(vpr: u128, element: u8) -> u16 {
    let pos = 7 - (element & 7);
    (vpr >> (pos * 16)) as u16
}

// Macro for standard branch instructions
macro_rules! branch_if {
    ($name:ident, $condition:expr) => {
        fn $name(n64: &mut N64, opcode: u32) {
            if $condition(n64, opcode) {
                n64.rsp.processor.branch_state.state = CpuState::Take;
                n64.rsp.processor.branch_state.pc = n64.rsp.registers.regs2[SpReg2::Pc as usize]
                    .wrapping_add(se16(imm(opcode) as i16) << 2)
                    + 4;
            } else {
                n64.rsp.processor.branch_state.state = CpuState::NotTaken;
            }
        }
    };
}

// Implement condition functions
fn beq_condition(n64: &N64, opcode: u32) -> bool {
    n64.rsp.registers.gpr[rs(opcode) as usize] == n64.rsp.registers.gpr[rt(opcode) as usize]
}
fn bne_condition(n64: &N64, opcode: u32) -> bool {
    n64.rsp.registers.gpr[rs(opcode) as usize] != n64.rsp.registers.gpr[rt(opcode) as usize]
}
fn blez_condition(n64: &N64, opcode: u32) -> bool {
    n64.rsp.registers.gpr[rs(opcode) as usize] as i32 <= 0
}
fn bgtz_condition(n64: &N64, opcode: u32) -> bool {
    n64.rsp.registers.gpr[rs(opcode) as usize] as i32 > 0
}

// Generate branch instructions
branch_if!(beq, beq_condition);
branch_if!(bne, bne_condition);
branch_if!(blez, blez_condition);
branch_if!(bgtz, bgtz_condition);

// Jump instructions kept separate due to different logic

fn j(n64: &mut N64, opcode: u32) {
    if rsp::in_delay_slot_taken(n64) {
        return;
    }
    n64.rsp.processor.branch_state.state = CpuState::Take;
    n64.rsp.processor.branch_state.pc = (n64.rsp.registers.regs2[SpReg2::Pc as usize] + 4)
        & 0xF0000000
        | ((opcode & 0x3FFFFFF) << 2)
}

fn jal(n64: &mut N64, opcode: u32) {
    if rsp::in_delay_slot_taken(n64) {
        n64.rsp.registers.gpr[31] = (n64.rsp.processor.branch_state.pc + 4) & ADDR_MASK
    } else {
        n64.rsp.registers.gpr[31] = (n64.rsp.registers.regs2[SpReg2::Pc as usize] + 8) & ADDR_MASK
    }
    if !rsp::in_delay_slot_taken(n64) {
        n64.rsp.processor.branch_state.state = CpuState::Take;
        n64.rsp.processor.branch_state.pc = (n64.rsp.registers.regs2[SpReg2::Pc as usize] + 4)
            & 0xF0000000
            | ((opcode & 0x3FFFFFF) << 2)
    } else if !rsp::in_delay_slot(n64) {
        n64.rsp.processor.branch_state.state = CpuState::NotTaken;
    }
}

macro_rules! gen_imm_op {
    ($name:ident, $op:expr) => {
        fn $name(n64: &mut N64, opcode: u32) {
            n64.rsp.registers.gpr[rt(opcode) as usize] =
                $op(n64.rsp.registers.gpr[rs(opcode) as usize], imm(opcode));
        }
    };
}

macro_rules! gen_imm_op_signed {
    ($name:ident, $op:expr) => {
        fn $name(n64: &mut N64, opcode: u32) {
            n64.rsp.registers.gpr[rt(opcode) as usize] = $op(
                n64.rsp.registers.gpr[rs(opcode) as usize] as i32,
                imm(opcode) as i16 as i32,
            ) as u32;
        }
    };
}

gen_imm_op!(addi, |a: u32, b| a.wrapping_add(b as i16 as i32 as u32));
gen_imm_op!(addiu, |a: u32, b| a.wrapping_add(b as i16 as i32 as u32));
gen_imm_op_signed!(slti, |a, b| (a < b) as u32);
gen_imm_op!(sltiu, |a, b| (a < b as i16 as i32 as u32) as u32);
gen_imm_op!(andi, |a, b| a & b as u32);
gen_imm_op!(ori, |a, b| a | b as u32);
gen_imm_op!(xori, |a, b| a ^ b as u32);

fn lui(n64: &mut N64, opcode: u32) {
    n64.rsp.registers.gpr[rt(opcode) as usize] = (imm(opcode) as u32) << 16
}

macro_rules! gen_load_op {
    ($name:ident, $size:expr, $sign_extend:expr) => {
        fn $name(n64: &mut N64, opcode: u32) {
            let address = n64.rsp.registers.gpr[rs(opcode) as usize]
                .wrapping_add(imm(opcode) as i16 as i32 as u32);
            let mut w = [0; $size];
            for i in 0..$size {
                w[i] = n64.rsp.memory.mem[(address as usize + i) & ADDR_MASK as usize];
            }
            let value = match $size {
                1 => w[0] as u32,
                2 => (((w[0] as u16) << 8) | w[1] as u16) as u32,
                4 => (w[0] as u32) << 24 | (w[1] as u32) << 16 | (w[2] as u32) << 8 | (w[3] as u32),
                _ => unreachable!(),
            };
            n64.rsp.registers.gpr[rt(opcode) as usize] = if $sign_extend {
                match $size {
                    1 => value as i8 as i32 as u32,
                    2 => value as i16 as i32 as u32,
                    4 => value,
                    _ => unreachable!(),
                }
            } else {
                value
            };
        }
    };
}

gen_load_op!(lb, 1, true);
gen_load_op!(lh, 2, true);
gen_load_op!(lw, 4, true);
gen_load_op!(lbu, 1, false);
gen_load_op!(lhu, 2, false);
gen_load_op!(lwu, 4, false);

fn sb(n64: &mut N64, opcode: u32) {
    let address =
        n64.rsp.registers.gpr[rs(opcode) as usize].wrapping_add(imm(opcode) as i16 as i32 as u32);

    n64.rsp.memory.mem[address as usize & ADDR_MASK as usize] =
        (n64.rsp.registers.gpr[rt(opcode) as usize]) as u8;
}

fn sh(n64: &mut N64, opcode: u32) {
    let address =
        n64.rsp.registers.gpr[rs(opcode) as usize].wrapping_add(imm(opcode) as i16 as i32 as u32);

    n64.rsp.memory.mem[address as usize & ADDR_MASK as usize] =
        (n64.rsp.registers.gpr[rt(opcode) as usize] >> 8) as u8;
    n64.rsp.memory.mem[(address as usize + 1) & ADDR_MASK as usize] =
        (n64.rsp.registers.gpr[rt(opcode) as usize]) as u8;
}

fn sw(n64: &mut N64, opcode: u32) {
    let address =
        n64.rsp.registers.gpr[rs(opcode) as usize].wrapping_add(imm(opcode) as i16 as i32 as u32);

    n64.rsp.memory.mem[address as usize & ADDR_MASK as usize] =
        (n64.rsp.registers.gpr[rt(opcode) as usize] >> 24) as u8;
    n64.rsp.memory.mem[(address as usize + 1) & ADDR_MASK as usize] =
        (n64.rsp.registers.gpr[rt(opcode) as usize] >> 16) as u8;
    n64.rsp.memory.mem[(address as usize + 2) & ADDR_MASK as usize] =
        (n64.rsp.registers.gpr[rt(opcode) as usize] >> 8) as u8;
    n64.rsp.memory.mem[(address as usize + 3) & ADDR_MASK as usize] =
        (n64.rsp.registers.gpr[rt(opcode) as usize]) as u8;
}

macro_rules! gen_shift_op {
    ($name:ident, $op:expr, $use_sa:expr) => {
        fn $name(n64: &mut N64, opcode: u32) {
            let shift_amount = if $use_sa {
                sa(opcode)
            } else {
                n64.rsp.registers.gpr[rs(opcode) as usize] & 31
            };
            n64.rsp.registers.gpr[rd(opcode) as usize] =
                $op(n64.rsp.registers.gpr[rt(opcode) as usize], shift_amount);
        }
    };
}

gen_shift_op!(sll, |a, b| a << b, true);
gen_shift_op!(srl, |a, b| a >> b, true);
gen_shift_op!(sra, |a, b| ((a as i32) >> b) as u32, true);
gen_shift_op!(sllv, |a, b| a << b, false);
gen_shift_op!(srlv, |a, b| a >> b, false);
gen_shift_op!(srav, |a, b| ((a as i32) >> b) as u32, false);

fn jr(n64: &mut N64, opcode: u32) {
    if !rsp::in_delay_slot_taken(n64) {
        n64.rsp.processor.branch_state.state = CpuState::Take;
        n64.rsp.processor.branch_state.pc = n64.rsp.registers.gpr[rs(opcode) as usize]
    } else if !rsp::in_delay_slot(n64) {
        n64.rsp.processor.branch_state.state = CpuState::NotTaken;
    }
}

fn jalr(n64: &mut N64, opcode: u32) {
    let in_delay_slot_taken = rsp::in_delay_slot_taken(n64);

    if !in_delay_slot_taken {
        n64.rsp.processor.branch_state.state = CpuState::Take;
        n64.rsp.processor.branch_state.pc = n64.rsp.registers.gpr[rs(opcode) as usize]
    } else if !rsp::in_delay_slot(n64) {
        n64.rsp.processor.branch_state.state = CpuState::NotTaken;
    }

    if in_delay_slot_taken {
        n64.rsp.registers.gpr[rd(opcode) as usize] =
            (n64.rsp.processor.branch_state.pc + 4) & ADDR_MASK
    } else {
        n64.rsp.registers.gpr[rd(opcode) as usize] =
            (n64.rsp.registers.regs2[SpReg2::Pc as usize] + 8) & ADDR_MASK
    }
}

fn break_(n64: &mut N64, _opcode: u32) {
    n64.rsp.processor.broken = true;
}

macro_rules! gen_arith_op {
    ($name:ident, $op:expr) => {
        fn $name(n64: &mut N64, opcode: u32) {
            n64.rsp.registers.gpr[rd(opcode) as usize] = $op(
                n64.rsp.registers.gpr[rs(opcode) as usize],
                n64.rsp.registers.gpr[rt(opcode) as usize],
            );
        }
    };
}

gen_arith_op!(add, |a: u32, b| a.wrapping_add(b));
gen_arith_op!(addu, |a: u32, b| a.wrapping_add(b));
gen_arith_op!(sub, |a: u32, b| a.wrapping_sub(b));
gen_arith_op!(subu, |a: u32, b| a.wrapping_sub(b));

macro_rules! gen_bitwise_op {
    ($name:ident, $op:expr) => {
        fn $name(n64: &mut N64, opcode: u32) {
            n64.rsp.registers.gpr[rd(opcode) as usize] = $op(
                n64.rsp.registers.gpr[rs(opcode) as usize],
                n64.rsp.registers.gpr[rt(opcode) as usize],
            );
        }
    };
}

gen_bitwise_op!(and, |a, b| a & b);
gen_bitwise_op!(or, |a, b| a | b);
gen_bitwise_op!(xor, |a, b| a ^ b);
gen_bitwise_op!(nor, |a: u32, b: u32| !(a | b));

fn slt(n64: &mut N64, opcode: u32) {
    n64.rsp.registers.gpr[rd(opcode) as usize] =
        ((n64.rsp.registers.gpr[rs(opcode) as usize] as i32)
            < (n64.rsp.registers.gpr[rt(opcode) as usize] as i32)) as u32
}

fn sltu(n64: &mut N64, opcode: u32) {
    n64.rsp.registers.gpr[rd(opcode) as usize] = (n64.rsp.registers.gpr[rs(opcode) as usize]
        < n64.rsp.registers.gpr[rt(opcode) as usize])
        as u32
}

macro_rules! gen_branch_op {
    ($name:ident, $cond:expr, $link:expr) => {
        fn $name(n64: &mut N64, opcode: u32) {
            if $cond(n64.rsp.registers.gpr[rs(opcode) as usize] as i32) {
                n64.rsp.processor.branch_state.state = CpuState::Take;
                n64.rsp.processor.branch_state.pc = n64.rsp.registers.regs2[SpReg2::Pc as usize]
                    .wrapping_add(se16(imm(opcode) as i16) << 2)
                    + 4;
            } else {
                n64.rsp.processor.branch_state.state = CpuState::NotTaken;
            }
            if $link {
                n64.rsp.registers.gpr[31] =
                    (n64.rsp.registers.regs2[SpReg2::Pc as usize] + 8) & ADDR_MASK;
            }
        }
    };
}

gen_branch_op!(bltz, |a| a < 0, false);
gen_branch_op!(bgez, |a| a >= 0, false);
gen_branch_op!(bltzal, |a| a < 0, true);
gen_branch_op!(bgezal, |a| a >= 0, true);

fn mfc0(n64: &mut N64, opcode: u32) {
    if rd(opcode) < SpReg::RegsCount as u32 {
        n64.rsp.registers.gpr[rt(opcode) as usize] =
            rsp::read_regs(n64, (rd(opcode) << 2) as u64, memory::AccessSize::Word)
    } else {
        n64.rsp.registers.gpr[rt(opcode) as usize] = rdp::read_regs_dpc(
            n64,
            ((rd(opcode) as u32 - SpReg::RegsCount as u32) << 2) as u64,
            memory::AccessSize::Word,
        )
    }
    n64.rsp.processor.cycle_counter += 4; // needed for DK64
    n64.rsp.processor.sync_point = true;
}

fn mtc0(n64: &mut N64, opcode: u32) {
    if rd(opcode) < SpReg::RegsCount as u32 {
        rsp::write_regs(
            n64,
            (rd(opcode) << 2) as u64,
            n64.rsp.registers.gpr[rt(opcode) as usize],
            0xFFFFFFFF,
        )
    } else {
        rdp::write_regs_dpc(
            n64,
            ((rd(opcode) as u32 - SpReg::RegsCount as u32) << 2) as u64,
            n64.rsp.registers.gpr[rt(opcode) as usize],
            0xFFFFFFFF,
        )
    }
    if rd(opcode) == SpReg::Status as u32
        && n64.rsp.registers.gpr[rt(opcode) as usize] & SpStatusWrite::SetHalt as u32 != 0
    {
        n64.rsp.registers.regs[SpReg::Status as usize] &= !(SpStatus::Halt as u32); // set halt when event happens
        n64.rsp.processor.halted = true; // the RSP can halt itself by setting SP_SET_HALT
    }
    n64.rsp.processor.sync_point = true;
}

fn mfc2(n64: &mut N64, opcode: u32) {
    let hi = get_vpr_byte(n64.rsp.registers.vpr[rd(opcode) as usize], velement(opcode));
    let lo = get_vpr_byte(
        n64.rsp.registers.vpr[rd(opcode) as usize],
        velement(opcode) + 1,
    );
    n64.rsp.registers.gpr[rt(opcode) as usize] =
        ((hi as u16) << 8 | (lo as u16)) as i16 as i32 as u32
}

fn cfc2(n64: &mut N64, opcode: u32) {
    let hi;
    let lo;
    let mut zero = unsafe { _mm_setzero_si128() };
    match rd(opcode) & 3 {
        0x00 => {
            hi = &mut n64.rsp.vector_control.vcoh;
            lo = &mut n64.rsp.vector_control.vcol;
        }
        0x01 => {
            hi = &mut n64.rsp.vector_control.vcch;
            lo = &mut n64.rsp.vector_control.vccl;
        }
        0x02 | 0x03 => {
            hi = &mut zero;
            lo = &mut n64.rsp.vector_control.vce;
        }
        _ => {
            panic!("unknown ctc2")
        }
    }

    unsafe {
        let reverse = _mm_set_epi8(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15);
        n64.rsp.registers.gpr[rt(opcode) as usize] =
            (_mm_movemask_epi8(_mm_shuffle_epi8(_mm_packs_epi16(*hi, *lo), reverse))) as i16 as u32;
    }
}

fn mtc2(n64: &mut N64, opcode: u32) {
    modify_vpr_byte(
        &mut n64.rsp.registers.vpr[rd(opcode) as usize],
        (n64.rsp.registers.gpr[rt(opcode) as usize] >> 8) as u8,
        velement(opcode),
    );
    if velement(opcode) != 15 {
        modify_vpr_byte(
            &mut n64.rsp.registers.vpr[rd(opcode) as usize],
            n64.rsp.registers.gpr[rt(opcode) as usize] as u8,
            velement(opcode) + 1,
        );
    }
}

fn ctc2(n64: &mut N64, opcode: u32) {
    unsafe {
        let zero = _mm_setzero_si128();
        let rt_val = n64.rsp.registers.gpr[rt(opcode) as usize];
        let mask = _mm_set_epi16(
            0x0101,
            0x0202,
            0x0404,
            0x0808,
            0x1010,
            0x2020,
            0x4040,
            0x8080u16 as i16,
        );

        let computed_value =
            _mm_cmpeq_epi8(_mm_and_si128(_mm_set1_epi8(!rt_val as i8), mask), zero);

        let computed_high = _mm_cmpeq_epi8(
            _mm_and_si128(_mm_set1_epi8(!(rt_val >> 8) as i8), mask),
            zero,
        );

        // Direct assignment based on opcode
        match rd(opcode) & 3 {
            0x00 => {
                n64.rsp.vector_control.vcoh = computed_high;
                n64.rsp.vector_control.vcol = computed_value;
            }
            0x01 => {
                n64.rsp.vector_control.vcch = computed_high;
                n64.rsp.vector_control.vccl = computed_value;
            }
            0x02 | 0x03 => {
                n64.rsp.vector_control.vce = computed_value;
            }
            _ => panic!("Invalid CTC2 opcode"),
        }
    }
}

const VPR_SIZE: u8 = 16;
const ELEMENT_MASK: u32 = 15;

macro_rules! impl_vector_load {
    ($name:ident, $shift:expr, $elem_size:expr, $special_addr:expr, $max_elems:expr) => {
        fn $name(n64: &mut N64, opcode: u32) {
            let base_address = n64.rsp.registers.gpr[rs(opcode) as usize]
                .wrapping_add(sign_extend_7bit_offset(voffset(opcode), $shift));

            let initial_element = velement(opcode);
            let start_element = if $special_addr {
                VPR_SIZE.wrapping_sub(
                    ((base_address & ELEMENT_MASK) as u8).wrapping_sub(initial_element),
                )
            } else {
                initial_element
            };

            let aligned_address = if $special_addr {
                base_address & !ELEMENT_MASK
            } else {
                base_address
            };
            let end_element = if $max_elems > 0 {
                std::cmp::min(start_element + $max_elems, VPR_SIZE)
            } else {
                VPR_SIZE
            };

            (start_element..end_element)
                .enumerate()
                .for_each(|(offset, element)| {
                    let current_address = (aligned_address + offset as u32) & ADDR_MASK;
                    modify_vpr_byte(
                        &mut n64.rsp.registers.vpr[rt(opcode) as usize],
                        n64.rsp.memory.mem[current_address as usize],
                        element,
                    );
                });
        }
    };

    ($name:ident, $shift:expr, $elem_mult:expr, $data_shift:expr) => {
        fn $name(n64: &mut N64, opcode: u32) {
            let base_address = n64.rsp.registers.gpr[rs(opcode) as usize]
                .wrapping_add(sign_extend_7bit_offset(voffset(opcode), $shift));

            let index = ((base_address & 7) as u8).wrapping_sub(velement(opcode));
            let aligned_address = base_address & ADDR_ALIGN_MASK;

            (0..VECTOR_SIZE).for_each(|offset| {
                let offset = offset as u8;
                let element_index = (index.wrapping_add(offset * $elem_mult)) & ELEMENT_MASK as u8;
                let current_address = (aligned_address + element_index as u32) & ADDR_MASK;

                modify_vpr_element(
                    &mut n64.rsp.registers.vpr[rt(opcode) as usize],
                    (n64.rsp.memory.mem[current_address as usize] as u16) << $data_shift,
                    offset,
                );
            });
        }
    };
}

impl_vector_load!(lbv, 0, 1, false, 1);
impl_vector_load!(lsv, 1, 2, false, 2);
impl_vector_load!(llv, 2, 4, false, 4);
impl_vector_load!(ldv, 3, 8, false, 8);
impl_vector_load!(lrv, 4, 16, true, 0);

// Special vector loads
impl_vector_load!(lpv, 3, 1, 8);
impl_vector_load!(luv, 3, 1, 7);
impl_vector_load!(lhv, 4, 2, 7);

fn lqv(n64: &mut N64, opcode: u32) {
    let mut address = n64.rsp.registers.gpr[rs(opcode) as usize]
        .wrapping_add(sign_extend_7bit_offset(voffset(opcode), 4));

    let mut element = velement(opcode);
    let end = std::cmp::min(16 + element - ((address & 15) as u8), 16);
    while element < end {
        modify_vpr_byte(
            &mut n64.rsp.registers.vpr[rt(opcode) as usize],
            n64.rsp.memory.mem[(address & ADDR_MASK) as usize],
            element,
        );
        address += 1;
        element += 1;
    }
}

fn read_memory(n64: &N64, address: u32) -> u8 {
    n64.rsp.memory.mem[(address & ADDR_MASK) as usize]
}

fn lfv(n64: &mut N64, opcode: u32) {
    let mut address = calculate_address(n64, opcode);
    let index = ((address & 7) as u8).wrapping_sub(velement(opcode));
    address &= !7;
    let start = velement(opcode);
    let end = std::cmp::min(start + 8, 16);
    let mut tmp: u128 = 0;
    let mut offset: u8 = 0;
    while offset < 4 {
        modify_vpr_element(
            &mut tmp,
            (read_memory(
                n64,
                address.wrapping_add(((index.wrapping_add(offset * 4)) & 15) as u32),
            ) as u16)
                << 7,
            offset,
        );
        modify_vpr_element(
            &mut tmp,
            (read_memory(
                n64,
                address
                    .wrapping_add(((index.wrapping_add(offset * 4).wrapping_add(8)) & 15) as u32),
            ) as u16)
                << 7,
            offset + 4,
        );
        offset += 1;
    }
    offset = start;
    while offset < end {
        let value = get_vpr_byte(tmp, offset);
        modify_vpr_byte(
            &mut n64.rsp.registers.vpr[rt(opcode) as usize],
            value,
            offset,
        );
        offset += 1;
    }
}

fn lwv(_device: &mut N64, _opcode: u32) {}

fn ltv(n64: &mut N64, opcode: u32) {
    const VPR_GROUP_SIZE: usize = 8;
    const ADDRESS_MASK: u32 = 15;
    const ADDRESS_ALIGN: u32 = !7;
    const VT_OFFSET_MASK: u32 = 7;

    let base_address = calculate_address(n64, opcode);
    let begin = base_address & ADDRESS_ALIGN;
    let initial_address = begin + (((velement(opcode)) as u32 + (base_address & 8)) & ADDRESS_MASK);
    let vt_base = rt(opcode) & !7;
    let initial_vt_offset = (velement(opcode)) as u32 >> 1;

    (0..VPR_GROUP_SIZE).fold(
        (initial_address, initial_vt_offset),
        |(mut addr, vt_off), i| {
            // First byte
            let value1 = read_memory(n64, addr);
            modify_vpr_byte(
                &mut n64.rsp.registers.vpr[(vt_base + vt_off) as usize],
                value1,
                (i * 2) as u8,
            );
            addr = if (addr + 1) == begin + 16 {
                begin
            } else {
                addr + 1
            };

            // Second byte
            let value2 = read_memory(n64, addr);
            modify_vpr_byte(
                &mut n64.rsp.registers.vpr[(vt_base + vt_off) as usize],
                value2,
                (i * 2 + 1) as u8,
            );
            addr = if (addr + 1) == begin + 16 {
                begin
            } else {
                addr + 1
            };

            (addr, (vt_off + 1) & VT_OFFSET_MASK)
        },
    );
}

fn sbv(n64: &mut N64, opcode: u32) {
    let address = n64.rsp.registers.gpr[rs(opcode) as usize]
        .wrapping_add(sign_extend_7bit_offset(voffset(opcode), 0));

    n64.rsp.memory.mem[(address & ADDR_MASK) as usize] =
        get_vpr_byte(n64.rsp.registers.vpr[rt(opcode) as usize], velement(opcode))
}

macro_rules! impl_vector_store {
    // Basic vector stores with fixed count
    ($name:ident, $shift:expr, $elem_count:expr) => {
        fn $name(n64: &mut N64, opcode: u32) {
            let base_address = n64.rsp.registers.gpr[rs(opcode) as usize]
                .wrapping_add(sign_extend_7bit_offset(voffset(opcode), $shift));
            let start_element = velement(opcode);

            (0..$elem_count).for_each(|offset| {
                let current_address = (base_address.wrapping_add(offset)) & ADDR_MASK;
                let current_element = start_element.wrapping_add(offset as u8);
                n64.rsp.memory.mem[current_address as usize] =
                    get_vpr_byte(n64.rsp.registers.vpr[rt(opcode) as usize], current_element);
            });
        }
    };

    // Special stores with dynamic end calculation
    ($name:ident, $shift:expr, dynamic_end) => {
        fn $name(n64: &mut N64, opcode: u32) {
            let base_address = n64.rsp.registers.gpr[rs(opcode) as usize]
                .wrapping_add(sign_extend_7bit_offset(voffset(opcode), $shift));
            let start_element = velement(opcode);
            let element_count = (VPR_SIZE - (base_address & 15)) as u8;

            (0..element_count).for_each(|offset| {
                let current_address = (base_address.wrapping_add(offset)) & ADDR_MASK;
                let current_element = start_element.wrapping_add(offset);
                n64.rsp.memory.mem[current_address as usize] =
                    get_vpr_byte(n64.rsp.registers.vpr[rt(opcode) as usize], current_element);
            });
        }
    };

    // Special stores with custom element handling
    ($name:ident, $shift:expr, $handler:expr) => {
        fn $name(n64: &mut N64, opcode: u32) {
            let base_address = n64.rsp.registers.gpr[rs(opcode) as usize]
                .wrapping_add(sign_extend_7bit_offset(voffset(opcode), $shift));
            let start_element = velement(opcode);

            (0..8).for_each(|offset| {
                let current_address = (base_address.wrapping_add(offset)) & ADDR_MASK;
                let current_element = start_element.wrapping_add(offset as u8);
                n64.rsp.memory.mem[current_address as usize] =
                    $handler(n64, opcode, current_element);
            });
        }
    };
}

impl_vector_store!(ssv, 1, 2);
impl_vector_store!(slv, 2, 4);
impl_vector_store!(sdv, 3, 8);

macro_rules! impl_vec_store {
    ($name:ident, $addr_shift:expr, $count_fn:expr, $element_logic:expr) => {
        fn $name(n64: &mut N64, opcode: u32) {
            let base_address = n64.rsp.registers.gpr[rs(opcode) as usize]
                .wrapping_add(sign_extend_7bit_offset(voffset(opcode), $addr_shift));

            let start_element = velement(opcode);
            let element_count = ($count_fn)(base_address);

            (0..element_count).for_each(|offset| {
                let current_address = (base_address.wrapping_add(offset as u32)) & ADDR_MASK;
                let current_element = start_element + offset as u8;
                n64.rsp.memory.mem[current_address as usize] =
                    $element_logic(n64, opcode, current_element);
            });
        }
    };
}

impl_vec_store!(sqv, 4, |addr: u32| (16 - (addr & 15)) as u8, sqv_element);
impl_vec_store!(spv, 3, |_| 8, spv_element);
impl_vec_store!(suv, 3, |_| 8, suv_element);

#[inline(always)]
fn sqv_element(n64: &N64, opcode: u32, element: u8) -> u8 {
    get_vpr_byte(n64.rsp.registers.vpr[rt(opcode) as usize], element)
}

#[inline(always)]
fn spv_element(n64: &N64, opcode: u32, element: u8) -> u8 {
    if (element & 15) < 8 {
        get_vpr_byte(
            n64.rsp.registers.vpr[rt(opcode) as usize],
            (element & 7) << 1,
        )
    } else {
        (get_vpr_element(n64.rsp.registers.vpr[rt(opcode) as usize], element) >> 7) as u8
    }
}

#[inline(always)]
fn suv_element(n64: &N64, opcode: u32, element: u8) -> u8 {
    if (element & 15) < 8 {
        (get_vpr_element(n64.rsp.registers.vpr[rt(opcode) as usize], element) >> 7) as u8
    } else {
        get_vpr_byte(
            n64.rsp.registers.vpr[rt(opcode) as usize],
            (element & 7) << 1,
        )
    }
}

fn calculate_address(n64: &mut N64, opcode: u32) -> u32 {
    n64.rsp.registers.gpr[rs(opcode) as usize]
        .wrapping_add(sign_extend_7bit_offset(voffset(opcode), 4))
}
fn write_memory(n64: &mut N64, address: u32, value: u8) {
    n64.rsp.memory.mem[(address & ADDR_MASK) as usize] = value
}
fn get_vpr_byte_value(n64: &mut N64, opcode: u32, element: u8) -> u8 {
    get_vpr_byte(n64.rsp.registers.vpr[rt(opcode) as usize], element)
}

fn srv(n64: &mut N64, opcode: u32) {
    const SRV_ADDR_ALIGN_MASK: u32 = !15;
    const ADDR_OFFSET_MASK: u32 = 15;
    const VPR_SIZE: u8 = 16;

    let address = calculate_address(n64, opcode);
    let aligned_addr = address & SRV_ADDR_ALIGN_MASK;
    let offset = (address & ADDR_OFFSET_MASK) as u8;
    let start_element = velement(opcode);
    let end_element = start_element + offset;
    let base = VPR_SIZE - offset;

    // Pre-calculate all values
    let values: Vec<u8> = (start_element..end_element)
        .map(|element| get_vpr_byte_value(n64, opcode, element + base))
        .collect();

    // Perform memory writes
    for (i, value) in values.iter().enumerate() {
        write_memory(n64, aligned_addr + i as u32, *value);
    }
}

const VECTOR_SIZE: usize = 8;
const ADDR_ALIGN_MASK: u32 = !7;
const BASE_MASK: u32 = 15;
const BYTE_SHIFT_LEFT: u32 = 1;
const BYTE_SHIFT_RIGHT: u32 = 7;

fn shv(n64: &mut N64, opcode: u32) {
    let base_address = calculate_address(n64, opcode);
    let start_element = velement(opcode);
    let initial_index = (base_address & 7) as u8;
    let aligned_address = base_address & ADDR_ALIGN_MASK;

    // Calculate all byte values first
    let byte_values: Vec<u8> = (0..VECTOR_SIZE)
        .map(|offset| {
            let byte_val = start_element + (offset * 2) as u8;
            (get_vpr_byte_value(n64, opcode, byte_val) << BYTE_SHIFT_LEFT)
                | (get_vpr_byte_value(n64, opcode, byte_val + 1) >> BYTE_SHIFT_RIGHT)
        })
        .collect();

    // Perform memory writes with calculated values
    byte_values.iter().enumerate().for_each(|(offset, &value)| {
        let current_index = ((initial_index as usize + offset * 2) & BASE_MASK as usize) as u32;
        write_memory(n64, aligned_address + current_index, value);
    });
}

const ELEMENTS_PER_GROUP: usize = 4;
const VPR_SHIFT: u32 = 7;

#[derive(Debug, Clone, Copy)]
struct ElementPattern([usize; ELEMENTS_PER_GROUP]);

impl ElementPattern {
    const fn new(pattern: [usize; ELEMENTS_PER_GROUP]) -> Self {
        Self(pattern)
    }
}

const ELEMENT_PATTERNS: &[(u8, ElementPattern)] = &[
    (0, ElementPattern::new([0, 1, 2, 3])),
    (15, ElementPattern::new([0, 1, 2, 3])),
    (1, ElementPattern::new([6, 7, 4, 5])),
    (4, ElementPattern::new([1, 2, 3, 0])),
    (5, ElementPattern::new([7, 4, 5, 6])),
    (8, ElementPattern::new([4, 5, 6, 7])),
    (11, ElementPattern::new([3, 0, 1, 2])),
    (12, ElementPattern::new([5, 6, 7, 4])),
];

fn sfv(n64: &mut N64, opcode: u32) {
    let address = calculate_address(n64, opcode);
    let base = address & 7;
    let aligned_address = address & ADDR_ALIGN_MASK;
    let element = velement(opcode);
    let vpr_register = rt(opcode) as usize;

    match ELEMENT_PATTERNS.iter().find(|&&(e, _)| e == element) {
        Some((_, pattern)) => {
            pattern.0.iter().enumerate().for_each(|(i, &elem)| {
                let offset = ((base + (i * 4) as u32) & BASE_MASK) as usize;
                let vpr_value = get_vpr_element(n64.rsp.registers.vpr[vpr_register], elem as u8);
                let byte_value = (vpr_value >> VPR_SHIFT) as u8;
                write_memory(n64, aligned_address + offset as u32, byte_value);
            });
        }
        None => {
            (0..16).step_by(4).for_each(|i| {
                let offset = ((base + i) & BASE_MASK) as u32;
                write_memory(n64, aligned_address + offset, 0);
            });
        }
    }
}

const ADDR_MASK: u32 = 0xFFF;
const ELEMENT_COUNT: u8 = 16;

fn swv(n64: &mut N64, opcode: u32) {
    let base_address = n64.rsp.registers.gpr[rs(opcode) as usize]
        .wrapping_add(sign_extend_7bit_offset(voffset(opcode), 4));

    let start_element = velement(opcode);
    let initial_base = base_address & 7;
    let aligned_address = base_address & ADDR_ALIGN_MASK;
    let vpr_register = rt(opcode) as usize;

    (0..ELEMENT_COUNT).for_each(|offset| {
        let current_element = start_element + offset;
        let current_base = initial_base.wrapping_add(offset as u32);

        let mem_addr = ((aligned_address + (current_base & BASE_MASK)) & ADDR_MASK) as usize;
        n64.rsp.memory.mem[mem_addr] =
            get_vpr_byte(n64.rsp.registers.vpr[vpr_register], current_element);
    });
}

const VPR_ELEMENTS: usize = 16;

fn stv(n64: &mut N64, opcode: u32) {
    let base_address = n64.rsp.registers.gpr[rs(opcode) as usize]
        .wrapping_add(sign_extend_7bit_offset(voffset(opcode), 4));

    let start_reg = rt(opcode) & !7;
    let start_element = VPR_ELEMENTS - ((velement(opcode) & !1) as usize);
    let base_offset = (base_address & 7).wrapping_sub((velement(opcode) & !1) as u32);
    let aligned_address = base_address & !7;

    let mut write_vpr_byte = |offset: usize, element: usize, base: u32| {
        let mem_addr = ((aligned_address + (base & 15)) & ADDR_MASK) as usize;
        n64.rsp.memory.mem[mem_addr] = get_vpr_byte(n64.rsp.registers.vpr[offset], element as u8);
    };

    (0..VECTOR_SIZE).for_each(|i| {
        let reg_idx = start_reg + i as u32;
        let current_base = base_offset.wrapping_add(2 * i as u32);
        let current_element = start_element + 2 * i;

        write_vpr_byte(reg_idx as usize, current_element, current_base);
        write_vpr_byte(
            reg_idx as usize,
            current_element + 1,
            current_base.wrapping_add(1),
        );
    });
}

fn vt(opcode: u32) -> u32 {
    (opcode >> 16) & 0x1F
}
fn ve(opcode: u32) -> u32 {
    (opcode >> 21) & 0xF
}
fn vs(opcode: u32) -> u32 {
    (opcode >> 11) & 0x1F
}
fn vd(opcode: u32) -> u32 {
    (opcode >> 6) & 0x1F
}
fn de(opcode: u32) -> u32 {
    (opcode >> 11) & 0x7
}

fn clamp_signed_32(value: i32) -> i16 {
    value.clamp(i16::MIN as i32, i16::MAX as i32) as i16
}
fn clamp_signed_64(value: i64) -> i16 {
    value.clamp(i16::MIN as i64, i16::MAX as i64) as i16
}
fn count_leading_zeros(value: u32) -> u32 {
    value.leading_zeros()
}

fn s_clip(x: i64, bits: u32) -> i64 {
    let b = 1_u64 << (bits - 1);
    let m = b * 2 - 1;
    ((((x as u64) & m) ^ b).wrapping_sub(b)) as i64
}

fn vte(n64: &N64, vt: u32, index: usize) -> __m128i {
    unsafe {
        _mm_shuffle_epi8(
            std::mem::transmute::<u128, std::arch::x86_64::__m128i>(
                n64.rsp.registers.vpr[vt as usize],
            ),
            n64.rsp.vector.shuffle[index],
        )
    }
}

// Helper functions
#[inline(always)]
unsafe fn vs_reg(n64: &N64, opcode: u32) -> __m128i {
    std::mem::transmute(n64.rsp.registers.vpr[vs(opcode) as usize])
}

#[inline(always)]
unsafe fn set_vd_reg(n64: &mut N64, opcode: u32, value: __m128i) {
    n64.rsp.registers.vpr[vd(opcode) as usize] = std::mem::transmute(value);
}

macro_rules! vector_multiply {
    ($n64:expr, $opcode:expr, $final_op:expr) => {{
        let vte = vte($n64, vt($opcode), ve($opcode) as usize);
        unsafe {
            let vs = vs_reg($n64, $opcode);
            let lo = _mm_mullo_epi16(vs, vte);
            let round = _mm_slli_epi16(
                _mm_cmpeq_epi16(_mm_setzero_si128(), _mm_setzero_si128()),
                15,
            );

            let sign1 = _mm_srli_epi16(lo, 15);
            let lo = _mm_add_epi16(lo, lo);
            let sign2 = _mm_srli_epi16(lo, 15);

            $n64.rsp.accumulator.accl = _mm_add_epi16(round, lo);

            let hi = _mm_slli_epi16(_mm_mulhi_epi16(vs, vte), 1);
            let neq = _mm_cmpeq_epi16(vs, vte);

            $n64.rsp.accumulator.accm = _mm_add_epi16(hi, _mm_add_epi16(sign1, sign2));
            let neg = _mm_srai_epi16($n64.rsp.accumulator.accm, 15);

            $n64.rsp.accumulator.acch = _mm_andnot_si128(neq, neg);

            // Final operation differs between vmulf and vmulu
            let result = $final_op($n64, neq, neg);
            set_vd_reg($n64, $opcode, result)
        }
    }};
}

fn vmulf(n64: &mut N64, opcode: u32) {
    vector_multiply!(n64, opcode, |n64: &N64, neq, neg| {
        let eq = _mm_and_si128(neq, neg);
        _mm_add_epi16(n64.rsp.accumulator.accm, eq)
    });
}

fn vmulu(n64: &mut N64, opcode: u32) {
    vector_multiply!(n64, opcode, |n64: &N64, _neq, neg| {
        let hi = _mm_or_si128(n64.rsp.accumulator.accm, neg);
        _mm_andnot_si128(n64.rsp.accumulator.acch, hi)
    });
}

#[inline(always)]
fn get_accumulator(acch: u128, accm: u128, accl: u128, n: usize) -> i64 {
    let mut acc = 0i64;
    acc |= get_vpr_element(acch, n as u8) as i64;
    acc <<= 16;
    acc |= get_vpr_element(accm, n as u8) as i64;
    acc <<= 16;
    acc |= get_vpr_element(accl, n as u8) as i64;
    acc << 16 >> 16
}

#[inline(always)]
fn set_accumulator(acch: &mut u128, accm: &mut u128, accl: &mut u128, acc: i64, n: usize) {
    modify_vpr_element(acch, (acc >> 32) as u16, n as u8);
    modify_vpr_element(accm, (acc >> 16) as u16, n as u8);
    modify_vpr_element(accl, acc as u16, n as u8);
}

fn vrndp(n64: &mut N64, opcode: u32) {
    let vte = unsafe {
        std::mem::transmute::<std::arch::x86_64::__m128i, u128>(vte(
            n64,
            vt(opcode),
            ve(opcode) as usize,
        ))
    };

    let (acch, accm, accl) = unsafe {
        (
            std::mem::transmute::<_, &mut u128>(&mut n64.rsp.accumulator.acch),
            std::mem::transmute::<_, &mut u128>(&mut n64.rsp.accumulator.accm),
            std::mem::transmute::<_, &mut u128>(&mut n64.rsp.accumulator.accl),
        )
    };

    for n in 0..8 {
        let product = if vs(opcode) & 1 != 0 {
            (get_vpr_element(vte, n) as i16 as i32) << 16
        } else {
            get_vpr_element(vte, n) as i16 as i32
        };

        let mut acc = get_accumulator(*acch, *accm, *accl, n as usize);
        if acc >= 0 {
            acc = s_clip(acc + product as i64, 48);
        }

        set_accumulator(acch, accm, accl, acc, n as usize);

        modify_vpr_element(
            &mut n64.rsp.registers.vpr[vd(opcode) as usize],
            clamp_signed_64(acc >> 16) as u16,
            n,
        );
    }
}

#[inline(always)]
fn vector_mul_element(vs: u128, vte: u128, n: usize) -> i32 {
    (get_vpr_element(vs, n as u8) as i16 as i32)
        .wrapping_mul(get_vpr_element(vte, n as u8) as i16 as i32)
}

#[inline(always)]
fn set_acc_elements(acch: &mut u128, accm: &mut u128, accl: &mut u128, product: i32, n: usize) {
    modify_vpr_element(acch, (product >> 16) as u16, n as u8);
    modify_vpr_element(accm, product as u16, n as u8);
    modify_vpr_element(accl, 0, n as u8);
}

fn vmulq(n64: &mut N64, opcode: u32) {
    let vte =
        unsafe { std::mem::transmute::<__m128i, u128>(vte(n64, vt(opcode), ve(opcode) as usize)) };

    let vs = n64.rsp.registers.vpr[vs(opcode) as usize];
    let (acch, accm, accl) = unsafe {
        (
            std::mem::transmute::<_, &mut u128>(&mut n64.rsp.accumulator.acch),
            std::mem::transmute::<_, &mut u128>(&mut n64.rsp.accumulator.accm),
            std::mem::transmute::<_, &mut u128>(&mut n64.rsp.accumulator.accl),
        )
    };

    for n in 0..8 {
        let mut product = vector_mul_element(vs, vte, n);
        if product < 0 {
            product += 31
        }

        set_acc_elements(acch, accm, accl, product, n);

        modify_vpr_element(
            &mut n64.rsp.registers.vpr[vd(opcode) as usize],
            (clamp_signed_32(product >> 1) & !15) as u16,
            n as u8,
        );
    }
}

#[inline(always)]
unsafe fn get_vs_reg(n64: &N64, opcode: u32) -> __m128i {
    std::mem::transmute(n64.rsp.registers.vpr[vs(opcode) as usize])
}

fn vmudl(n64: &mut N64, opcode: u32) {
    unsafe {
        let vs = get_vs_reg(n64, opcode);
        let vte = vte(n64, vt(opcode), ve(opcode) as usize);

        n64.rsp.accumulator.accl = _mm_mulhi_epu16(vs, vte);

        n64.rsp.accumulator.accm = _mm_setzero_si128();
        n64.rsp.accumulator.acch = _mm_setzero_si128();

        set_vd_reg(n64, opcode, n64.rsp.accumulator.accl);
    }
}

macro_rules! impl_vmud {
    ($name:ident, $operation:expr) => {
        fn $name(n64: &mut N64, opcode: u32) {
            let vte = vte(n64, vt(opcode), ve(opcode) as usize);
            unsafe {
                let vs = vpr_as_m128i(&n64.rsp.registers.vpr[vs(opcode) as usize]);

                n64.rsp.accumulator.accl = _mm_mullo_epi16(vs, vte);
                n64.rsp.accumulator.accm = _mm_mulhi_epu16(vs, vte);

                let result = $operation(n64, vs, vte);

                n64.rsp.registers.vpr[vd(opcode) as usize] = m128i_as_u128(result);
            }
        }
    };
}

#[inline(always)]
unsafe fn vpr_as_m128i(vpr: &u128) -> __m128i {
    std::mem::transmute::<u128, __m128i>(*vpr)
}
#[inline(always)]
unsafe fn m128i_as_u128(val: __m128i) -> u128 {
    std::mem::transmute::<__m128i, u128>(val)
}

impl_vmud!(vmudm, |n64: &mut N64, vs, vte| -> __m128i {
    let sign = _mm_srai_epi16(vs, 15);
    let vta = _mm_and_si128(vte, sign);
    n64.rsp.accumulator.accm = _mm_sub_epi16(n64.rsp.accumulator.accm, vta);
    n64.rsp.accumulator.acch = _mm_srai_epi16(n64.rsp.accumulator.accm, 15);
    n64.rsp.accumulator.accm
});

impl_vmud!(vmudn, |n64: &mut N64, vs, vte| {
    let sign = _mm_srai_epi16(vte, 15);
    let vsa = _mm_and_si128(vs, sign);
    n64.rsp.accumulator.accm = _mm_sub_epi16(n64.rsp.accumulator.accm, vsa);
    n64.rsp.accumulator.acch = _mm_srai_epi16(n64.rsp.accumulator.accm, 15);
    n64.rsp.accumulator.accl
});

impl_vmud!(vmudh, |n64: &mut N64, vs, vte| {
    n64.rsp.accumulator.accl = _mm_setzero_si128();
    n64.rsp.accumulator.accm = _mm_mullo_epi16(vs, vte);
    n64.rsp.accumulator.acch = _mm_mulhi_epi16(vs, vte);
    let lo = _mm_unpacklo_epi16(n64.rsp.accumulator.accm, n64.rsp.accumulator.acch);
    let hi = _mm_unpackhi_epi16(n64.rsp.accumulator.accm, n64.rsp.accumulator.acch);
    _mm_packs_epi32(lo, hi)
});

#[inline(always)]
unsafe fn get_simd_vs(n64: &N64, opcode: u32) -> __m128i {
    std::mem::transmute(n64.rsp.registers.vpr[vs(opcode) as usize])
}

#[inline(always)]
unsafe fn set_vd_simd(n64: &mut N64, opcode: u32, value: __m128i) {
    n64.rsp.registers.vpr[vd(opcode) as usize] = std::mem::transmute(value);
}

fn vmacf(n64: &mut N64, opcode: u32) {
    unsafe {
        let vs = get_simd_vs(n64, opcode);
        let vte = vte(n64, vt(opcode), ve(opcode) as usize);

        // Multiply step
        let lo = _mm_mullo_epi16(vs, vte);
        let hi = _mm_mulhi_epi16(vs, vte);
        let md = _mm_slli_epi16(hi, 1);
        let carry = _mm_srli_epi16(lo, 15);
        let hi = _mm_srai_epi16(hi, 15);
        let md = _mm_or_si128(md, carry);
        let lo = _mm_slli_epi16(lo, 1);

        // Accumulate low bits
        let omask = _mm_adds_epu16(n64.rsp.accumulator.accl, lo);
        n64.rsp.accumulator.accl = _mm_add_epi16(n64.rsp.accumulator.accl, lo);
        let omask = _mm_cmpeq_epi16(n64.rsp.accumulator.accl, omask);
        let omask = _mm_cmpeq_epi16(omask, _mm_setzero_si128());

        // Handle middle bits
        let md = _mm_sub_epi16(md, omask);
        let carry = _mm_cmpeq_epi16(md, _mm_setzero_si128());
        let carry = _mm_and_si128(carry, omask);
        let hi = _mm_sub_epi16(hi, carry);

        // Accumulate middle bits
        let omask = _mm_adds_epu16(n64.rsp.accumulator.accm, md);
        n64.rsp.accumulator.accm = _mm_add_epi16(n64.rsp.accumulator.accm, md);
        let omask = _mm_cmpeq_epi16(n64.rsp.accumulator.accm, omask);
        let omask = _mm_cmpeq_epi16(omask, _mm_setzero_si128());

        // Accumulate high bits
        n64.rsp.accumulator.acch = _mm_add_epi16(n64.rsp.accumulator.acch, hi);
        n64.rsp.accumulator.acch = _mm_sub_epi16(n64.rsp.accumulator.acch, omask);

        // Pack result
        let lo = _mm_unpacklo_epi16(n64.rsp.accumulator.accm, n64.rsp.accumulator.acch);
        let hi = _mm_unpackhi_epi16(n64.rsp.accumulator.accm, n64.rsp.accumulator.acch);
        let result = _mm_packs_epi32(lo, hi);

        set_vd_simd(n64, opcode, result);
    }
}

fn vmacu(n64: &mut N64, opcode: u32) {
    let vte = vte(n64, vt(opcode), ve(opcode) as usize);
    unsafe {
        let lo = _mm_mullo_epi16(
            std::mem::transmute::<u128, std::arch::x86_64::__m128i>(
                n64.rsp.registers.vpr[vs(opcode) as usize],
            ),
            vte,
        );
        let hi = _mm_mulhi_epi16(
            std::mem::transmute::<u128, std::arch::x86_64::__m128i>(
                n64.rsp.registers.vpr[vs(opcode) as usize],
            ),
            vte,
        );
        let mut md = _mm_slli_epi16(hi, 1);
        let carry = _mm_srli_epi16(lo, 15);
        let hi = _mm_srai_epi16(hi, 15);
        md = _mm_or_si128(md, carry);
        let lo = _mm_slli_epi16(lo, 1);
        let mut omask = _mm_adds_epu16(n64.rsp.accumulator.accl, lo);
        n64.rsp.accumulator.accl = _mm_add_epi16(n64.rsp.accumulator.accl, lo);
        omask = _mm_cmpeq_epi16(n64.rsp.accumulator.accl, omask);
        omask = _mm_cmpeq_epi16(omask, _mm_setzero_si128());
        md = _mm_sub_epi16(md, omask);
        let carry = _mm_cmpeq_epi16(md, _mm_setzero_si128());
        let carry = _mm_and_si128(carry, omask);
        let hi = _mm_sub_epi16(hi, carry);
        omask = _mm_adds_epu16(n64.rsp.accumulator.accm, md);
        n64.rsp.accumulator.accm = _mm_add_epi16(n64.rsp.accumulator.accm, md);
        omask = _mm_cmpeq_epi16(n64.rsp.accumulator.accm, omask);
        omask = _mm_cmpeq_epi16(omask, _mm_setzero_si128());
        n64.rsp.accumulator.acch = _mm_add_epi16(n64.rsp.accumulator.acch, hi);
        n64.rsp.accumulator.acch = _mm_sub_epi16(n64.rsp.accumulator.acch, omask);

        let mmask = _mm_srai_epi16(n64.rsp.accumulator.accm, 15);
        let hmask = _mm_srai_epi16(n64.rsp.accumulator.acch, 15);
        md = _mm_or_si128(mmask, n64.rsp.accumulator.accm);
        omask = _mm_cmpgt_epi16(n64.rsp.accumulator.acch, _mm_setzero_si128());
        md = _mm_andnot_si128(hmask, md);
        n64.rsp.registers.vpr[vd(opcode) as usize] =
            std::mem::transmute::<std::arch::x86_64::__m128i, u128>(_mm_or_si128(omask, md));
    }
}

#[inline(always)]
fn get_acc_value(acch: u128, accm: u128, accl: u128, n: usize) -> i64 {
    let mut acc = 0i64;
    acc |= get_vpr_element(acch, n as u8) as i64;
    acc <<= 16;
    acc |= get_vpr_element(accm, n as u8) as i64;
    acc <<= 16;
    acc |= get_vpr_element(accl, n as u8) as i64;
    acc << 16 >> 16
}

#[inline(always)]
fn set_acc_value(acch: &mut u128, accm: &mut u128, accl: &mut u128, acc: i64, n: usize) {
    modify_vpr_element(acch, (acc >> 32) as u16, n as u8);
    modify_vpr_element(accm, (acc >> 16) as u16, n as u8);
    modify_vpr_element(accl, acc as u16, n as u8);
}

fn vrndn(n64: &mut N64, opcode: u32) {
    let vte =
        unsafe { std::mem::transmute::<__m128i, u128>(vte(n64, vt(opcode), ve(opcode) as usize)) };

    let (acch, accm, accl) = unsafe {
        (
            std::mem::transmute::<_, &mut u128>(&mut n64.rsp.accumulator.acch),
            std::mem::transmute::<_, &mut u128>(&mut n64.rsp.accumulator.accm),
            std::mem::transmute::<_, &mut u128>(&mut n64.rsp.accumulator.accl),
        )
    };

    for n in 0..8 {
        let product = if vs(opcode) & 1 != 0 {
            (get_vpr_element(vte, n) as i16 as i32) << 16
        } else {
            get_vpr_element(vte, n) as i16 as i32
        };

        let mut acc = get_acc_value(*acch, *accm, *accl, n as usize);
        if acc < 0 {
            acc = s_clip(acc + product as i64, 48);
        }

        set_acc_value(acch, accm, accl, acc, n as usize);

        modify_vpr_element(
            &mut n64.rsp.registers.vpr[vd(opcode) as usize],
            clamp_signed_64(acc >> 16) as u16,
            n,
        );
    }
}

fn vmacq(n64: &mut N64, opcode: u32) {
    let acch: &mut u128 = unsafe { std::mem::transmute(&mut n64.rsp.accumulator.acch) };
    let accm: &mut u128 = unsafe { std::mem::transmute(&mut n64.rsp.accumulator.accm) };

    let mut n = 0;
    while n < 8 {
        let mut product =
            (get_vpr_element(*acch, n) as i32) << 16 | (get_vpr_element(*accm, n) as i32);
        if product < 0 && (product & (1 << 5)) == 0 {
            product += 32
        } else if product >= 32 && (product & (1 << 5)) == 0 {
            product -= 32
        }
        modify_vpr_element(acch, (product >> 16) as u16, n);
        modify_vpr_element(accm, (product) as u16, n);
        modify_vpr_element(
            &mut n64.rsp.registers.vpr[vd(opcode) as usize],
            (clamp_signed_32(product >> 1) & !15) as u16,
            n,
        );
        n += 1;
    }
}

#[inline(always)]
unsafe fn get_vs_simd(n64: &N64, opcode: u32) -> __m128i {
    std::mem::transmute(n64.rsp.registers.vpr[vs(opcode) as usize])
}

fn vmadl(n64: &mut N64, opcode: u32) {
    unsafe {
        // Get source vectors and multiply high
        let vte = vte(n64, vt(opcode), ve(opcode) as usize);
        let vs = get_vs_simd(n64, opcode);
        let hi = _mm_mulhi_epu16(vs, vte);

        // Accumulate low bits
        let omask = _mm_adds_epu16(n64.rsp.accumulator.accl, hi);
        n64.rsp.accumulator.accl = _mm_add_epi16(n64.rsp.accumulator.accl, hi);
        let mut omask = _mm_cmpeq_epi16(n64.rsp.accumulator.accl, omask);
        omask = _mm_cmpeq_epi16(omask, _mm_setzero_si128());

        // Handle middle bits
        let hi = _mm_sub_epi16(_mm_setzero_si128(), omask);
        let omask = _mm_adds_epu16(n64.rsp.accumulator.accm, hi);
        n64.rsp.accumulator.accm = _mm_add_epi16(n64.rsp.accumulator.accm, hi);
        let omask = _mm_cmpeq_epi16(n64.rsp.accumulator.accm, omask);
        let omask = _mm_cmpeq_epi16(omask, _mm_setzero_si128());

        // Handle high bits and shifts
        n64.rsp.accumulator.acch = _mm_sub_epi16(n64.rsp.accumulator.acch, omask);
        let nhi = _mm_srai_epi16(n64.rsp.accumulator.acch, 15);
        let nmd = _mm_srai_epi16(n64.rsp.accumulator.accm, 15);

        // Calculate masks for final blend
        let shi = _mm_cmpeq_epi16(nhi, n64.rsp.accumulator.acch);
        let smd = _mm_cmpeq_epi16(nhi, nmd);
        let cmask = _mm_and_si128(smd, shi);
        let cval = _mm_cmpeq_epi16(nhi, _mm_setzero_si128());

        // Store final result
        let result = _mm_blendv_epi8(cval, n64.rsp.accumulator.accl, cmask);
        set_vd_simd(n64, opcode, result);
    }
}

fn vmadm(n64: &mut N64, opcode: u32) {
    unsafe {
        // Get source vectors
        let vs = get_vs_simd(n64, opcode);
        let vte = vte(n64, vt(opcode), ve(opcode) as usize);

        // Multiply step
        let lo = _mm_mullo_epi16(vs, vte);
        let hi = _mm_mulhi_epu16(vs, vte);
        let sign = _mm_srai_epi16(vs, 15);
        let vta = _mm_and_si128(vte, sign);
        let hi = _mm_sub_epi16(hi, vta);

        // Accumulate low bits
        let mut omask = _mm_adds_epu16(n64.rsp.accumulator.accl, lo);
        n64.rsp.accumulator.accl = _mm_add_epi16(n64.rsp.accumulator.accl, lo);
        omask = _mm_cmpeq_epi16(n64.rsp.accumulator.accl, omask);
        omask = _mm_cmpeq_epi16(omask, _mm_setzero_si128());

        // Handle middle bits
        let hi = _mm_sub_epi16(hi, omask);
        let omask = _mm_adds_epu16(n64.rsp.accumulator.accm, hi);
        n64.rsp.accumulator.accm = _mm_add_epi16(n64.rsp.accumulator.accm, hi);
        let omask = _mm_cmpeq_epi16(n64.rsp.accumulator.accm, omask);
        let omask = _mm_cmpeq_epi16(omask, _mm_setzero_si128());

        // Handle high bits
        let hi = _mm_srai_epi16(hi, 15);
        n64.rsp.accumulator.acch = _mm_add_epi16(n64.rsp.accumulator.acch, hi);
        n64.rsp.accumulator.acch = _mm_sub_epi16(n64.rsp.accumulator.acch, omask);

        // Pack result
        let lo = _mm_unpacklo_epi16(n64.rsp.accumulator.accm, n64.rsp.accumulator.acch);
        let hi = _mm_unpackhi_epi16(n64.rsp.accumulator.accm, n64.rsp.accumulator.acch);
        let result = _mm_packs_epi32(lo, hi);

        set_vd_simd(n64, opcode, result);
    }
}

#[macro_export]
macro_rules! vpr_as_m128i {
    ($n64:expr, $idx:expr) => {
        unsafe {
            std::mem::transmute::<u128, std::arch::x86_64::__m128i>(
                $n64.rsp.registers.vpr[$idx as usize],
            )
        }
    };
}

macro_rules! m128i_as_vpr {
    ($val:expr) => {
        std::mem::transmute::<std::arch::x86_64::__m128i, u128>($val)
    };
}

fn handle_accumulator(acc: __m128i, value: __m128i) -> (__m128i, __m128i) {
    unsafe {
        let sum = _mm_adds_epu16(acc, value);
        let result = _mm_add_epi16(acc, value);
        let omask = _mm_cmpeq_epi16(_mm_cmpeq_epi16(result, sum), _mm_setzero_si128());
        (result, omask)
    }
}

fn vmadn(n64: &mut N64, opcode: u32) {
    let vte = vte(n64, vt(opcode), ve(opcode) as usize);
    unsafe {
        let vs = get_vs_register(n64, opcode);

        // Multiply and sign handling
        let lo = _mm_mullo_epi16(vs, vte);
        let sign = _mm_srai_epi16(vte, 15);
        let mut hi = _mm_sub_epi16(_mm_mulhi_epu16(vs, vte), _mm_and_si128(vs, sign));

        // Accumulator handling
        let (accl, omask1) = handle_accumulator(n64.rsp.accumulator.accl, lo);
        hi = _mm_sub_epi16(hi, omask1);
        let (accm, omask2) = handle_accumulator(n64.rsp.accumulator.accm, hi);

        // Update accumulators
        n64.rsp.accumulator.accl = accl;
        n64.rsp.accumulator.accm = accm;
        n64.rsp.accumulator.acch = _mm_sub_epi16(
            _mm_add_epi16(n64.rsp.accumulator.acch, _mm_srai_epi16(hi, 15)),
            omask2,
        );

        // Final result calculation
        let nhi = _mm_srai_epi16(n64.rsp.accumulator.acch, 15);
        let cmask = _mm_and_si128(
            _mm_cmpeq_epi16(nhi, _mm_srai_epi16(accm, 15)),
            _mm_cmpeq_epi16(nhi, n64.rsp.accumulator.acch),
        );

        set_vd_register(
            n64,
            opcode,
            _mm_blendv_epi8(_mm_cmpeq_epi16(nhi, _mm_setzero_si128()), accl, cmask),
        );
    }
}

fn vmadh(n64: &mut N64, opcode: u32) {
    let vte = vte(n64, vt(opcode), ve(opcode) as usize);
    let (mut lo, mut hi, mut omask);
    unsafe {
        lo = _mm_mullo_epi16(vpr_as_m128i!(n64, vs(opcode)), vte);
        hi = _mm_mulhi_epi16(vpr_as_m128i!(n64, vs(opcode)), vte);
        omask = _mm_adds_epu16(n64.rsp.accumulator.accm, lo);
        n64.rsp.accumulator.accm = _mm_add_epi16(n64.rsp.accumulator.accm, lo);
        omask = _mm_cmpeq_epi16(n64.rsp.accumulator.accm, omask);
        omask = _mm_cmpeq_epi16(omask, _mm_setzero_si128());
        hi = _mm_sub_epi16(hi, omask);
        n64.rsp.accumulator.acch = _mm_add_epi16(n64.rsp.accumulator.acch, hi);
        lo = _mm_unpacklo_epi16(n64.rsp.accumulator.accm, n64.rsp.accumulator.acch);
        hi = _mm_unpackhi_epi16(n64.rsp.accumulator.accm, n64.rsp.accumulator.acch);
        n64.rsp.registers.vpr[vd(opcode) as usize] = m128i_as_vpr!(_mm_packs_epi32(lo, hi));
    }
}

fn vadd(n64: &mut N64, opcode: u32) {
    let vte = vte(n64, vt(opcode), ve(opcode) as usize);
    let (sum, mut min, max);
    unsafe {
        sum = _mm_add_epi16(vpr_as_m128i!(n64, vs(opcode)), vte);
        n64.rsp.accumulator.accl = _mm_sub_epi16(sum, n64.rsp.vector_control.vcol);
        min = _mm_min_epi16(vpr_as_m128i!(n64, vs(opcode)), vte);
        max = _mm_max_epi16(vpr_as_m128i!(n64, vs(opcode)), vte);
        min = _mm_subs_epi16(min, n64.rsp.vector_control.vcol);
        n64.rsp.registers.vpr[vd(opcode) as usize] = m128i_as_vpr!(_mm_adds_epi16(min, max));
        n64.rsp.vector_control.vcol = _mm_setzero_si128();
        n64.rsp.vector_control.vcoh = _mm_setzero_si128();
    }
}

/// Vector subtract with saturation and overflow handling
#[inline]
fn vsub(n64: &mut N64, opcode: u32) {
    let dest = vd(opcode) as usize;
    let vector_t = vte(n64, vt(opcode), ve(opcode) as usize);
    let vector_s = vpr_as_m128i!(n64, vs(opcode));

    unsafe {
        // Calculate differences
        let udiff = _mm_sub_epi16(vector_t, n64.rsp.vector_control.vcol);
        let sdiff = _mm_subs_epi16(vector_t, n64.rsp.vector_control.vcol);

        // Update accumulator and detect overflow
        n64.rsp.accumulator.accl = _mm_sub_epi16(vector_s, udiff);
        let overflow = _mm_cmpgt_epi16(sdiff, udiff);

        // Store result with overflow correction
        n64.rsp.registers.vpr[dest] = m128i_as_vpr!(_mm_subs_epi16(vector_s, sdiff));
        n64.rsp.registers.vpr[dest] =
            m128i_as_vpr!(_mm_adds_epi16(vpr_as_m128i!(n64, dest), overflow));

        // Clear carry flags
        n64.rsp.vector_control.vcol = _mm_setzero_si128();
        n64.rsp.vector_control.vcoh = _mm_setzero_si128();
    }
}

fn vzero(n64: &mut N64, opcode: u32) {
    let vte = vte(n64, vt(opcode), ve(opcode) as usize);
    unsafe {
        n64.rsp.accumulator.accl = _mm_add_epi16(vpr_as_m128i!(n64, vs(opcode)), vte);
        n64.rsp.registers.vpr[vd(opcode) as usize] = m128i_as_vpr!(_mm_xor_si128(
            vpr_as_m128i!(n64, vd(opcode)),
            vpr_as_m128i!(n64, vd(opcode))
        ));
    }
}

// Helper functions
fn get_vs_register(n64: &N64, opcode: u32) -> __m128i {
    unsafe { std::mem::transmute::<u128, __m128i>(n64.rsp.registers.vpr[vs(opcode) as usize]) }
}

fn set_vd_register(n64: &mut N64, opcode: u32, value: __m128i) {
    unsafe {
        n64.rsp.registers.vpr[vd(opcode) as usize] = std::mem::transmute::<__m128i, u128>(value);
    }
}

fn vabs(n64: &mut N64, opcode: u32) {
    let vte = vte(n64, vt(opcode), ve(opcode) as usize);
    unsafe {
        let vs = get_vs_register(n64, opcode);
        let vs0 = _mm_cmpeq_epi16(vs, _mm_setzero_si128());
        let slt = _mm_srai_epi16(vs, 15);

        let mut result = _mm_andnot_si128(vs0, vte);
        result = _mm_xor_si128(result, slt);

        n64.rsp.accumulator.accl = _mm_sub_epi16(result, slt);
        set_vd_register(n64, opcode, _mm_subs_epi16(result, slt));
    }
}

fn vaddc(n64: &mut N64, opcode: u32) {
    let vte = vte(n64, vt(opcode), ve(opcode) as usize);
    unsafe {
        let vs = get_vs_register(n64, opcode);
        let sum = _mm_adds_epu16(vs, vte);
        n64.rsp.accumulator.accl = _mm_add_epi16(vs, vte);

        n64.rsp.vector_control.vcol = _mm_cmpeq_epi16(sum, n64.rsp.accumulator.accl);
        n64.rsp.vector_control.vcol =
            _mm_cmpeq_epi16(n64.rsp.vector_control.vcol, _mm_setzero_si128());
        n64.rsp.vector_control.vcoh = _mm_setzero_si128();

        set_vd_register(n64, opcode, n64.rsp.accumulator.accl);
    }
}

fn vsubc(n64: &mut N64, opcode: u32) {
    let vte = vte(n64, vt(opcode), ve(opcode) as usize);
    unsafe {
        let vs = get_vs_register(n64, opcode);
        let udiff = _mm_subs_epu16(vs, vte);
        let equal = _mm_cmpeq_epi16(vs, vte);
        let diff0 = _mm_cmpeq_epi16(udiff, _mm_setzero_si128());

        n64.rsp.vector_control.vcoh = _mm_cmpeq_epi16(equal, _mm_setzero_si128());
        n64.rsp.vector_control.vcol = _mm_andnot_si128(equal, diff0);
        n64.rsp.accumulator.accl = _mm_sub_epi16(vs, vte);

        set_vd_register(n64, opcode, n64.rsp.accumulator.accl);
    }
}

fn transmute_acc(acc: __m128i) -> u128 {
    unsafe { std::mem::transmute::<__m128i, u128>(acc) }
}

fn vsar(n64: &mut N64, opcode: u32) {
    let value = match ve(opcode) {
        0x8 => transmute_acc(n64.rsp.accumulator.acch),
        0x9 => transmute_acc(n64.rsp.accumulator.accm),
        0xa => transmute_acc(n64.rsp.accumulator.accl),
        _ => 0,
    };
    n64.rsp.registers.vpr[vd(opcode) as usize] = value;
}

fn vlt(n64: &mut N64, opcode: u32) {
    let vte = vte(n64, vt(opcode), ve(opcode) as usize);
    unsafe {
        let vs = get_vs_register(n64, opcode);
        let eq = _mm_and_si128(
            _mm_and_si128(n64.rsp.vector_control.vcoh, _mm_cmpeq_epi16(vs, vte)),
            n64.rsp.vector_control.vcol,
        );
        n64.rsp.vector_control.vccl = _mm_or_si128(_mm_cmplt_epi16(vs, vte), eq);
        n64.rsp.accumulator.accl = _mm_blendv_epi8(vte, vs, n64.rsp.vector_control.vccl);

        // Clear condition codes
        n64.rsp.vector_control.vcch = _mm_setzero_si128();
        n64.rsp.vector_control.vcoh = _mm_setzero_si128();
        n64.rsp.vector_control.vcol = _mm_setzero_si128();

        set_vd_register(n64, opcode, n64.rsp.accumulator.accl);
    }
}

fn veq(n64: &mut N64, opcode: u32) {
    let vte = vte(n64, vt(opcode), ve(opcode) as usize);
    unsafe {
        let vs = get_vs_register(n64, opcode);
        let eq = _mm_cmpeq_epi16(vs, vte);
        n64.rsp.vector_control.vccl = _mm_andnot_si128(n64.rsp.vector_control.vcoh, eq);
        n64.rsp.accumulator.accl = _mm_blendv_epi8(vte, vs, n64.rsp.vector_control.vccl);

        // Clear condition codes
        n64.rsp.vector_control.vcch = _mm_setzero_si128();
        n64.rsp.vector_control.vcoh = _mm_setzero_si128();
        n64.rsp.vector_control.vcol = _mm_setzero_si128();

        set_vd_register(n64, opcode, n64.rsp.accumulator.accl);
    }
}

fn vne(n64: &mut N64, opcode: u32) {
    let vte = vte(n64, vt(opcode), ve(opcode) as usize);
    unsafe {
        let vs = get_vs_register(n64, opcode);
        let eq = _mm_cmpeq_epi16(vs, vte);
        let ne = _mm_cmpeq_epi16(eq, _mm_setzero_si128());

        n64.rsp.vector_control.vccl =
            _mm_or_si128(_mm_and_si128(n64.rsp.vector_control.vcoh, eq), ne);
        n64.rsp.accumulator.accl = _mm_blendv_epi8(vte, vs, n64.rsp.vector_control.vccl);

        // Clear condition codes
        n64.rsp.vector_control.vcch = _mm_setzero_si128();
        n64.rsp.vector_control.vcoh = _mm_setzero_si128();
        n64.rsp.vector_control.vcol = _mm_setzero_si128();

        set_vd_register(n64, opcode, n64.rsp.accumulator.accl);
    }
}

fn vge(n64: &mut N64, opcode: u32) {
    let vte = vte(n64, vt(opcode), ve(opcode) as usize);
    unsafe {
        let vs = get_vs_register(n64, opcode);
        let eq = _mm_cmpeq_epi16(vs, vte);
        let gt = _mm_cmpgt_epi16(vs, vte);
        let es = _mm_and_si128(n64.rsp.vector_control.vcoh, n64.rsp.vector_control.vcol);

        n64.rsp.vector_control.vccl = _mm_or_si128(gt, _mm_andnot_si128(es, eq));
        n64.rsp.accumulator.accl = _mm_blendv_epi8(vte, vs, n64.rsp.vector_control.vccl);

        // Clear condition codes
        n64.rsp.vector_control.vcch = _mm_setzero_si128();
        n64.rsp.vector_control.vcoh = _mm_setzero_si128();
        n64.rsp.vector_control.vcol = _mm_setzero_si128();

        set_vd_register(n64, opcode, n64.rsp.accumulator.accl);
    }
}

fn vcl(n64: &mut N64, opcode: u32) {
    let vte = vte(n64, vt(opcode), ve(opcode) as usize);
    unsafe {
        let vs = get_vs_register(n64, opcode);
        let nvt = _mm_sub_epi16(
            _mm_xor_si128(vte, n64.rsp.vector_control.vcol),
            n64.rsp.vector_control.vcol,
        );

        let diff = _mm_sub_epi16(vs, nvt);
        let ncarry = _mm_cmpeq_epi16(_mm_adds_epu16(vs, vte), diff);
        let nvce = _mm_cmpeq_epi16(n64.rsp.vector_control.vce, _mm_setzero_si128());
        let diff0 = _mm_cmpeq_epi16(diff, _mm_setzero_si128());

        let lec1 = _mm_and_si128(nvce, _mm_and_si128(diff0, ncarry));
        let lec2 = _mm_and_si128(n64.rsp.vector_control.vce, _mm_or_si128(diff0, ncarry));
        let leeq = _mm_or_si128(lec1, lec2);

        let geeq = _mm_cmpeq_epi16(_mm_subs_epu16(vte, vs), _mm_setzero_si128());
        let le = _mm_blendv_epi8(
            n64.rsp.vector_control.vccl,
            leeq,
            _mm_andnot_si128(n64.rsp.vector_control.vcoh, n64.rsp.vector_control.vcol),
        );

        let ge = _mm_blendv_epi8(
            geeq,
            n64.rsp.vector_control.vcch,
            _mm_or_si128(n64.rsp.vector_control.vcol, n64.rsp.vector_control.vcoh),
        );

        let mask = _mm_blendv_epi8(ge, le, n64.rsp.vector_control.vcol);
        n64.rsp.accumulator.accl = _mm_blendv_epi8(vs, nvt, mask);

        // Update condition codes
        n64.rsp.vector_control.vcch = ge;
        n64.rsp.vector_control.vccl = le;
        n64.rsp.vector_control.vcoh = _mm_setzero_si128();
        n64.rsp.vector_control.vcol = _mm_setzero_si128();
        n64.rsp.vector_control.vce = _mm_setzero_si128();

        set_vd_register(n64, opcode, n64.rsp.accumulator.accl);
    }
}

fn vch(n64: &mut N64, opcode: u32) {
    let vte = vte(n64, vt(opcode), ve(opcode) as usize);
    unsafe {
        let vs = get_vs_register(n64, opcode);

        n64.rsp.vector_control.vcol = _mm_cmplt_epi16(_mm_xor_si128(vs, vte), _mm_setzero_si128());
        let nvt = _mm_sub_epi16(
            _mm_xor_si128(vte, n64.rsp.vector_control.vcol),
            n64.rsp.vector_control.vcol,
        );
        let diff = _mm_sub_epi16(vs, nvt);

        let diff0 = _mm_cmpeq_epi16(diff, _mm_setzero_si128());
        let vtn = _mm_cmplt_epi16(vte, _mm_setzero_si128());
        let dlez = _mm_cmpeq_epi16(
            _mm_setzero_si128(),
            _mm_cmpgt_epi16(diff, _mm_setzero_si128()),
        );
        let dgez = _mm_or_si128(_mm_cmpgt_epi16(diff, _mm_setzero_si128()), diff0);

        n64.rsp.vector_control.vcch = _mm_blendv_epi8(dgez, vtn, n64.rsp.vector_control.vcol);
        n64.rsp.vector_control.vccl = _mm_blendv_epi8(vtn, dlez, n64.rsp.vector_control.vcol);
        n64.rsp.vector_control.vce = _mm_and_si128(
            _mm_cmpeq_epi16(diff, n64.rsp.vector_control.vcol),
            n64.rsp.vector_control.vcol,
        );
        n64.rsp.vector_control.vcoh = _mm_cmpeq_epi16(
            _mm_or_si128(diff0, n64.rsp.vector_control.vce),
            _mm_setzero_si128(),
        );

        let mask = _mm_blendv_epi8(
            n64.rsp.vector_control.vcch,
            n64.rsp.vector_control.vccl,
            n64.rsp.vector_control.vcol,
        );
        n64.rsp.accumulator.accl = _mm_blendv_epi8(vs, nvt, mask);

        set_vd_register(n64, opcode, n64.rsp.accumulator.accl);
    }
}

fn vcr(n64: &mut N64, opcode: u32) {
    let vte = vte(n64, vt(opcode), ve(opcode) as usize);
    unsafe {
        let vs = get_vs_register(n64, opcode);
        let sign = _mm_srai_epi16(_mm_xor_si128(vs, vte), 15);

        let dlez = _mm_add_epi16(_mm_and_si128(vs, sign), vte);
        n64.rsp.vector_control.vccl = _mm_srai_epi16(dlez, 15);

        let dgez = _mm_min_epi16(_mm_or_si128(vs, sign), vte);
        n64.rsp.vector_control.vcch = _mm_cmpeq_epi16(dgez, vte);

        let nvt = _mm_xor_si128(vte, sign);
        let mask = _mm_blendv_epi8(
            n64.rsp.vector_control.vcch,
            n64.rsp.vector_control.vccl,
            sign,
        );
        n64.rsp.accumulator.accl = _mm_blendv_epi8(vs, nvt, mask);

        set_vd_register(n64, opcode, n64.rsp.accumulator.accl);

        // Clear condition codes
        n64.rsp.vector_control.vcol = _mm_setzero_si128();
        n64.rsp.vector_control.vcoh = _mm_setzero_si128();
        n64.rsp.vector_control.vce = _mm_setzero_si128();
    }
}

macro_rules! vector_op {
    ($name:ident, $op:expr) => {
        fn $name(n64: &mut N64, opcode: u32) {
            let vte = vte(n64, vt(opcode), ve(opcode) as usize);
            unsafe {
                n64.rsp.accumulator.accl = $op(
                    std::mem::transmute::<u128, std::arch::x86_64::__m128i>(
                        n64.rsp.registers.vpr[vs(opcode) as usize],
                    ),
                    vte,
                );
                n64.rsp.registers.vpr[vd(opcode) as usize] = std::mem::transmute::<
                    std::arch::x86_64::__m128i,
                    u128,
                >(n64.rsp.accumulator.accl);
            }
        }
    };
}

macro_rules! vector_op_with_xor {
    ($name:ident, $op:expr) => {
        fn $name(n64: &mut N64, opcode: u32) {
            let vte = vte(n64, vt(opcode), ve(opcode) as usize);
            unsafe {
                n64.rsp.accumulator.accl = $op(
                    std::mem::transmute::<u128, std::arch::x86_64::__m128i>(
                        n64.rsp.registers.vpr[vs(opcode) as usize],
                    ),
                    vte,
                );
                n64.rsp.accumulator.accl =
                    _mm_xor_si128(n64.rsp.accumulator.accl, _mm_set1_epi32(-1));
                n64.rsp.registers.vpr[vd(opcode) as usize] = std::mem::transmute::<
                    std::arch::x86_64::__m128i,
                    u128,
                >(n64.rsp.accumulator.accl);
            }
        }
    };
}

vector_op!(vand, _mm_and_si128);
vector_op!(vor, _mm_or_si128);
vector_op!(vxor, _mm_xor_si128);

vector_op_with_xor!(vnand, _mm_and_si128);
vector_op_with_xor!(vnor, _mm_or_si128);
vector_op_with_xor!(vnxor, _mm_xor_si128);

fn vmrg(n64: &mut N64, opcode: u32) {
    let vte = vte(n64, vt(opcode), ve(opcode) as usize);
    unsafe {
        n64.rsp.accumulator.accl = _mm_blendv_epi8(
            vte,
            std::mem::transmute::<u128, std::arch::x86_64::__m128i>(
                n64.rsp.registers.vpr[vs(opcode) as usize],
            ),
            n64.rsp.vector_control.vccl,
        );
        n64.rsp.vector_control.vcoh = _mm_setzero_si128();
        n64.rsp.vector_control.vcol = _mm_setzero_si128();
        n64.rsp.registers.vpr[vd(opcode) as usize] =
            std::mem::transmute::<std::arch::x86_64::__m128i, u128>(n64.rsp.accumulator.accl);
    }
}

macro_rules! vrcp_common {
    ($n64:expr, $opcode:expr, $input:expr) => {{
        let mut result;
        let vte = vte($n64, vt($opcode), ve($opcode) as usize);
        let input = $input;
        let mask = input >> 31;
        let mut data = input ^ mask;
        if input > -32768 {
            data -= mask
        }
        if data == 0 {
            result = 0x7fffffff
        } else if input == -32768 {
            result = 0xffff0000
        } else {
            let shift = count_leading_zeros(data as u32);
            let index = (((data as u64) << shift) & 0x7fc00000) >> 22;
            result = $n64.rsp.vector.reciprocals[index as usize] as u32;
            result = (0x10000 | result) << 14;
            result = (result >> (31 - shift)) ^ mask as u32
        }
        $n64.rsp.division.divdp = false;
        $n64.rsp.division.divout = (result >> 16) as i16;
        $n64.rsp.accumulator.accl = vte;
        modify_vpr_element(
            &mut $n64.rsp.registers.vpr[vd($opcode) as usize],
            result as u16,
            de($opcode) as u8,
        );
    }};
}

fn vrcp(n64: &mut N64, opcode: u32) {
    vrcp_common!(n64, opcode, {
        get_vpr_element(n64.rsp.registers.vpr[vt(opcode) as usize], ve(opcode) as u8) as i16 as i32
    });
}

fn vrcpl(n64: &mut N64, opcode: u32) {
    vrcp_common!(n64, opcode, {
        if n64.rsp.division.divdp {
            (n64.rsp.division.divin as i32) << 16
                | get_vpr_element(n64.rsp.registers.vpr[vt(opcode) as usize], ve(opcode) as u8)
                    as u16 as i32
        } else {
            get_vpr_element(n64.rsp.registers.vpr[vt(opcode) as usize], ve(opcode) as u8) as i16
                as i32
        }
    });
}

macro_rules! vrcp_vrsq_common {
    ($n64:expr, $opcode:expr) => {{
        let vte = vte($n64, vt($opcode), ve($opcode) as usize);
        $n64.rsp.accumulator.accl = vte;
        $n64.rsp.division.divdp = true;

        $n64.rsp.division.divin = get_vpr_element(
            $n64.rsp.registers.vpr[vt($opcode) as usize],
            ve($opcode) as u8,
        ) as i16;
        modify_vpr_element(
            &mut $n64.rsp.registers.vpr[vd($opcode) as usize],
            $n64.rsp.division.divout as u16,
            de($opcode) as u8,
        );
    }};
}

fn vrcph(n64: &mut N64, opcode: u32) {
    vrcp_vrsq_common!(n64, opcode)
}
fn vrsqh(n64: &mut N64, opcode: u32) {
    vrcp_vrsq_common!(n64, opcode)
}

fn vmov(n64: &mut N64, opcode: u32) {
    let vte = vte(n64, vt(opcode), ve(opcode) as usize);
    let value = get_vpr_element(
        unsafe { std::mem::transmute::<std::arch::x86_64::__m128i, u128>(vte) },
        de(opcode) as u8,
    );
    modify_vpr_element(
        &mut n64.rsp.registers.vpr[vd(opcode) as usize],
        value,
        de(opcode) as u8,
    );
    n64.rsp.accumulator.accl = vte;
}

macro_rules! vrsq_common {
    ($n64:expr, $opcode:expr, $input:expr) => {{
        let mut result;
        let vte = vte($n64, vt($opcode), ve($opcode) as usize);
        let input = $input;
        let mask = input >> 31;
        let mut data = input ^ mask;
        if input > -32768 {
            data -= mask
        }
        if data == 0 {
            result = 0x7fffffff
        } else if input == -32768 {
            result = 0xffff0000
        } else {
            let shift = count_leading_zeros(data as u32);
            let index = (((data as u64) << shift) & 0x7fc00000) as u32 >> 22;
            result = $n64.rsp.vector.inverse_square_roots[((index & 0x1fe) | (shift & 1)) as usize]
                as u32;
            result = (0x10000 | result) << 14;
            result = (result >> ((31 - shift) >> 1)) ^ mask as u32
        }
        $n64.rsp.division.divdp = false;
        $n64.rsp.division.divout = (result >> 16) as i16;
        $n64.rsp.accumulator.accl = vte;
        modify_vpr_element(
            &mut $n64.rsp.registers.vpr[vd($opcode) as usize],
            result as u16,
            de($opcode) as u8,
        );
    }};
}

fn vrsq(n64: &mut N64, opcode: u32) {
    vrsq_common!(
        n64,
        opcode,
        get_vpr_element(n64.rsp.registers.vpr[vt(opcode) as usize], ve(opcode) as u8) as i16 as i32
    );
}

fn vrsql(n64: &mut N64, opcode: u32) {
    vrsq_common!(
        n64,
        opcode,
        if n64.rsp.division.divdp {
            (n64.rsp.division.divin as i32) << 16
                | get_vpr_element(n64.rsp.registers.vpr[vt(opcode) as usize], ve(opcode) as u8)
                    as u16 as i32
        } else {
            get_vpr_element(n64.rsp.registers.vpr[vt(opcode) as usize], ve(opcode) as u8) as i16
                as i32
        }
    );
}

fn vnop(_device: &mut N64, _opcode: u32) {}

fn execute_vec(n64: &mut N64, opcode: u32) {
    n64.rsp.instruction.instruction_type = rsp::InstructionType::Vu;
    n64.rsp.instruction_tables.vector_table[(opcode & 0x3F) as usize](n64, opcode)
}
