use crate::exceptions;

use super::{mmu::{MmuCause, MmuRegister, MmuStatus}, cpu, rcp, memory::{self, AccessSize, AccessType}, N64};

pub const FCR31_CAUSE_UNIMP_BIT: u32 = 1 << 17;
pub const FCR31_CMP_BIT: u32 = 1 << 23;
pub const FCR31_FS_BIT: u32 = 1 << 24;

pub const FCR31_CAUSE_MASK: u32 = 0b00000000000000111111000000000000;
pub const FCR31_ENABLE_MASK: u32 = 0b00000000000000000000111110000000;
pub const FCR31_WRITE_MASK: u32 = 0b00000001100000111111111111111111;

pub struct Fpu {
    pub fcr0: u32,
    pub fcr31: u32,
    pub flush_mode: u32,
    pub fgr32: [[u8; 4]; 32],
    pub fgr64: [[u8; 8]; 32],
    pub instrs: [fn(&mut N64, u32); 32],
    pub b_instrs: [fn(&mut N64, u32); 4],
    pub s_instrs: [fn(&mut N64, u32); 64],
    pub d_instrs: [fn(&mut N64, u32); 64],
    pub w_instrs: [fn(&mut N64, u32); 64],
    pub l_instrs: [fn(&mut N64, u32); 64],
}

impl Default for Fpu {
    fn default() -> Self {
        fn default_instruction(_device: &mut N64, _value: u32) {}

        let mut fpu = Self {
            fcr0: 0,
            fcr31: 0,
            flush_mode: 0,
            fgr32: [[0; 4]; 32],
            fgr64: [[0; 8]; 32],
            instrs: [default_instruction; 32],
            b_instrs: [default_instruction; 4],
            s_instrs: [default_instruction; 64],
            d_instrs: [default_instruction; 64],
            w_instrs: [default_instruction; 64],
            l_instrs: [default_instruction; 64],
        };

        // Set FGR registers
        let status_reg = 0;
        let is_32bit_mode = (status_reg & (MmuStatus::FR as u64)) == 0;

        (0..32).step_by(2).for_each(|i| {
            if is_32bit_mode {
                let bytes = fpu.fgr64[i];
                fpu.fgr32[i] = bytes[0..4].try_into().unwrap();
                fpu.fgr32[i + 1] = bytes[4..8].try_into().unwrap();
            } else {
                let bytes_lo = fpu.fgr32[i];
                let bytes_hi = fpu.fgr32[i + 1];
                fpu.fgr64[i] = [bytes_lo, bytes_hi].concat().try_into().unwrap();
            }
        });

        // Initialize FCR0
        fpu.fcr0 = 0b101000000000;

        // Initialize instruction tables
        fpu.b_instrs = B_INSTRUCTIONS;
        fpu.s_instrs = init_s_instructions();
        fpu.d_instrs = init_d_instructions();

        let (l_instrs, w_instrs) = init_conversion_instructions();
        fpu.l_instrs = l_instrs;
        fpu.w_instrs = w_instrs;

        fpu.instrs = MAIN_INSTRUCTIONS;

        fpu
    }
}

#[inline(always)]
fn check_fpu_access(n64: &N64) -> bool {
    n64.cpu.mmu.regs[MmuRegister::Status as usize] & (MmuStatus::CU1 as u64) != 0
}

fn handle_fpu_op<F>(n64: &mut N64,opcode: u32, op: F)
where F: FnOnce(&mut N64,u32) {
    if !check_fpu_access(n64) {
        return unusable(n64, opcode);
    }
    op(n64, opcode);
}

fn handle_fpu_mem_op<F>(n64: &mut N64,opcode: u32, access_type: AccessType, op: F) 
where F: FnOnce(&mut N64,u64, bool) {
    if !check_fpu_access(n64) {
        return unusable(n64, opcode);
    }

    let addr = n64.cpu.gpr[cpu::rs(opcode) as usize].wrapping_add(cpu::se16(cpu::imm(opcode) as i16));

    let (phys_addr, cached, err) = memory::translate_address(n64, addr, access_type);
    if err {
        return;
    }

    op(n64, phys_addr, cached);
}

// Helper function to check FR status bit
#[inline(always)]
fn is_fr_mode(n64: &N64) -> bool {
    n64.cpu.mmu.regs[MmuRegister::Status as usize] & (MmuStatus::FR as u64) != 0
}

// Macro for FPR access patterns
macro_rules! fpr_access {
    ($n64: expr, $index:expr, $fr_op:expr, $non_fr_op:expr) => {
        if is_fr_mode($n64) {
            $fr_op
        } else {
            $non_fr_op
        }
    };
}

pub fn lwc1(n64: &mut N64,opcode: u32) {
    handle_fpu_mem_op(n64, opcode, AccessType::Read, |dev, addr, cached| {
        let value = memory::read_data(dev, addr, AccessSize::Word, cached).to_ne_bytes();
        set_fpr_single(
            dev,
            rcp::ft(opcode) as usize,
            f32::from_ne_bytes(value),
            false,
        )
    });
}

pub fn ldc1(n64: &mut N64,opcode: u32) {
    handle_fpu_mem_op(n64, opcode, AccessType::Read, |dev, addr, cached| {
        let w0 = memory::read_data(dev, addr, AccessSize::Dword, cached);
        let w1 = memory::read_data(dev, addr + 4, AccessSize::Dword, cached);
        let value = ((w0 as u64) << 32) | (w1 as u64);
        set_fpr_double(
            dev,
            rcp::ft(opcode) as usize,
            f64::from_ne_bytes(value.to_ne_bytes()),
        )
    });
}

pub fn swc1(n64: &mut N64,opcode: u32) {
    handle_fpu_mem_op(n64, opcode, AccessType::Write, |dev, addr, cached| {
        let value = get_fpr_single(dev, rcp::ft(opcode) as usize).to_ne_bytes();
        memory::write_data(dev, addr, u32::from_ne_bytes(value), 0xFFFFFFFF, cached);
    });
}

pub fn sdc1(n64: &mut N64,opcode: u32) {
    handle_fpu_mem_op(n64, opcode, AccessType::Write, |dev, addr, cached| {
        let value = get_fpr_double(dev, rcp::ft(opcode) as usize).to_ne_bytes();
        memory::write_data( dev, addr, u32::from_ne_bytes(value[4..8].try_into().unwrap()), 0xFFFFFFFF, cached);
        memory::write_data( dev, addr + 4, u32::from_ne_bytes(value[0..4].try_into().unwrap()), 0xFFFFFFFF, cached);
    });
}

fn mfc1(n64: &mut N64,opcode: u32) {
    handle_fpu_op(n64, opcode, |dev, op| {
        let value = get_fpr_single(dev, rcp::fs(op) as usize);
        dev.cpu.gpr[cpu::rt(op) as usize] = cpu::se32(u32::from_ne_bytes(value.to_ne_bytes()) as i32);
    });
}

fn dmfc1(n64: &mut N64,opcode: u32) {
    handle_fpu_op(n64, opcode, |dev, op| {
        let value = get_fpr_double(dev, rcp::fs(op) as usize);
        dev.cpu.gpr[cpu::rt(op) as usize] = u64::from_ne_bytes(value.to_ne_bytes());
    });
}

fn cfc1(n64: &mut N64,opcode: u32) {
    handle_fpu_op(n64, opcode, |dev, op| {
        dev.cpu.gpr[cpu::rt(op) as usize] = cpu::se32(get_control_registers_fpu(dev, rcp::fs(op)) as i32)
    });
}

fn dcfc1(n64: &mut N64,opcode: u32) {
    handle_fpu_op(n64, opcode, |dev, _op| {
        dev.cpu.fpu.fcr31 &= !FCR31_CAUSE_MASK;
        dev.cpu.fpu.fcr31 |= FCR31_CAUSE_UNIMP_BIT;
        exceptions::floating_point_exception(dev)
    });
}

fn mtc1(n64: &mut N64,opcode: u32) {
    handle_fpu_op(n64, opcode, |dev, op| {
        let value = f32::from_ne_bytes(
            (dev.cpu.gpr[cpu::rt(op) as usize] as u32).to_ne_bytes(),
        );
        set_fpr_single(
            dev,
            rcp::fs(op) as usize,
            value,
            false,
        )
    });
}

fn dmtc1(n64: &mut N64,opcode: u32) {
    handle_fpu_op(n64, opcode, |dev, op| {
        let value = f64::from_ne_bytes(dev.cpu.gpr[cpu::rt(op) as usize].to_ne_bytes());
        set_fpr_double(dev, rcp::fs(op) as usize, value)
    });
}

fn ctc1(n64: &mut N64,opcode: u32) {
    handle_fpu_op(n64, opcode, |dev, op| {
        set_control_registers_fpu(
            dev,
            rcp::fs(op),
            dev.cpu.gpr[cpu::rt(op) as usize] as u32,
        )
    });
}

fn execute_fpu_b(n64: &mut N64,opcode: u32) {
    handle_fpu_op(n64, opcode, |dev, op| {
        dev.cpu.fpu.b_instrs[((op >> 16) & 0x3) as usize](dev, op)
    });
}

fn execute_fpu_s(n64: &mut N64,opcode: u32) {
    handle_fpu_op(n64, opcode, |dev, op| {
        dev.cpu.fpu.s_instrs[(op & 0x3F) as usize](dev, op)
    });
}

fn execute_fpu_d(n64: &mut N64,opcode: u32) {
    handle_fpu_op(n64, opcode, |dev, op| {
        dev.cpu.fpu.d_instrs[(op & 0x3F) as usize](dev, op)
    });
}

fn execute_fpu_l(n64: &mut N64,opcode: u32) {
    handle_fpu_op(n64, opcode, |dev, op| {
        dev.cpu.fpu.l_instrs[(op & 0x3F) as usize](dev, op)
    });
}

fn execute_fpu_w(n64: &mut N64,opcode: u32) {
    handle_fpu_op(n64, opcode, |dev, op| {
        dev.cpu.fpu.w_instrs[(op & 0x3F) as usize](dev, op)
    });
}

fn dctc1(n64: &mut N64,opcode: u32) {
    handle_fpu_op(n64, opcode, |dev, op| {
        set_control_registers_fpu(
            dev,
            rcp::fs(op),
            dev.cpu.gpr[cpu::rt(op) as usize] as u32,
        )
    });
}

fn unusable(n64: &mut N64,_opcode: u32) {
    exceptions::cop_unusable_exception(n64, MmuCause::CE1 as u64)
}

pub fn reserved(n64: &mut N64,opcode: u32) {
    handle_fpu_op(n64, opcode, |dev, _op| {
        exceptions::reserved_exception(dev, MmuCause::CE1 as u64)
    });
}

fn handle_fcr31(n64: &mut N64,data: u32) {
    n64.cpu.fpu.fcr31 = data & FCR31_WRITE_MASK;
    
    let cause_enabled = (n64.cpu.fpu.fcr31 & FCR31_CAUSE_MASK) >> 5 
        & (n64.cpu.fpu.fcr31 & FCR31_ENABLE_MASK) != 0;
    let unimp_operation = n64.cpu.fpu.fcr31 & FCR31_CAUSE_UNIMP_BIT != 0;
    
    if cause_enabled || unimp_operation {
        exceptions::floating_point_exception(n64)
    }
}

fn update_flush_mode(n64: &mut N64) {
    unsafe {
        let _flush_mode = 0;
        let flush_mode = if n64.cpu.fpu.fcr31 & 2 != 0 {
            if n64.cpu.fpu.fcr31 & FCR31_FS_BIT != 0 {
                0
            } else {
                0
            }
        } else {
            0
        };

        if flush_mode != n64.cpu.fpu.flush_mode {
            #[allow(deprecated)]
            std::arch::x86_64::_MM_SET_FLUSH_ZERO_MODE(flush_mode);
            n64.cpu.fpu.flush_mode = flush_mode;
        }
    }
}

fn set_control_registers_fpu(n64: &mut N64,index: u32, data: u32) {
    match index {
        0 => (), // read only
        31 => {
            handle_fcr31(n64, data);
            update_flush_mode(n64);
        }
        _ => panic!("unknown FCR register")
    }
}

fn get_control_registers_fpu(n64: &N64, index: u32) -> u32 {
    match index {
        0 => n64.cpu.fpu.fcr0,
        31 => n64.cpu.fpu.fcr31,
        _ => {
            panic!("unknown FCR register")
        }
    }
}

pub fn set_fpr_single(n64: &mut N64,index: usize, value: f32, clear_high: bool) {
    let bytes = value.to_ne_bytes();
    if n64.cpu.mmu.regs[MmuRegister::Status as usize] & (MmuStatus::FR as u64) == 0 {
        n64.cpu.fpu.fgr32[index] = bytes;
    } else {
        let bytes_lo = bytes;
        let bytes_hi: [u8; 4] = if clear_high {
            [0, 0, 0, 0]
        } else {
            n64.cpu.fpu.fgr64[index][4..8].try_into().unwrap()
        };
        n64.cpu.fpu.fgr64[index] = [bytes_lo, bytes_hi].concat().try_into().unwrap();
    }
}

pub fn get_fpr_single(n64: &N64, index: usize) -> f32 {
    fpr_access!(
        n64,
        index,
        f32::from_ne_bytes(n64.cpu.fpu.fgr64[index][0..4].try_into().unwrap()),
        f32::from_ne_bytes(n64.cpu.fpu.fgr32[index])
    )
}

pub fn set_fpr_double(n64: &mut N64,index: usize, value: f64) {
    let bytes = value.to_ne_bytes();
    fpr_access!(
        n64,
        index,
        n64.cpu.fpu.fgr64[index] = bytes,
        {
            n64.cpu.fpu.fgr32[index & !1] = bytes[0..4].try_into().unwrap();
            n64.cpu.fpu.fgr32[(index & !1) + 1] = bytes[4..8].try_into().unwrap();
        }
    )
}

pub fn get_fpr_double(n64: &N64, index: usize) -> f64 {
    fpr_access!(
        n64,
        index,
        f64::from_ne_bytes(n64.cpu.fpu.fgr64[index]),
        {
            let bytes_lo = n64.cpu.fpu.fgr32[index & !1];
            let bytes_hi = n64.cpu.fpu.fgr32[(index & !1) + 1];
            f64::from_ne_bytes([bytes_lo, bytes_hi].concat().try_into().unwrap())
        }
    )
}

const B_INSTRUCTIONS: [fn(&mut N64,u32); 4] = [
    rcp::bc1f,
    rcp::bc1t,
    rcp::bc1fl,
    rcp::bc1tl,
];

const MAIN_INSTRUCTIONS: [fn(&mut N64,u32); 32] = {
    // Define the reserved function with matching signature first
    let reserved: fn(&mut N64,u32) = reserved;
    
    let mut arr = [reserved; 32];
    arr[0] = mfc1;
    arr[1] = dmfc1;
    arr[2] = cfc1;
    arr[3] = dcfc1;
    arr[4] = mtc1;
    arr[5] = dmtc1;
    arr[6] = ctc1;
    arr[7] = dctc1;
    arr[8] = execute_fpu_b;
    arr[16] = execute_fpu_s;
    arr[17] = execute_fpu_d;
    arr[20] = execute_fpu_w;
    arr[21] = execute_fpu_l;
    arr
};

fn init_s_instructions() -> [fn(&mut N64,u32); 64] {
    // Correctly declare reserved function type
    let reserved: fn(&mut N64,u32) = reserved;
    let mut arr = [reserved; 64];
    
    // Basic arithmetic
    arr[0..8].copy_from_slice(&[
        rcp::add_s,
        rcp::sub_s, 
        rcp::mul_s,
        rcp::div_s,
        rcp::sqrt_s,
        rcp::abs_s,
        rcp::mov_s,
        rcp::neg_s,
    ]);

    // Conversions
    arr[8..16].copy_from_slice(&[
        rcp::round_l_s,
        rcp::trunc_l_s,
        rcp::ceil_l_s,
        rcp::floor_l_s,
        rcp::round_w_s,
        rcp::trunc_w_s,
        rcp::ceil_w_s,
        rcp::floor_w_s,
    ]);

    // Type conversions
    arr[33] = rcp::cvt_d_s;
    arr[36] = rcp::cvt_w_s;
    arr[37] = rcp::cvt_l_s;

    // Comparisons
    arr[48..64].copy_from_slice(&[
        rcp::c_f_s,
        rcp::c_un_s,
        rcp::c_eq_s,
        rcp::c_ueq_s,
        rcp::c_olt_s,
        rcp::c_ult_s,
        rcp::c_ole_s,
        rcp::c_ule_s,
        rcp::c_sf_s,
        rcp::c_ngle_s,
        rcp::c_seq_s,
        rcp::c_ngl_s,
        rcp::c_lt_s,
        rcp::c_nge_s,
        rcp::c_le_s,
        rcp::c_ngt_s,
    ]);

    arr
}

fn init_d_instructions() -> [fn(&mut N64,u32); 64] {
    let reserved: fn(&mut N64,u32) = reserved;
    let mut arr = [reserved; 64];
    
    // Basic arithmetic
    arr[0..8].copy_from_slice(&[
        rcp::add_d,
        rcp::sub_d,
        rcp::mul_d,
        rcp::div_d,
        rcp::sqrt_d,
        rcp::abs_d,
        rcp::mov_d,
        rcp::neg_d,
    ]);

    // Conversions
    arr[8..16].copy_from_slice(&[
        rcp::round_l_d,
        rcp::trunc_l_d,
        rcp::ceil_l_d,
        rcp::floor_l_d,
        rcp::round_w_d,
        rcp::trunc_w_d,
        rcp::ceil_w_d,
        rcp::floor_w_d,
    ]);

    // Type conversions
    arr[32] = rcp::cvt_s_d;
    arr[36] = rcp::cvt_w_d;
    arr[37] = rcp::cvt_l_d;

    // Comparisons
    arr[48..64].copy_from_slice(&[
        rcp::c_f_d,
        rcp::c_un_d,
        rcp::c_eq_d,
        rcp::c_ueq_d,
        rcp::c_olt_d,
        rcp::c_ult_d,
        rcp::c_ole_d,
        rcp::c_ule_d,
        rcp::c_sf_d,
        rcp::c_ngle_d,
        rcp::c_seq_d,
        rcp::c_ngl_d,
        rcp::c_lt_d,
        rcp::c_nge_d,
        rcp::c_le_d,
        rcp::c_ngt_d,
    ]);

    arr
}

fn init_conversion_instructions() -> ([fn(&mut N64,u32); 64], [fn(&mut N64,u32); 64]) {
    let reserved: fn(&mut N64,u32) = reserved;
    let mut l_arr = [reserved; 64];
    let mut w_arr = [reserved; 64];

    // L format conversions
    l_arr[32] = rcp::cvt_s_l;
    l_arr[33] = rcp::cvt_d_l;

    // W format conversions
    w_arr[32] = rcp::cvt_s_w;
    w_arr[33] = rcp::cvt_d_w;

    (l_arr, w_arr)
}

pub fn set_fgr_registers(n64: &mut N64,status_reg: u64) {
    let is_32bit_mode = (status_reg & (MmuStatus::FR as u64)) == 0;
    
    (0..32).step_by(2).for_each(|i| {
        if is_32bit_mode {
            split_64bit_to_32bit(n64, i);
        } else {
            merge_32bit_to_64bit(n64, i);
        }
    });
}

#[inline]
fn split_64bit_to_32bit(n64: &mut N64,i: usize) {
    let bytes = n64.cpu.fpu.fgr64[i];
    n64.cpu.fpu.fgr32[i] = bytes[0..4].try_into().unwrap();
    n64.cpu.fpu.fgr32[i + 1] = bytes[4..8].try_into().unwrap();
}

#[inline]
fn merge_32bit_to_64bit(n64: &mut N64,i: usize) {
    let bytes_lo = n64.cpu.fpu.fgr32[i];
    let bytes_hi = n64.cpu.fpu.fgr32[i + 1];
    n64.cpu.fpu.fgr64[i] = [bytes_lo, bytes_hi].concat().try_into().unwrap();
}