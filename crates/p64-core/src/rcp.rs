use crate::N64;
use super::{mmu::{add_cycles, MmuCause, MmuRegister, MmuStatus}, fpu::{self, set_fpr_single, FCR31_CMP_BIT}, cpu::{self, check_relative_idle_loop, rt, se32, State}, exceptions::{cop_unusable_exception, reserved_exception}};

pub struct Rcp {
    pub instrs: [fn(&mut N64,u32); 32],
    pub reg_latch: u64,
}

impl Default for Rcp {
    fn default() -> Self {
        fn default_instruction(_device: &mut N64, _value: u32) {}

        fn mfc2(d: &mut N64, op: u32) { handle_move(d, op, false, false); }
        fn dmfc2(d: &mut N64, op: u32) { handle_move(d, op, true, false); }
        fn cfc2(d: &mut N64, op: u32) { handle_move(d, op, false, false); }
        fn mtc2(d: &mut N64, op: u32) { handle_move(d, op, false, true); }
        fn dmtc2(d: &mut N64, op: u32) { handle_move(d, op, true, true); }
        fn ctc2(d: &mut N64, op: u32) { handle_move(d, op, false, true); }

        let mut instrs: [fn(&mut N64, u32); 32] = [default_instruction; 32];
        let functions: [fn(&mut N64, u32); 6] = [mfc2, dmfc2, cfc2, mtc2, dmtc2, ctc2];

        for (i, &f) in functions.iter().enumerate() {
            instrs[i] = f;
        }

        Rcp {
            instrs,
            reg_latch: 0,
        }
    }
}

#[inline(always)] 
fn check_rcp_access(n64: &N64) -> bool {
    n64.cpu.mmu.regs[MmuRegister::Status as usize] & (MmuStatus::CU2 as u64) != 0
}

fn handle_move(n64: &mut N64,opcode: u32, double: bool, to_rcp: bool) {
    if !check_rcp_access(n64) {
        return cop_unusable_exception(n64, MmuCause::CE2 as u64);
    }
    let rt = rt(opcode) as usize;
    if to_rcp {
        n64.cpu.rcp.reg_latch = n64.cpu.gpr[rt]
    } else {
        n64.cpu.gpr[rt] = if double { n64.cpu.rcp.reg_latch } else { se32(n64.cpu.rcp.reg_latch as u32 as i32) }
    }
}

pub fn vector_reserved(n64: &mut N64,_opcode: u32) { reserved_exception(n64, MmuCause::CE2 as u64); }

pub fn fs(opcode: u32) -> u32 {
    (opcode >> 11) & 0x1F
}

pub fn ft(opcode: u32) -> u32 {
    (opcode >> 16) & 0x1F
}

pub fn fd(opcode: u32) -> u32 {
    (opcode >> 6) & 0x1F
}


fn handle_branch(n64: &mut N64,opcode: u32, condition: bool, discard: bool) {
    if condition {
        check_relative_idle_loop(n64, opcode);
        n64.cpu.branch_state.state = State::Take;
        n64.cpu.branch_state.pc = n64.cpu.pc.wrapping_add(
            cpu::se16(cpu::imm(opcode) as i16) << 2,
        ) + 4;
    } else {
        n64.cpu.branch_state.state = if discard {
            State::Discard
        } else {
            State::NotTaken
        };
    }
}

pub fn bc1f(n64: &mut N64,opcode: u32) {
    handle_branch(n64, opcode, n64.cpu.fpu.fcr31 & FCR31_CMP_BIT == 0, false);
}

pub fn bc1t(n64: &mut N64,opcode: u32) {
    handle_branch(n64, opcode, n64.cpu.fpu.fcr31 & FCR31_CMP_BIT != 0, false);
}

pub fn bc1fl(n64: &mut N64,opcode: u32) {
    handle_branch(n64, opcode, n64.cpu.fpu.fcr31 & FCR31_CMP_BIT == 0, true);
}

pub fn bc1tl(n64: &mut N64,opcode: u32) {
    handle_branch(n64, opcode, n64.cpu.fpu.fcr31 & FCR31_CMP_BIT != 0, true);
}

fn perform_rcp_operation<F>(n64: &mut N64,opcode: u32, operation: F, cycles: u32)
where
    F: Fn(f32, f32) -> f32,
{
    let fs = fpu::get_fpr_single(n64, fs(opcode) as usize);
    let ft = fpu::get_fpr_single(n64, ft(opcode) as usize);
    fpu::set_fpr_single(n64, fd(opcode) as usize, operation(fs, ft), true);
    add_cycles(n64, cycles as u64);
}

fn perform_rcp_unary_operation<F>(n64: &mut N64,opcode: u32, operation: F, cycles: u32)
where
    F: Fn(f32) -> f32,
{
    let fs = fpu::get_fpr_single(n64, fs(opcode) as usize);
    fpu::set_fpr_single(n64, fd(opcode) as usize, operation(fs), true);
    add_cycles(n64, cycles as u64);
}

pub fn add_s(n64: &mut N64,opcode: u32) {
    perform_rcp_operation(n64, opcode, |fs, ft| fs + ft, 2);
}

pub fn sub_s(n64: &mut N64,opcode: u32) {
    perform_rcp_operation(n64, opcode, |fs, ft| fs - ft, 2);
}

pub fn mul_s(n64: &mut N64,opcode: u32) {
    perform_rcp_operation(n64, opcode, |fs, ft| fs * ft, 4);
}

pub fn div_s(n64: &mut N64,opcode: u32) {
    perform_rcp_operation(n64, opcode, |fs, ft| fs / ft, 28);
}

pub fn sqrt_s(n64: &mut N64,opcode: u32) {
    perform_rcp_unary_operation(n64, opcode, |fs| fs.sqrt(), 28);
}

pub fn abs_s(n64: &mut N64,opcode: u32) {
    perform_rcp_unary_operation(n64, opcode, |fs| fs.abs(), 2);
}

pub fn mov_s(n64: &mut N64,opcode: u32) {
    let fs = fpu::get_fpr_double(n64, fs(opcode) as usize);
    fpu::set_fpr_double(n64, fd(opcode) as usize, fs);
}

pub fn neg_s(n64: &mut N64,opcode: u32) {
    let fs = fpu::get_fpr_single(n64, fs(opcode) as usize);
    fpu::set_fpr_single(n64, fd(opcode) as usize, -fs, true);
}

fn perform_rcp_conversion<F, T>(n64: &mut N64,opcode: u32, operation: F, cycles: u32)
where
    F: Fn(f32) -> T,
    T: Into<f64>,
{
    let fs = fpu::get_fpr_single(n64, fs(opcode) as usize);
    let value = operation(fs).into();
    fpu::set_fpr_double(n64, fd(opcode) as usize, value);
    add_cycles(n64, cycles as u64);
}

fn perform_rcp_conversion_single<F>(n64: &mut N64,opcode: u32, operation: F, cycles: u32)
where
    F: Fn(f32) -> f32,
{
    let fs = fpu::get_fpr_single(n64, fs(opcode) as usize);
    let value = operation(fs);
    fpu::set_fpr_single(n64, fd(opcode) as usize, value, true);
    add_cycles(n64, cycles as u64);
}

pub fn round_l_s(n64: &mut N64,opcode: u32) {
    perform_rcp_conversion(n64, opcode, |fs| f64::from_ne_bytes((fs.round_ties_even() as i64).to_ne_bytes()), 4);
}

pub fn trunc_l_s(n64: &mut N64,opcode: u32) {
    perform_rcp_conversion(n64, opcode, |fs| f64::from_ne_bytes((fs.trunc() as i64).to_ne_bytes()), 4);
}

pub fn ceil_l_s(n64: &mut N64,opcode: u32) {
    perform_rcp_conversion(n64, opcode, |fs| f64::from_ne_bytes((fs.ceil() as i64).to_ne_bytes()), 4);
}

pub fn floor_l_s(n64: &mut N64,opcode: u32) {
    perform_rcp_conversion(n64, opcode, |fs| f64::from_ne_bytes((fs.floor() as i64).to_ne_bytes()), 4);
}

pub fn round_w_s(n64: &mut N64,opcode: u32) {
    perform_rcp_conversion_single(n64, opcode, |fs| f32::from_ne_bytes((fs.round_ties_even() as i32).to_ne_bytes()), 4);
}

pub fn trunc_w_s(n64: &mut N64,opcode: u32) {
    perform_rcp_conversion_single(n64, opcode, |fs| f32::from_ne_bytes((fs.trunc() as i32).to_ne_bytes()), 4);
}

pub fn ceil_w_s(n64: &mut N64,opcode: u32) {
    perform_rcp_conversion_single(n64, opcode, |fs| f32::from_ne_bytes((fs.ceil() as i32).to_ne_bytes()), 4);
}

pub fn floor_w_s(n64: &mut N64,opcode: u32) {
    perform_rcp_conversion_single(n64, opcode, |fs| f32::from_ne_bytes((fs.floor() as i32).to_ne_bytes()), 4);
}

pub fn cvt_d_s(n64: &mut N64,opcode: u32) {
    let fs = fpu::get_fpr_single(n64, fs(opcode) as usize);
    fpu::set_fpr_double(n64, fd(opcode) as usize, fs as f64);
    add_cycles(n64, 4);
}

fn convert_fpr(n64: &mut N64,opcode: u32, round_fn: fn(&mut N64, u32), trunc_fn: fn(&mut N64, u32), ceil_fn: fn(&mut N64, u32), floor_fn: fn(&mut N64, u32)) {
    match n64.cpu.fpu.fcr31 & 3 {
        0 => round_fn(n64, opcode),
        1 => trunc_fn(n64, opcode),
        2 => ceil_fn(n64, opcode),
        3 => floor_fn(n64, opcode),
        _ => panic!("unknown conversion type"),
    }
}

pub fn cvt_w_s(n64: &mut N64,opcode: u32) {
    convert_fpr(n64, opcode, round_w_s, trunc_w_s, ceil_w_s, floor_w_s);
}

pub fn cvt_l_s(n64: &mut N64,opcode: u32) {
    convert_fpr(n64, opcode, round_l_s, trunc_l_s, ceil_l_s, floor_l_s);
}

pub fn c_f_s(n64: &mut N64,_opcode: u32) {
    n64.cpu.fpu.fcr31 &= !fpu::FCR31_CMP_BIT
}

fn set_cmp_bit(n64: &mut N64,condition: bool) {
    if condition {
        n64.cpu.fpu.fcr31 |= fpu::FCR31_CMP_BIT;
    } else {
        n64.cpu.fpu.fcr31 &= !fpu::FCR31_CMP_BIT;
    }
}

fn compare_floats(n64: &mut N64,opcode: u32, cmp: fn(f32, f32) -> bool) {
    let fs = fpu::get_fpr_single(n64, fs(opcode) as usize);
    let ft = fpu::get_fpr_single(n64, ft(opcode) as usize);
    if fs.is_nan() || ft.is_nan() {
        set_cmp_bit(n64, false);
    } else {
        set_cmp_bit(n64, cmp(fs, ft));
    }
}

fn get_fpr_values(n64: &mut N64,opcode: u32) -> (f32, f32) {
    let fs = fpu::get_fpr_single(n64, fs(opcode) as usize);
    let ft = fpu::get_fpr_single(n64, ft(opcode) as usize);
    (fs, ft)
}

pub fn c_un_s(n64: &mut N64,opcode: u32) {
    let (fs, ft) = get_fpr_values(n64, opcode);
    set_cmp_bit(n64, fs.is_nan() || ft.is_nan());
}

pub fn c_eq_s(n64: &mut N64,opcode: u32) {
    compare_floats(n64, opcode, |fs, ft| fs == ft);
}

pub fn c_ueq_s(n64: &mut N64,opcode: u32) {
    let (fs, ft) = get_fpr_values(n64, opcode);
    set_cmp_bit(n64, fs.is_nan() || ft.is_nan() || fs == ft);
}

pub fn c_olt_s(n64: &mut N64,opcode: u32) {
    compare_floats(n64, opcode, |fs, ft| fs < ft);
}

pub fn c_ult_s(n64: &mut N64,opcode: u32) {
    let (fs, ft) = get_fpr_values(n64, opcode);
    set_cmp_bit(n64, fs.is_nan() || ft.is_nan() || fs < ft);
}

pub fn c_ole_s(n64: &mut N64,opcode: u32) {
    compare_floats(n64, opcode, |fs, ft| fs <= ft);
}

pub fn c_ule_s(n64: &mut N64,opcode: u32) {
    let (fs, ft) = get_fpr_values(n64, opcode);
    set_cmp_bit(n64, fs.is_nan() || ft.is_nan() || fs <= ft);
}

pub fn c_sf_s(n64: &mut N64,_opcode: u32) {
    set_cmp_bit(n64, false);
}

pub fn c_ngle_s(n64: &mut N64,_opcode: u32) {
    set_cmp_bit(n64, false);
}

pub fn c_seq_s(n64: &mut N64,opcode: u32) {
    compare_floats(n64, opcode, |fs, ft| fs == ft);
}

pub fn c_ngl_s(n64: &mut N64,opcode: u32) {
    compare_floats(n64, opcode, |fs, ft| fs == ft);
}

pub fn c_lt_s(n64: &mut N64,opcode: u32) {
    compare_floats(n64, opcode, |fs, ft| fs < ft);
}

pub fn c_nge_s(n64: &mut N64,opcode: u32) {
    compare_floats(n64, opcode, |fs, ft| fs < ft);
}

pub fn c_le_s(n64: &mut N64,opcode: u32) {
    compare_floats(n64, opcode, |fs, ft| fs <= ft);
}

pub fn c_ngt_s(n64: &mut N64,opcode: u32) {
    compare_floats(n64, opcode, |fs, ft| fs <= ft);
}

fn get_fpr_double_values(n64: &mut N64,opcode: u32) -> (f64, f64) {
    let fs = fpu::get_fpr_double(n64, fs(opcode) as usize);
    let ft = fpu::get_fpr_double(n64, ft(opcode) as usize);
    (fs, ft)
}

fn set_fpr_double(n64: &mut N64,opcode: u32, value: f64) {
    fpu::set_fpr_double(n64, fd(opcode) as usize, value);
}


pub fn add_d(n64: &mut N64,opcode: u32) {
    let (fs, ft) = get_fpr_double_values(n64, opcode);
    set_fpr_double(n64, opcode, fs + ft);
    add_cycles(n64, 2);
}

pub fn sub_d(n64: &mut N64,opcode: u32) {
    let (fs, ft) = get_fpr_double_values(n64, opcode);
    set_fpr_double(n64, opcode, fs - ft);
    add_cycles(n64, 2);
}

pub fn mul_d(n64: &mut N64,opcode: u32) {
    let (fs, ft) = get_fpr_double_values(n64, opcode);
    set_fpr_double(n64, opcode, fs * ft);
    add_cycles(n64, 7);
}

pub fn div_d(n64: &mut N64,opcode: u32) {
    let (fs, ft) = get_fpr_double_values(n64, opcode);
    set_fpr_double(n64, opcode, fs / ft);
    add_cycles(n64, 57);
}

pub fn sqrt_d(n64: &mut N64,opcode: u32) {
    let fs = fpu::get_fpr_double(n64, fs(opcode) as usize);
    set_fpr_double(n64, opcode, fs.sqrt());
    add_cycles(n64, 57);
}

pub fn abs_d(n64: &mut N64,opcode: u32) {
    let fs = fpu::get_fpr_double(n64, fs(opcode) as usize);
    set_fpr_double(n64, opcode, fs.abs());
    add_cycles(n64, 2);
}

pub fn mov_d(n64: &mut N64,opcode: u32) {
    let fs = fpu::get_fpr_double(n64, fs(opcode) as usize);
    set_fpr_double(n64, opcode, fs);
}

pub fn neg_d(n64: &mut N64,opcode: u32) {
    let fs = fpu::get_fpr_double(n64, fs(opcode) as usize);
    set_fpr_double(n64, opcode, -fs);
}

fn set_fpr_double_with_conversion(n64: &mut N64,opcode: u32, value: i64) {
    let converted_value = f64::from_ne_bytes(value.to_ne_bytes());
    set_fpr_double(n64, opcode, converted_value);
    add_cycles(n64, 4);
}

pub fn round_l_d(n64: &mut N64,opcode: u32) {
    let fs = fpu::get_fpr_double(n64, fs(opcode) as usize);
    set_fpr_double_with_conversion(n64, opcode, fs.round_ties_even() as i64);
}

pub fn trunc_l_d(n64: &mut N64,opcode: u32) {
    let fs = fpu::get_fpr_double(n64, fs(opcode) as usize);
    set_fpr_double_with_conversion(n64, opcode, fs.trunc() as i64);
}

pub fn ceil_l_d(n64: &mut N64,opcode: u32) {
    let fs = fpu::get_fpr_double(n64, fs(opcode) as usize);
    set_fpr_double_with_conversion(n64, opcode, fs.ceil() as i64);
}

pub fn floor_l_d(n64: &mut N64,opcode: u32) {
    let fs = fpu::get_fpr_double(n64, fs(opcode) as usize);
    set_fpr_double_with_conversion(n64, opcode, fs.floor() as i64);
}

fn set_fpr_single_with_conversion(n64: &mut N64,opcode: u32, value: i32) {
    let converted_value = f32::from_ne_bytes(value.to_ne_bytes());
    fpu::set_fpr_single(n64, fd(opcode) as usize, converted_value, true);
    add_cycles(n64, 4);
}

pub fn round_w_d(n64: &mut N64,opcode: u32) {
    let fs = fpu::get_fpr_double(n64, fs(opcode) as usize);
    set_fpr_single_with_conversion(n64, opcode, fs.round_ties_even() as i32);
}

pub fn trunc_w_d(n64: &mut N64,opcode: u32) {
    let fs = fpu::get_fpr_double(n64, fs(opcode) as usize);
    set_fpr_single_with_conversion(n64, opcode, fs.trunc() as i32);
}

pub fn ceil_w_d(n64: &mut N64,opcode: u32) {
    let fs = fpu::get_fpr_double(n64, fs(opcode) as usize);
    set_fpr_single_with_conversion(n64, opcode, fs.ceil() as i32);
}

pub fn floor_w_d(n64: &mut N64,opcode: u32) {
    let fs = fpu::get_fpr_double(n64, fs(opcode) as usize);
    set_fpr_single_with_conversion(n64, opcode, fs.floor() as i32);
}

pub fn cvt_s_d(n64: &mut N64,opcode: u32) {
    let fs = fpu::get_fpr_double(n64, fs(opcode) as usize);
    fpu::set_fpr_single(n64, fd(opcode) as usize, fs as f32, true);
    add_cycles(n64, 4);
}

pub fn cvt_w_d(n64: &mut N64,opcode: u32) {
    match n64.cpu.fpu.fcr31 & 3 {
        0 => round_w_d(n64, opcode),
        1 => trunc_w_d(n64, opcode),
        2 => ceil_w_d(n64, opcode),
        3 => floor_w_d(n64, opcode),
        _ => panic!("unknown cvt_w_d"),
    }
}

pub fn cvt_l_d(n64: &mut N64,opcode: u32) {
    match n64.cpu.fpu.fcr31 & 3 {
        0 => round_l_d(n64, opcode),
        1 => trunc_l_d(n64, opcode),
        2 => ceil_l_d(n64, opcode),
        3 => floor_l_d(n64, opcode),
        _ => panic!("unknown cvt_l_d"),
    }
}

pub fn c_f_d(n64: &mut N64,_opcode: u32) {
    n64.cpu.fpu.fcr31 &= !fpu::FCR31_CMP_BIT;
}

fn compare_doubles(n64: &mut N64,opcode: u32, cmp: fn(f64, f64) -> bool) {
    let (fs, ft) = get_fpr_double_values(n64, opcode);
    if fs.is_nan() || ft.is_nan() {
        set_cmp_bit(n64, false);
    } else {
        set_cmp_bit(n64, cmp(fs, ft));
    }
}

pub fn c_un_d(n64: &mut N64,opcode: u32) {
    let (fs, ft) = get_fpr_double_values(n64, opcode);
    set_cmp_bit(n64, fs.is_nan() || ft.is_nan());
}

pub fn c_eq_d(n64: &mut N64,opcode: u32) {
    compare_doubles(n64, opcode, |fs, ft| fs == ft);
}

pub fn c_ueq_d(n64: &mut N64,opcode: u32) {
    let (fs, ft) = get_fpr_double_values(n64, opcode);
    set_cmp_bit(n64, fs.is_nan() || ft.is_nan() || fs == ft);
}

pub fn c_olt_d(n64: &mut N64,opcode: u32) {
    compare_doubles(n64, opcode, |fs, ft| fs < ft);
}

pub fn c_ult_d(n64: &mut N64,opcode: u32) {
    let (fs, ft) = get_fpr_double_values(n64, opcode);
    set_cmp_bit(n64, fs.is_nan() || ft.is_nan() || fs < ft);
}

pub fn c_ole_d(n64: &mut N64,opcode: u32) {
    compare_doubles(n64, opcode, |fs, ft| fs <= ft);
}

pub fn c_ule_d(n64: &mut N64,opcode: u32) {
    let (fs, ft) = get_fpr_double_values(n64, opcode);
    set_cmp_bit(n64, fs.is_nan() || ft.is_nan() || fs <= ft);
}

pub fn c_sf_d(n64: &mut N64,_opcode: u32) {
    set_cmp_bit(n64, false);
}

pub fn c_ngle_d(n64: &mut N64,_opcode: u32) {
    set_cmp_bit(n64, false);
}

pub fn c_seq_d(n64: &mut N64,opcode: u32) {
    compare_doubles(n64, opcode, |fs, ft| fs == ft);
}

pub fn c_ngl_d(n64: &mut N64,opcode: u32) {
    compare_doubles(n64, opcode, |fs, ft| fs == ft);
}

pub fn c_lt_d(n64: &mut N64,opcode: u32) {
    compare_doubles(n64, opcode, |fs, ft| fs < ft);
}

pub fn c_nge_d(n64: &mut N64,opcode: u32) {
    compare_doubles(n64, opcode, |fs, ft| fs < ft);
}

pub fn c_le_d(n64: &mut N64,opcode: u32) {
    compare_doubles(n64, opcode, |fs, ft| fs <= ft);
}

pub fn c_ngt_d(n64: &mut N64,opcode: u32) {
    compare_doubles(n64, opcode, |fs, ft| fs <= ft);
}

pub fn cvt_s_l(n64: &mut N64,opcode: u32) {
    let fs = i64::from_ne_bytes(fpu::get_fpr_double(n64, fs(opcode) as usize).to_ne_bytes());
    set_fpr_single(n64, fd(opcode) as usize, fs as f32, true);
    add_cycles(n64, 4)
}

pub fn cvt_d_l(n64: &mut N64,opcode: u32) {
    let fs =
        i64::from_ne_bytes(fpu::get_fpr_double(n64, fs(opcode) as usize).to_ne_bytes());
    fpu::set_fpr_double(n64, fd(opcode) as usize, fs as f64);
    add_cycles(n64, 4)
}

pub fn cvt_s_w(n64: &mut N64,opcode: u32) {
    let fs = i32::from_ne_bytes(fpu::get_fpr_single(n64, fs(opcode) as usize).to_ne_bytes());
    fpu::set_fpr_single(n64, fd(opcode) as usize, fs as f32, true);
    add_cycles(n64, 4)
}

pub fn cvt_d_w(n64: &mut N64,opcode: u32) {
    let fs = i32::from_ne_bytes(
        fpu::get_fpr_double(n64, fs(opcode) as usize).to_ne_bytes()[0..4]
            .try_into()
            .unwrap(),
    );
    fpu::set_fpr_double(n64, fd(opcode) as usize, fs as f64);
    add_cycles(n64, 4)
}
