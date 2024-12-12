use super::{builder::registers::Registers, mmu::{MmuCause, MmuRegister}, events::{create_event, EventType}, exceptions::{self, check_pending_interrupts}, memory::{self, AccessSize}, N64};

const MI_INIT_MODE_REG: u32 = 0;
const MI_VERSION_REG: u32 = 1;
const MI_INTR_REG: u32 = 2;
const MI_INTR_MASK_REG: u32 = 3;
pub const MI_REGS_COUNT: u32 = 4;

/* read */
pub const MI_INTR_SP: u32 = 1 << 0;
pub const MI_INTR_SI: u32 = 1 << 1;
pub const MI_INTR_AI: u32 = 1 << 2;
pub const MI_INTR_VI: u32 = 1 << 3;
pub const MI_INTR_PI: u32 = 1 << 4;
pub const MI_INTR_DP: u32 = 1 << 5;

/* write */
const MI_CLR_SP: u32 = 1 << 0;
const MI_SET_SP: u32 = 1 << 1;
const MI_CLR_SI: u32 = 1 << 2;
const MI_SET_SI: u32 = 1 << 3;
const MI_CLR_AI: u32 = 1 << 4;
const MI_SET_AI: u32 = 1 << 5;
const MI_CLR_VI: u32 = 1 << 6;
const MI_SET_VI: u32 = 1 << 7;
const MI_CLR_PI: u32 = 1 << 8;
const MI_SET_PI: u32 = 1 << 9;
const MI_CLR_DP: u32 = 1 << 10;
const MI_SET_DP: u32 = 1 << 11;

/* mode read */
const MI_INIT_MODE: u32 = 1 << 7;
const MI_EBUS_MODE: u32 = 1 << 8;
const MI_RDRAM_MODE: u32 = 1 << 9;

/* mode write */
const MI_CLR_INIT: u32 = 1 << 7;
const MI_SET_INIT: u32 = 1 << 8;
const MI_CLR_EBUS: u32 = 1 << 9;
const MI_SET_EBUS: u32 = 1 << 10;
const MI_CLR_DP_INTR: u32 = 1 << 11;
const MI_CLR_RDRAM: u32 = 1 << 12;
const MI_SET_RDRAM: u32 = 1 << 13;

pub const MI_INIT_LENGTH_MASK: u32 = 0b1111111;

// Essential interrupt flags
pub struct Mips {
    pub regs: Registers,
}

impl Default for Mips {
    fn default() -> Self {
        let mut mips = Self {
            regs: Registers::new()
                .with_size(MI_REGS_COUNT as usize)
                .with_write_mask(MI_INIT_LENGTH_MASK)
                .build(),
        };
        
        mips.regs[MI_VERSION_REG as usize] = 0x02020102;
        mips
    }
}

// Macro to handle common register operations
macro_rules! update_cause_reg {
    ($n64: expr) => {{
        $n64.cpu.mmu.regs[MmuRegister::Cause as usize] &=
            !(MmuCause::ExccodeMask as u64);
        $n64.cpu.mmu.regs[MmuRegister::Cause as usize] |= 
            MmuCause::IP2 as u64;
    }};
}

pub fn read_regs(n64: &mut N64,address: u64, access_size: AccessSize) -> u32 {
    n64.mips.regs.read(n64, address, access_size)
}

pub fn write_regs(n64: &mut N64,address: u64, value: u32, mask: u32) {
    let reg = (address & 0xFFFF) >> 2;
    match reg as u32 {
        MI_INIT_MODE_REG => update_init_mode(n64, value),
        MI_INTR_MASK_REG => update_intr_mask(n64, value),
        _ => memory::masked_write_32(&mut n64.mips.regs[reg as usize], value, mask),
    }

    if n64.mips.regs[MI_INTR_REG as usize] & n64.mips.regs[MI_INTR_MASK_REG as usize] != 0 {
        n64.cpu.mmu.regs[MmuRegister::Cause as usize] &=
            !(MmuCause::ExccodeMask as u64);
        n64.cpu.mmu.regs[MmuRegister::Cause as usize] |= MmuCause::IP2 as u64;
    } else {
        n64.cpu.mmu.regs[MmuRegister::Cause as usize] &=
            !(MmuCause::IP2 as u64);
    }
    check_pending_interrupts(n64)
}

macro_rules! handle_bits {
    ($n64: expr, $reg:expr, $w:expr, $(($clear:expr, $set:expr, $flag:expr)),*) => {
        $(
            if $w & $clear != 0 {
                $n64.mips.regs[$reg as usize] &= !$flag;
            }
            if $w & $set != 0 {
                $n64.mips.regs[$reg as usize] |= $flag;
            }
        )*
    };
}

fn update_init_mode(n64: &mut N64,w: u32) {
    // Handle INIT_LENGTH separately as it's a special case
    n64.mips.regs[MI_INIT_MODE_REG as usize] = 
        (n64.mips.regs[MI_INIT_MODE_REG as usize] & !MI_INIT_LENGTH_MASK) | 
        (w & MI_INIT_LENGTH_MASK);

    handle_bits!(n64, MI_INIT_MODE_REG, w,
        (MI_CLR_INIT, MI_SET_INIT, MI_INIT_MODE),
        (MI_CLR_EBUS, MI_SET_EBUS, MI_EBUS_MODE),
        (MI_CLR_RDRAM, MI_SET_RDRAM, MI_RDRAM_MODE)
    );

    if w & MI_CLR_DP_INTR != 0 {
        clear_rcp_interrupt(n64, MI_INTR_DP);
    }
}

fn update_intr_mask(n64: &mut N64,w: u32) {
    handle_bits!(n64, MI_INTR_MASK_REG, w,
        (MI_CLR_SP, MI_SET_SP, MI_INTR_SP),
        (MI_CLR_SI, MI_SET_SI, MI_INTR_SI),
        (MI_CLR_AI, MI_SET_AI, MI_INTR_AI),
        (MI_CLR_VI, MI_SET_VI, MI_INTR_VI),
        (MI_CLR_PI, MI_SET_PI, MI_INTR_PI),
        (MI_CLR_DP, MI_SET_DP, MI_INTR_DP)
    );
}

pub fn clear_rcp_interrupt(n64: &mut N64,interrupt: u32) {
    n64.mips.regs[MI_INTR_REG as usize] &= !interrupt;

    if n64.mips.regs[MI_INTR_REG as usize] & n64.mips.regs[MI_INTR_MASK_REG as usize] == 0 {
        n64.cpu.mmu.regs[MmuRegister::Cause as usize] &=
            !(MmuCause::IP2 as u64);    
    }
    check_pending_interrupts(n64)
}


fn handle_interrupt_common(n64: &mut N64,interrupt: u32) {
    n64.mips.regs[MI_INTR_REG as usize] |= interrupt;

    if n64.mips.regs[MI_INTR_REG as usize] & n64.mips.regs[MI_INTR_MASK_REG as usize] != 0 {
        update_cause_reg!(n64);
    }
}

pub fn set_rcp_interrupt(n64: &mut N64,interrupt: u32) {
    handle_interrupt_common(n64, interrupt);
    check_pending_interrupts(n64)
}

pub fn schedule_rcp_interrupt(n64: &mut N64,interrupt: u32) {
    handle_interrupt_common(n64, interrupt);
    create_event(n64, EventType::InterruptCheck, n64.cpu.mmu.regs[MmuRegister::Count as usize], exceptions::check_pending_interrupts)
}
