pub const VI_REGS_COUNT: u32 = 14;

use governor::clock::Clock;
use p64_gui::video;

use crate::events;
use crate::mips;

use super::builder::registers::Registers;
use super::mmu::MmuRegister;
use super::events::create_event;
use super::events::EventType;
use super::memory;
use super::mips::clear_rcp_interrupt;
use super::N64;

pub enum ViRegister {
    Status = 0,
    Current = 4,
    VSync = 6,
    HSync = 7,
}

pub struct Video {
    pub regs: Registers,
    pub clock: u64,
    pub delay: u64,
    pub field: u32,
    pub limiter: Option<governor::DefaultDirectRateLimiter>,
    pub count_per_scanline: u64,
}

impl Default for Video {
    fn default() -> Self {
        Self {
            regs: Registers::new()
                .with_size(VI_REGS_COUNT as usize)
                .build(),
            clock: 0,
            delay: 0,
            field: 0,
            limiter: None,
            count_per_scanline: 0,
        }
    }
}

pub fn read_regs(n64: &mut N64, address: u64, _access_size: memory::AccessSize) -> u32 {
    let reg = (address & 0xFFFF) >> 2;
    if reg as u32 == ViRegister::Current as u32 {
        set_current_line(n64)
    }
    n64.video.regs[reg as usize]
}

pub fn write_regs(n64: &mut N64,address: u64, value: u32, mask: u32) {
    let reg = (address & 0xFFFF) >> 2;
    match reg as u32 {
        x if x == ViRegister::Current as u32 => clear_rcp_interrupt(n64, mips::MI_INTR_VI),
        x if x == ViRegister::VSync as u32 || x == ViRegister::HSync as u32 => {
            if n64.video.regs[reg as usize] != value & mask {
                memory::masked_write_32(&mut n64.video.regs[reg as usize], value, mask);
                set_expected_refresh_rate(n64);
                if reg as u32 == ViRegister::VSync as u32 {
                    set_vertical_interrupt(n64);
                }
                video::set_register(reg as u32, n64.video.regs[reg as usize]);
            }
        }
        _ => {
            memory::masked_write_32(&mut n64.video.regs[reg as usize], value, mask);
            video::set_register(reg as u32, n64.video.regs[reg as usize]);
        }
    }
}

pub fn set_cpu_clock_speed(n64: &mut N64) { n64.video.clock = if n64.cartridge.pal { 49656530 } else { 48681812 } }

fn set_expected_refresh_rate(n64: &mut N64) {
    let expected_refresh_rate = n64.video.clock as f64
        / (n64.video.regs[ViRegister::VSync as usize] + 1) as f64
        / ((n64.video.regs[ViRegister::HSync as usize] & 0xFFF) + 1) as f64
        * 2.0;
    n64.video.delay = (n64.cpu.clock_rate as f64 / expected_refresh_rate) as u64;
    n64.video.count_per_scanline =
        n64.video.delay / (n64.video.regs[ViRegister::VSync as usize] + 1) as u64;

    let quota = governor::Quota::with_period(std::time::Duration::from_secs_f64(
        1.0 / expected_refresh_rate,
    ))
    .unwrap();
    n64.video.limiter = Some(governor::RateLimiter::direct(quota))
}

fn set_vertical_interrupt(n64: &mut N64) {
    if events::get_event(n64, EventType::VI).is_none() {
        create_event(n64, EventType::VI, n64.cpu.mmu.regs[MmuRegister::Count as usize] + n64.video.delay, vertical_interrupt_event)
    }
}

fn set_current_line(n64: &mut N64) {
    let delay = n64.video.delay;
    let next_vi = events::get_event(n64, EventType::VI);
    if let Some(next_vi) = next_vi {
        n64.video.regs[ViRegister::Current as usize] = ((delay - (next_vi.count - n64.cpu.mmu.regs[MmuRegister::Count as usize])) / n64.video.count_per_scanline) as u32;

        if n64.video.regs[ViRegister::Current as usize] >= n64.video.regs[ViRegister::VSync as usize] {
            n64.video.regs[ViRegister::Current as usize] -= n64.video.regs[ViRegister::VSync as usize];
        }
    }
    n64.video.regs[ViRegister::Current as usize] = (n64.video.regs[ViRegister::Current as usize] & !1) | n64.video.field;
}

fn vertical_interrupt_event(n64: &mut N64) {
    n64.cpu.running = video::update_screen();
    speed_limiter(n64);
    n64.video.field ^= (n64.video.regs[ViRegister::Status as usize] >> 6) & 0x1;
    mips::set_rcp_interrupt(n64, mips::MI_INTR_VI);
    create_event(n64, EventType::VI, n64.cpu.next_event_count + n64.video.delay, vertical_interrupt_event)
}

fn speed_limiter(n64: &N64) {
    if let Err(outcome) = n64.video.limiter.as_ref().unwrap().check() {
        let duration = outcome.wait_time_from(governor::clock::DefaultClock::default().now());
        std::thread::sleep(duration);
        n64.video.limiter.as_ref().unwrap().check().unwrap();
    }
}