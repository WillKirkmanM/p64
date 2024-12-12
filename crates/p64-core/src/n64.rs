use audio::Audio;
use cartridge::{load_rom_save, write_cartridge_to_memory, Cartridge};
use cpu::{start_cpu, Cpu};
use memory::{initialise_memory_map, Memory};
use mips::Mips;
use p64_gui::ui::{load_saves, Ui};
use p64_gui::video::start_window;
use peripheral::Peripheral;
use pif::{set_cartridge_region, Pif};
use rdp::Rdp;
use rdram::{get_rdram_shared_pointer, Rdram, RdramInterface};
use rom::read_rom;
use rsp::{start_rsp, Rsp};
use serial::Serial;
use sram::Flashram;
use video::{set_cpu_clock_speed, Video};
use std::time::Instant;
use tracing::info;

pub mod audio;
pub mod cache;
pub mod cartridge;
pub mod mmu;
pub mod fpu;
pub mod cpu;
pub mod events;
pub mod exceptions;
pub mod rcp;
pub mod memory;
pub mod mempak;
pub mod mips;
pub mod peripheral;
pub mod pif;
pub mod rdp;
pub mod rdram;
pub mod rsp;
pub mod serial;
pub mod sram;
pub mod tlb;
pub mod video;
pub mod builder;
pub mod rom;

#[derive(Default)]
pub struct N64 {
    pub ui: Ui,
    pub byte_swap: usize,
    pub cpu: Cpu,
    pub pif: Pif,
    pub cartridge: Cartridge,
    pub memory: Memory,
    pub rsp: Rsp,
    pub rdp: Rdp,
    pub rdram: Rdram,
    pub mips: Mips,
    pub peripheral: Peripheral,
    pub video: Video,
    pub audio: Audio,
    pub serial: Serial,
    pub rdram_interface: RdramInterface,
    pub flashram: Flashram,
}

impl N64 {
    pub fn new() -> N64 {
        let mut byte_swap: usize = 0;
        let test: [u8; 4] = [1, 2, 3, 4];
        if u32::from_le_bytes(test) == u32::from_ne_bytes(test) {
            byte_swap = 3;
        }
        
        N64 {
            byte_swap,
            ..Default::default()
        }
    }
}

pub fn run_game(file_path: &std::path::Path, device: &mut N64, fullscreen: bool) {
    write_cartridge_to_memory(device, read_rom(file_path));
    
    let (rdram_ptr, rdram_size) = get_rdram_shared_pointer(device);
    
    let start_video = Instant::now();
    start_window(&mut device.ui, rdram_ptr, rdram_size, fullscreen);
    let duration_video = start_video.elapsed();
    info!("Video init took: {:?}", duration_video);

    set_cartridge_region(device);

    let start_memory = Instant::now();
    initialise_memory_map(device);
    let duration_memory = start_memory.elapsed();
    info!("Memory map init took: {:?}", duration_memory);

    start_rsp(device);
    set_cpu_clock_speed(device);
    start_cpu(device);

    let start_load_saves = Instant::now();
    load_saves(&mut device.ui);
    let duration_load_saves = start_load_saves.elapsed();
    info!("Load saves took: {:?}", duration_load_saves);

    let start_load_rom_save = Instant::now();
    load_rom_save(device);
    let duration_load_rom_save = start_load_rom_save.elapsed();
    info!("Load ROM save took: {:?}", duration_load_rom_save);

    cpu::run(device);
}

