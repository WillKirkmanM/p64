use crate::ui;

extern "C" {
    pub fn vk_init(rdram_ptr: usize, rdram_size: u32, fullscreen: u8);
    pub fn set_sdl_window(window: usize);
    pub fn rdp_update_screen() -> u8;
    pub fn rdp_set_vi_register(reg: u32, value: u32);
    pub fn rdp_process_commands(dpc_regs: &mut [u32; 8], SP_DMEM: *mut u8) -> u64;
    pub fn full_sync();
    pub fn set_upscaling_1x();
    pub fn set_upscaling_2x();
    pub fn set_upscaling_4x();
    pub fn set_upscaling_8x();
}

#[repr(C)]
pub struct ViewportData {
    pub x: f32,
    pub y: f32,
    pub width: f32,
    pub height: f32,
}

extern "C" {
    fn calculate_viewport_values(width: i32, height: i32) -> ViewportData;
}

// Usage
pub fn set_viewport(width: i32, height: i32) -> ViewportData {
    unsafe { calculate_viewport_values(width, height) }
}

pub fn start_window(ui: &mut ui::Ui, rdram_ptr: *mut u8, rdram_size: usize, width: u32, height: u32, fullscreen: bool) {

     let mut builder = ui
            .sdl_context
            .video_subsystem
            .as_ref()
            .unwrap()
            .window("p64", width, height);
        builder.vulkan();
        builder.position_centered().vulkan().resizable();

        if fullscreen {
            builder.fullscreen_desktop();
        }

        ui.window_audio.window = Some(builder.build().unwrap());
    
    unsafe {
        set_sdl_window(ui.window_audio.window.as_mut().unwrap().raw() as usize);
        vk_init(rdram_ptr as usize, rdram_size as u32, fullscreen as u8);
    }
}

pub fn update_screen() -> u8 { unsafe { rdp_update_screen() } }
pub fn set_register(reg: u32, value: u32) { unsafe { rdp_set_vi_register(reg, value); } }
pub fn process_rdp_list(dpc_regs: &mut [u32; 8], sp_dmem: &mut [u8]) -> u64 { unsafe { rdp_process_commands(dpc_regs, sp_dmem.as_mut_ptr()) } }
pub fn rdp_full_sync() { unsafe { full_sync(); } }

pub fn enable_1x_upscaling() { unsafe { set_upscaling_1x(); } }
pub fn enable_2x_upscaling() { unsafe { set_upscaling_2x(); } }
pub fn enable_4x_upscaling() { unsafe { set_upscaling_4x(); } }
pub fn enable_8x_upscaling() { unsafe { set_upscaling_8x() } }

