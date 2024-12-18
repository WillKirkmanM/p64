use dirs::ensure_directories;
use eframe::{egui::ViewportBuilder, run_native, NativeOptions};
use gui::P64Gui;
use p64_gui::video::enable_4x_upscaling;

pub mod rom;
pub mod audio;
pub mod gui;
pub mod controllers;
pub mod config;
pub mod dirs;

pub fn main() {
    tracing_subscriber::fmt::init();

    ensure_directories();
    enable_4x_upscaling();

    run_native(
        "p64",
        NativeOptions {
            viewport: ViewportBuilder::default().with_inner_size([960.0, 720.0]),
            ..Default::default()
        },
        Box::new(|_cc| Ok(Box::new(P64Gui::new()))),
    ).unwrap();
}