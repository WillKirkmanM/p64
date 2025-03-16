use std::path::PathBuf;

use p64_gui::ui::Ui;
use serde::{Deserialize, Serialize};

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct Config {
    pub rom_folder: Option<PathBuf>,
    pub settings: Option<Settings>,
    pub controller_settings: Option<ControllerSettings>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Settings {
    pub graphics: GraphicsSettings,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GraphicsSettings {
    pub width: u32,
    pub height: u32,
    pub fullscreen: bool,
    pub vsync: bool,
    pub upscaling: u8,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            graphics: GraphicsSettings::default(),
        }
    }
}

impl Default for GraphicsSettings {
    fn default() -> Self {
        Self {
            width: 1280,
            height: 720,
            fullscreen: false,
            vsync: true,
            upscaling: 1,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControllerSettings {
    pub selected_controller: [i32; 4],
    pub selected_profile: [String; 4],
}

impl Default for ControllerSettings {
    fn default() -> Self {
        Self {
            selected_controller: [-1; 4],
            selected_profile: ["Default".to_string(), "Default".to_string(), "Default".to_string(), "Default".to_string()],
        }
    }
}

pub fn save_config(game_ui: &mut Ui, selected_controller: [i32; 4], selected_profile: [String; 4]) {
    game_ui
        .config
        .config
        .input
        .controller_assignment
        .iter_mut()
        .zip(selected_controller.iter())
        .for_each(|(assign, &item)| {
            *assign = (item != -1).then(|| {
                game_ui
                    .input_controllers
                    .joystick_subsystem
                    .as_ref()
                    .unwrap()
                    .device_guid(item as u32)
                    .unwrap()
                    .to_string()
            })
        });

    game_ui.config.config.input.input_profile_binding = selected_profile;
}