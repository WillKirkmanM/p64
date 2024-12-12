use std::path::PathBuf;

use p64_gui::ui::Ui;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Default)]
pub struct Config {
    pub rom_folder: Option<PathBuf>,
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