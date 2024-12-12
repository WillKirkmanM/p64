use p64_gui::ui::Ui;

pub fn get_input_profiles(game_ui: &Ui) -> Vec<String> {
    let mut profiles = vec![];
    for key in game_ui.config.config.input.input_profiles.keys() {
        profiles.push((*key).clone())
    }
    profiles
}

pub fn get_controllers(game_ui: &Ui) -> Vec<String> {
    let mut controllers: Vec<String> = vec![];

    let joystick_subsystem = game_ui.input_controllers.joystick_subsystem.as_ref().unwrap();
    let num_joysticks = joystick_subsystem.num_joysticks().unwrap();
    for i in 0..num_joysticks {
        controllers.push(joystick_subsystem.name_for_index(i).unwrap());
    }
    controllers
}