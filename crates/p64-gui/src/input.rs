use std::time::{Duration, Instant};

use sdl2::controller::{Axis, Button, GameController};
use sdl2::event::WindowEvent;
use sdl2::joystick::{HatState, Joystick};
use sdl2::keyboard::{KeyboardState, Scancode};
use sdl2::{event::Event, pixels::Color};
use sdl2::rect::Point;
use sdl2::render::Canvas;
use sdl2::video::Window;
use rusttype::{Font, Scale, point};
use tracing::error;


use crate::ui;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Input {
    RDPad,
    LDPad,
    DDPad,
    UDPad,
    StartButton,
    ZTrig,
    BButton,
    AButton,
    RCButton,
    LCButton,
    DCButton,
    UCButton,
    RTrig,
    LTrig,
    XAxis,
    YAxis,
    AxisLeft,
    AxisRight,
    AxisUp,
    AxisDown,
}

impl Input {
    pub fn as_usize(self) -> usize {
        match self {
            Input::RDPad => 0,
            Input::LDPad => 1,
            Input::DDPad => 2,
            Input::UDPad => 3,
            Input::StartButton => 4,
            Input::ZTrig => 5,
            Input::BButton => 6,
            Input::AButton => 7,
            Input::RCButton => 8,
            Input::LCButton => 9,
            Input::DCButton => 10,
            Input::UCButton => 11,
            Input::RTrig => 12,
            Input::LTrig => 13,
            Input::XAxis => 16,
            Input::YAxis => 24,
            Input::AxisLeft => 14,
            Input::AxisRight => 15,
            Input::AxisUp => 16,
            Input::AxisDown => 17,
        }
    }
}

pub const MAX_AXIS_VALUE: f64 = 85.0;

#[derive(Default)]
pub struct Controllers {
    pub controller: Option<GameController>,
    pub joystick: Option<Joystick>,
}

fn bind_axis(x: f64, y: f64) -> (f64, f64) {
    const RADIUS: f64 = 95.0;
    if x.hypot(y) > RADIUS {
        let scale_factor = RADIUS / x.hypot(y);
        (x * scale_factor, y * scale_factor)
    } else {
        (x, y)
    }
}

pub fn set_joystick_axis(profile: &crate::config::InputProfile, joystick: &sdl2::joystick::Joystick) -> (f64, f64) {
    fn get_axis_value(profile: &crate::config::InputProfile, joystick: &sdl2::joystick::Joystick, axis: usize, invert: bool) -> f64 {
        if profile.joystick_axis[axis].0 {
            let axis_position = joystick.axis(profile.joystick_axis[axis].1).unwrap();
            if axis_position as isize * profile.joystick_axis[axis].2 as isize > 0 {
                let value = axis_position as f64 * MAX_AXIS_VALUE / i16::MAX as f64;
                return if invert { -value } else { value };
            }
        }
        0.0
    }

    let x = get_axis_value(profile, joystick, Input::AxisLeft.as_usize(), false) + get_axis_value(profile, joystick, Input::AxisRight.as_usize(), false);
    let y = get_axis_value(profile, joystick, Input::AxisDown.as_usize(), true) + get_axis_value(profile, joystick, Input::AxisUp.as_usize(), true);

    (x, y)
}

pub fn set_controller_axis(profile: &crate::config::InputProfile, controller: &GameController) -> (f64, f64) {
    fn get_axis_value(profile: &crate::config::InputProfile, controller: &GameController, axis: usize, invert: bool) -> f64 {
        if profile.controller_axis[axis].0 {
            let axis_enum = match profile.controller_axis[axis].1 {
                0 => Axis::LeftX,
                1 => Axis::LeftY,
                2 => Axis::RightX,
                3 => Axis::RightY,
                _ => return 0.0,
            };
            let axis_position = controller.axis(axis_enum);
            if axis_position as isize * profile.controller_axis[axis].2 as isize > 0 {
                let value = axis_position as f64 * MAX_AXIS_VALUE / i16::MAX as f64;
                return if invert { -value } else { value };
            }
        }
        0.0
    }

    let x = get_axis_value(profile, controller, Input::AxisLeft.as_usize(), false)
        + get_axis_value(profile, controller, Input::AxisRight.as_usize(), false);
    let y = get_axis_value(profile, controller, Input::AxisDown.as_usize(), true)
        + get_axis_value(profile, controller, Input::AxisUp.as_usize(), true);
    (x, y)
}

pub fn set_axis_from_keys(profile: &crate::config::InputProfile, keyboard_state: &KeyboardState) -> (f64, f64) {
    fn get_key_value(profile: &crate::config::InputProfile, keyboard_state: &KeyboardState, axis: usize, value: f64) -> f64 {
        if profile.keys[axis].0 {
            if let Some(scancode) = Scancode::from_i32(profile.keys[axis].1) {
                if keyboard_state.is_scancode_pressed(scancode) {
                    return value;
                }
            }
        }
        0.0
    }

    let x = get_key_value(profile, keyboard_state, Input::AxisLeft.as_usize(), -MAX_AXIS_VALUE)
        + get_key_value(profile, keyboard_state, Input::AxisRight.as_usize(), MAX_AXIS_VALUE);
    let y = get_key_value(profile, keyboard_state, Input::AxisDown.as_usize(), -MAX_AXIS_VALUE)
        + get_key_value(profile, keyboard_state, Input::AxisUp.as_usize(), MAX_AXIS_VALUE);
    (x, y)
}

pub fn set_joystick_buttons(
    profile: &crate::config::InputProfile,
    i: usize,
    joystick: &sdl2::joystick::Joystick,
    keys: &mut u32,
) {
    if profile.joystick_buttons[i].0 {
        if let Ok(button) = joystick.button(profile.joystick_buttons[i].1) {
            if button {
                *keys |= 1 << i;
            }
        }
    }

    if profile.joystick_hat[i].0 {
        if let Ok(hat_state) = joystick.hat(profile.joystick_hat[i].1) {
            let expected_state = match profile.joystick_hat[i].2 {
                0 => sdl2::joystick::HatState::Centered,
                1 => sdl2::joystick::HatState::Up,
                2 => sdl2::joystick::HatState::Right,
                4 => sdl2::joystick::HatState::Down,
                8 => sdl2::joystick::HatState::Left,
                _ => return,
            };
            if hat_state == expected_state {
                *keys |= 1 << i;
            }
        }
    }

    if profile.joystick_axis[i].0 {
        if let Ok(axis_position) = joystick.axis(profile.joystick_axis[i].1) {
            if axis_position as isize * profile.joystick_axis[i].2 as isize > 0 && axis_position.saturating_abs() > i16::MAX / 2
            {*keys |= 1 << i;}
        }
    }
}


pub fn set_controller_buttons(
    profile: &crate::config::InputProfile,
    i: usize,
    controller: &GameController,
    keys: &mut u32,
) {
    let profile_controller_button = profile.controller_buttons[i];
    if profile_controller_button.0 {
        unsafe {
            *keys |= (controller.button(std::mem::transmute::<i32, Button>(
                profile_controller_button.1,
            )) as u32)
                << i;
        }
    }

    let profile_controller_axis = profile.controller_axis[i];
    if profile_controller_axis.0 {
        let axis_position = unsafe {
            controller.axis(std::mem::transmute::<i32, Axis>(
                profile_controller_axis.1,
            ))
        };
        if axis_position as isize * profile_controller_axis.2 as isize > 0
            && axis_position.saturating_abs() > i16::MAX / 2
        {
            *keys |= 1 << i;
        }
    }
}

pub fn get(ui: &mut ui::Ui, channel: usize) -> u32 {
    let context = match ui.sdl_context.sdl_context.as_mut() {
        Some(ctx) => ctx,
        None => return 0,
    };
    let events = match context.event_pump() {
        Ok(ev) => ev,
        Err(_) => return 0,
    };
    let keyboard_state = events.keyboard_state();

    let profile_name = ui.config.config.input.input_profile_binding[channel].clone();
    let profile = match ui.config.config.input.input_profiles.get(&profile_name) {
        Some(p) => p,
        None => return 0,
    };

    let mut keys = 0;
    let controller = &ui.input_controllers.controllers[channel].controller;
    let joystick = &ui.input_controllers.controllers[channel].joystick;

    for i in 0..14 {
        if profile_name != "Default" || channel == 0 {
            let profile_key = profile.keys[i];
            if profile_key.0 {
                if let Some(scancode) = Scancode::from_i32(profile_key.1) {
                    if keyboard_state.is_scancode_pressed(scancode) {
                        keys |= 1 << i;
                    }
                }
            }
        }

        if let Some(joystick) = joystick {
            set_joystick_buttons(profile, i, joystick, &mut keys);
        } else if let Some(controller) = controller {
            set_controller_buttons(profile, i, controller, &mut keys);
        }
    }

    let (mut x, mut y) = if profile_name != "Default" || channel == 0 {
        set_axis_from_keys(profile, &keyboard_state)
    } else {
        (0.0, 0.0)
    };

    if let Some(joystick) = joystick {
        (x, y) = set_joystick_axis(profile, joystick);
    } else if let Some(controller) = controller {
        (x, y) = set_controller_axis(profile, controller);
    }

    let (bound_x, bound_y) = bind_axis(x, y);

    keys |= (bound_x.round() as i8 as u8 as u32) << Input::XAxis.as_usize();
    keys |= (bound_y.round() as i8 as u8 as u32) << Input::YAxis.as_usize();
    keys
}

pub fn list_controllers(ui: &ui::Ui) {
    let joystick_subsystem = match ui.input_controllers.joystick_subsystem.as_ref() {
        Some(subsystem) => subsystem,
        None => {
            error!("Joystick subsystem is not available");
            return;
        }
    };

    let num_joysticks = match joystick_subsystem.num_joysticks() {
        Ok(num) => num,
        Err(e) => {
            error!("Failed to get number of joysticks: {}", e);
            return;
        }
    };

    if num_joysticks == 0 {
        println!("No controllers connected");
        return;
    }

    for i in 0..num_joysticks {
        match joystick_subsystem.name_for_index(i) {
            Ok(name) => println!("{}: {}", i, name),
            Err(e) => error!("Failed to get name for joystick {}: {}", i, e),
        }
    }
}

pub fn assign_controller(ui: &mut ui::Ui, controller: u32, port: usize) {
    let joystick_subsystem = ui.input_controllers.joystick_subsystem.as_ref().unwrap();
    let num_joysticks = joystick_subsystem.num_joysticks().unwrap();
    if controller < num_joysticks {
        match joystick_subsystem.device_guid(controller) {
            Ok(guid) => ui.config.config.input.controller_assignment[port - 1] = Some(guid.to_string()),
            Err(e) => error!("Failed to get device GUID for controller {}: {}", controller, e),
        }
    } else {
        error!("Invalid controller number: {}", controller);
    }
}

pub fn bind_input_profile(ui: &mut ui::Ui, profile: String, port: usize) {
    if ui.config.config.input.input_profiles.contains_key(&profile) {
        ui.config.config.input.input_profile_binding[port - 1] = profile;
    } else {
        error!("Invalid profile name: {}", profile);
    }
}

pub fn clear_bindings(ui: &mut ui::Ui) {
    for i in 0..4 {
        ui.config.config.input.controller_assignment[i] = None;
        ui.config.config.input.input_profile_binding[i] = "Default".to_string();
    }
}

pub fn draw_text(text: &str, canvas: &mut Canvas<Window>, font: &Font) {
    let scale = Scale::uniform(32.0);
    let v_metrics = font.v_metrics(scale);
    
    // Calculate text dimensions
    let text_width = font
        .layout(text, scale, point(0.0, 0.0))
        .map(|g| g.position().x + g.unpositioned().h_metrics().advance_width)
        .last()
        .unwrap_or(0.0);
    
    let text_height = v_metrics.ascent - v_metrics.descent;
    
    // Calculate center position (960x720 screen)
    let x = (960.0 - text_width) / 3.0;
    let y = (720.0 - text_height) / 3.0;
    
    let offset = point(x, y + v_metrics.ascent);

    // Clear screen
    canvas.set_draw_color(Color::RGB(0, 0, 0));
    canvas.clear();

    // Draw centered text
    font.layout(text, scale, offset).for_each(|glyph| {
        if let Some(bb) = glyph.pixel_bounding_box() {
            glyph.draw(|x, y, v| {
                if v > 0.5 {
                    canvas.set_draw_color(Color::RGB(255, 255, 255));
                    canvas.draw_point(Point::new(x as i32 + bb.min.x, y as i32 + bb.min.y)).unwrap();
                }
            });
        }
    });

    canvas.present();
}

macro_rules! handle_event {
    ($event:expr, $value:expr, $new_keys:expr, $new_joystick_buttons:expr, $new_joystick_hat:expr, $new_joystick_axis:expr, $last_axis_result:expr, $key_set:expr, $last_input_time:expr) => {
        match $event {
            Event::Window { win_event: WindowEvent::Close, .. } => return,
            Event::KeyUp { .. } => continue,
            Event::JoyButtonUp { .. } => continue,
            Event::KeyDown { scancode: Some(scancode), repeat, .. } => {
                if !repeat {
                    $new_keys[$value.as_usize()] = (true, scancode as i32);
                    $key_set = true;
                    $last_input_time = Instant::now();
                }
            }
            Event::JoyButtonDown { button_idx, .. } => {
                $new_joystick_buttons[$value.as_usize()] = (true, button_idx as u32);
                $key_set = true;
                $last_input_time = Instant::now();
            }
            Event::JoyHatMotion { hat_idx, state, .. } => {
                if state != HatState::Centered {
                    $new_joystick_hat[$value.as_usize()] = (true, hat_idx as u32, state as i8);
                    $key_set = true;
                    $last_input_time = Instant::now();
                }
            }
            Event::JoyAxisMotion { axis_idx, value: axis_value, .. } => {
                if axis_value.saturating_abs() > 24576 {
                    let result = (true, axis_idx as u32, axis_value / axis_value.saturating_abs());
                    if result != $last_axis_result {
                        $new_joystick_axis[$value.as_usize()] = result;
                        $last_axis_result = result;
                        $key_set = true;
                        $last_input_time = Instant::now();
                    }
                }
            }
            _ => {}
        }
    };
}

pub fn configure_profile(ui: &mut ui::Ui, profile: String) {
    if profile == "Default" || profile.is_empty() {
        println!("Profile name cannot be {}", if profile.is_empty() { "empty" } else { "Default" });
        return;
    }

    let joystick_subsystem = ui.sdl_context.sdl_context.as_ref().unwrap().joystick().unwrap();
    let _joysticks: Vec<_> = (0..joystick_subsystem.num_joysticks().unwrap())
        .map(|i| joystick_subsystem.open(i))
        .collect();

    let window = ui.sdl_context.video_subsystem.as_ref().unwrap()
        .window("configure input profile", 640, 480)
        .position_centered()
        .build()
        .unwrap();
    let mut canvas = window.into_canvas().build().unwrap();

    let key_labels = [
        ("A", Input::AButton), ("B", Input::BButton), ("Start", Input::StartButton), ("D Up", Input::UDPad),
        ("D Down", Input::DDPad), ("D Left", Input::LDPad), ("D Right", Input::RDPad), ("C Up", Input::UCButton),
        ("C Down", Input::DCButton), ("C Left", Input::LCButton), ("C Right", Input::RCButton), ("L", Input::LTrig),
        ("R", Input::RTrig), ("Z", Input::ZTrig), ("Control Stick Up", Input::AxisUp), ("Control Stick Down", Input::AxisDown),
        ("Control Stick Left", Input::AxisLeft), ("Control Stick Right", Input::AxisRight),
    ];

    let mut new_keys = [(false, 0); 18];
    let mut new_joystick_buttons = [(false, 0u32); 14];
    let mut new_joystick_hat = [(false, 0u32, 0); 14];
    let mut new_joystick_axis = [(false, 0u32, 0); 18];

    let mut last_axis_result = (false, 0, 0);
    let mut event_pump = ui.sdl_context.sdl_context.as_ref().unwrap().event_pump().unwrap();

    for (key, value) in key_labels.iter() {
        event_pump.poll_iter().for_each(|_| {}); // clear events
    
        let roboto = Font::try_from_bytes(include_bytes!("Roboto-Regular.ttf")).unwrap();
        draw_text(&format!("Press the button to bind to: {key}"), &mut canvas, &roboto);
    
        let mut key_set = false;
        let mut last_input_time = Instant::now();
        let cooldown = Duration::from_millis(300);
    
        while !key_set {
            std::thread::sleep(std::time::Duration::from_millis(100));
            
            if last_input_time.elapsed() >= cooldown {
                for event in event_pump.poll_iter() {
                    handle_event!(event, value, new_keys, new_joystick_buttons, new_joystick_hat, new_joystick_axis, last_axis_result, key_set, last_input_time);
                }
            }
        }
    }

    let new_profile = crate::config::InputProfile {
        keys: new_keys,
        controller_buttons: Default::default(),
        controller_axis: Default::default(),
        joystick_buttons: new_joystick_buttons,
        joystick_hat: new_joystick_hat,
        joystick_axis: new_joystick_axis,
    };
    ui.config.config.input.input_profiles.insert(profile, new_profile);
}

fn set_keys(default_keys: &mut [(bool, i32); 18], mappings: &[(usize, Scancode)]) {
    for &(index, scancode) in mappings {
        default_keys[index] = (true, scancode as i32);
    }
}

fn set_controller_buttons_initially(default_buttons: &mut [(bool, i32); 14], mappings: &[(usize, Button)]) {
    for &(index, button) in mappings {
        default_buttons[index] = (true, button as i32);
    }
}

fn set_controller_axis_initially(default_axis: &mut [(bool, i32, i16); 18], mappings: &[(usize, Axis, i16)]) {
    for &(index, axis, direction) in mappings {
        default_axis[index] = (true, axis as i32, direction);
    }
}

pub fn get_default_profile() -> crate::config::InputProfile {

    let mut default_keys = [(false, 0); 18];
    set_keys(&mut default_keys, &[
        (Input::RDPad.as_usize(), Scancode::D),
        (Input::LDPad.as_usize(), Scancode::A),
        (Input::DDPad.as_usize(), Scancode::S),
        (Input::UDPad.as_usize(), Scancode::W),
        (Input::StartButton.as_usize(), Scancode::Return),
        (Input::ZTrig.as_usize(), Scancode::Z),
        (Input::BButton.as_usize(), Scancode::LCtrl),
        (Input::AButton.as_usize(), Scancode::LShift),
        (Input::RCButton.as_usize(), Scancode::L),
        (Input::LCButton.as_usize(), Scancode::J),
        (Input::DCButton.as_usize(), Scancode::K),
        (Input::UCButton.as_usize(), Scancode::I),
        (Input::RTrig.as_usize(), Scancode::C),
        (Input::LTrig.as_usize(), Scancode::X),
        (Input::AxisLeft.as_usize(), Scancode::Left),
        (Input::AxisRight.as_usize(), Scancode::Right),
        (Input::AxisUp.as_usize(), Scancode::Up),
        (Input::AxisDown.as_usize(), Scancode::Down),
    ]);
    
    let mut default_controller_buttons = [(false, 0); 14];
    set_controller_buttons_initially(&mut default_controller_buttons, &[
        (Input::RDPad.as_usize(), Button::DPadRight),
        (Input::LDPad.as_usize(), Button::DPadLeft),
        (Input::DDPad.as_usize(), Button::DPadDown),
        (Input::UDPad.as_usize(), Button::DPadUp),
        (Input::StartButton.as_usize(), Button::Start),
        (Input::BButton.as_usize(), Button::X),
        (Input::AButton.as_usize(), Button::A),
        (Input::RTrig.as_usize(), Button::RightShoulder),
        (Input::LTrig.as_usize(), Button::LeftShoulder),
    ]);
    
    let mut default_controller_axis = [(false, 0, 0); 18];
    set_controller_axis_initially(&mut default_controller_axis, &[
        (Input::ZTrig.as_usize(), Axis::TriggerLeft, 1),
        (Input::RCButton.as_usize(), Axis::RightX, 1),
        (Input::LCButton.as_usize(), Axis::RightX, -1),
        (Input::DCButton.as_usize(), Axis::RightY, 1),
        (Input::UCButton.as_usize(), Axis::RightY, -1),
        (Input::AxisLeft.as_usize(), Axis::LeftX, -1),
        (Input::AxisRight.as_usize(), Axis::LeftX, 1),
        (Input::AxisUp.as_usize(), Axis::LeftY, -1),
        (Input::AxisDown.as_usize(), Axis::LeftY, 1),
    ]);

    crate::config::InputProfile {
        keys: default_keys,
        controller_buttons: default_controller_buttons,
        controller_axis: default_controller_axis,
        joystick_buttons: Default::default(),
        joystick_hat: Default::default(),
        joystick_axis: Default::default(),
    }
}
