use crate::ui;

pub fn set_frequency(ui: &mut ui::Ui, frequency: u64) {
    let desired_spec = sdl2::audio::AudioSpecDesired {
        freq: Some(frequency as i32),
        channels: Some(2),
        samples: None,
    };
    ui.window_audio.audio_device = Some(
        ui.sdl_context.audio_subsystem.as_mut().unwrap().open_queue::<i16, _>(None, &desired_spec).unwrap(),
    );
    ui.window_audio.audio_device.as_mut().unwrap().resume();
}