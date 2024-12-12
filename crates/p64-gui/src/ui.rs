use std::fs::create_dir_all;
use std::fs::read;

use crate::input;
use crate::config;
use crate::storage;
use crate::storage::write_save;
use crate::storage::SaveTypes;
use dirs::data_dir;
use serde_json;
use serde_json::from_slice;

pub struct Ui {
    pub config: Config,
    pub game_info: GameInfo,
    pub input_controllers: InputControllers,
    pub save_data: SaveData,
    pub sdl_context: SdlContext,
    pub window_audio: WindowAudio,
}

pub struct Config {
    pub file_path: std::path::PathBuf,
    pub config: config::Config,
}

pub struct GameInfo {
    pub id: String,
    pub hash: String,
}

pub struct InputControllers {
    pub controllers: [input::Controllers; 4],
    pub joystick_subsystem: Option<sdl2::JoystickSubsystem>,
    pub controller_subsystem: Option<sdl2::GameControllerSubsystem>,
}

pub struct SaveData {
    pub save_type: Vec<storage::SaveTypes>,
    pub data: storage::SaveData,
}

pub struct SdlContext {
    pub sdl_context: Option<sdl2::Sdl>,
    pub video_subsystem: Option<sdl2::VideoSubsystem>,
    pub audio_subsystem: Option<sdl2::AudioSubsystem>,
}

pub struct WindowAudio {
    pub window: Option<sdl2::video::Window>,
    pub audio_device: Option<sdl2::audio::AudioQueue<i16>>,
}

impl Default for Ui {
    fn default() -> Self {
        let sdl_context = sdl2::init().unwrap();
        let video_subsystem = sdl_context.video().unwrap();
        let audio_subsystem = sdl_context.audio().unwrap();
        let joystick_subsystem = sdl_context.joystick().unwrap();
        let controller_subsystem = sdl_context.game_controller().unwrap();
    
        let file_path = dirs::config_dir()
            .unwrap()
            .join("p64")
            .join("config.json");
        let config_file = std::fs::read(file_path.clone());
        let mut config_map = config::Config::new();
        if config_file.is_ok() {
            let result = serde_json::from_slice(config_file.unwrap().as_ref());
            if result.is_ok() {
                config_map = result.unwrap();
            }
        }
    
        let mut ui = Ui {
            config: Config {
                file_path,
                config: config_map,
            },
            game_info: GameInfo {
                id: String::new(),
                hash: String::new(),
            },
            input_controllers: InputControllers {
                controllers: [
                    input::Controllers::default(),
                    input::Controllers::default(),
                    input::Controllers::default(),
                    input::Controllers::default(),
                ],
                joystick_subsystem: Some(joystick_subsystem),
                controller_subsystem: Some(controller_subsystem),
            },
            save_data: SaveData {
                save_type: vec![],
                data: storage::SaveData::default(),
            },
            sdl_context: SdlContext {
                sdl_context: Some(sdl_context),
                video_subsystem: Some(video_subsystem),
                audio_subsystem: Some(audio_subsystem),
            },
            window_audio: WindowAudio {
                window: None,
                audio_device: None,
            },
        };
    
        let desired_spec = sdl2::audio::AudioSpecDesired {
            freq: Some(44100),
            channels: Some(2),
            samples: None,
        };
        ui.window_audio.audio_device = Some(
            ui.sdl_context.audio_subsystem.as_mut().unwrap().open_queue::<i16, _>(None, &desired_spec).unwrap(),
        );
        ui.window_audio.audio_device.as_mut().unwrap().resume();
    
        let joystick_subsystem = ui.input_controllers.joystick_subsystem.as_ref().unwrap();
        let mut taken = [false; 4];
        for i in 0..4 {
            let controller_assignment = &ui.config.config.input.controller_assignment[i];
            if controller_assignment.is_some() {
                let mut joystick_index = u32::MAX;
                let guid = controller_assignment.clone().unwrap();
                for i in 0..joystick_subsystem.num_joysticks().unwrap() {
                    if joystick_subsystem.device_guid(i).unwrap().to_string() == guid
                        && !taken[i as usize]
                    {
                        joystick_index = i;
                        taken[i as usize] = true;
                        break;
                    }
                }
                if joystick_index < u32::MAX {
                    if ui.config.config.input.input_profile_binding[i] == "Default" {
                        let controller_result = ui
                            .sdl_context
                            .sdl_context
                            .as_ref()
                            .unwrap()
                            .game_controller()
                            .unwrap()
                            .open(joystick_index);
                        if controller_result.is_ok() {
                            ui.input_controllers.controllers[i].controller = Some(controller_result.unwrap());
                        }
                    }
                    if ui.input_controllers.controllers[i].controller.is_none() {
                        let joystick_result = ui
                            .sdl_context
                            .sdl_context
                            .as_ref()
                            .unwrap()
                            .joystick()
                            .unwrap()
                            .open(joystick_index);
                        if joystick_result.is_err() {
                            println!("could not connect joystick")
                        } else {
                            ui.input_controllers.controllers[i].joystick = Some(joystick_result.unwrap());
                        }
                    }
                } else {
                    println!("Could not bind assigned controller");
                }
            }
        }
    
        let id = ui.game_info.id.as_str();
        ui.save_data.save_type = get_save_type(id);
    
        let base_path = data_dir().unwrap().join("p64/saves");
        create_dir_all(&base_path).expect("could not create save dir");
    
        let file_names = ["eep", "sra", "fla", "mpk", "romsave"];
        let mut paths = [
            &mut ui.save_data.data.eeprom_path,
            &mut ui.save_data.data.sram_path,
            &mut ui.save_data.data.flash_path,
            &mut ui.save_data.data.mempak_path,
            &mut ui.save_data.data.romsave_path,
        ];
    
        for (path, &name) in paths.iter_mut().zip(&file_names) {
            path.clone_from(&base_path);
            path.push(format!("{}-{}.{}", ui.game_info.id, ui.game_info.hash, name));
        }
    
        ui
    }
}

pub fn get_save_type(game_id: &str) -> Vec<SaveTypes> {
    const EEPROM4K_GAMES: [&str; 45] = [
        "NAB", "NAD", "NAL", "NBC", "NCG", "NCT", "NDU", "NFW", "NGE", "NGL", 
        "NHS", "NHY", "NIJ", "NKG", "NKT", "NLR", "NM9", "NMF", "NML", "NMR", 
        "NMW", "NMZ", "NOS", "NP2", "NPL", "NPM", "NPW", "NRA", "NRC", "NSI", 
        "NSL", "NSM", "NSU", "NSV", "NTJ", "NTM", "NTN", "NTP", "NTR", "NTX", 
        "NVL", "NVP", "NVY", "NWL", "NWR"
    ];

    const EEPROM16K_GAMES: [&str; 35] = [
        "NB7", "NGT", "NFU", "NCW", "NCZ", "ND6", "NDO", "ND2", "N3D", "NMX",
        "NGC", "NIM", "NNB", "NMV", "NM8", "NEV", "NPP", "NUB", "NPD", "NRZ",
        "NR7", "NEP", "NYS", "NAK", "NBK", "NDY", "NGU", "NJG", "NRP", "NWQ",
        "NYW", "NB5", "NBM", "NCR", "NDR"
    ];
    
    const FLASH_GAMES: [&str; 34] = [
        "NCC", "NDA", "NAF", "NJF", "NKJ", "NZS", "NM6", "NCK", "NMQ", "NPN",
        "NPF", "NPO", "CP2", "NP3", "NRH", "NSQ", "NT9", "NW4", "NDP", "NLB",
        "NMS", "NMT", "NRC", "NSM", "NSS", "NTC", "NTD", "NTE", "NTF", "NTG",
        "NTH", "NTI", "NTJ", "NTK"
    ];

    match game_id {
        id if EEPROM4K_GAMES.contains(&id) => vec![SaveTypes::Eeprom4k],
        id if EEPROM16K_GAMES.contains(&id) => vec![SaveTypes::Eeprom16k],
        id if FLASH_GAMES.contains(&id) => vec![SaveTypes::Flash],
        "NPQ" => vec![],
        _ => vec![SaveTypes::Eeprom4k, SaveTypes::Sram],
    }
}


pub fn load_saves(ui: &mut Ui) {
    let mut paths = [
        (&mut ui.save_data.data.eeprom_path, &mut ui.save_data.data.eeprom.0),
        (&mut ui.save_data.data.sram_path, &mut ui.save_data.data.sram.0),
        (&mut ui.save_data.data.flash_path, &mut ui.save_data.data.flash.0),
        (&mut ui.save_data.data.mempak_path, &mut ui.save_data.data.mempak.0),
    ];

    for (path, save) in paths.iter_mut() {
        if let Ok(data) = read(path) { **save = data; }
    }

    if let Ok(data) = read(&ui.save_data.data.romsave_path) { ui.save_data.data.romsave.0 = from_slice(&data).unwrap(); }
}

impl Drop for Ui {
    fn drop(&mut self) {
        write_saves(self);
        write_config(self);
    }
}

pub fn write_saves(ui: &Ui) {
    [(SaveTypes::Eeprom16k, ui.save_data.data.eeprom.1),
     (SaveTypes::Sram, ui.save_data.data.sram.1),
     (SaveTypes::Flash, ui.save_data.data.flash.1),
     (SaveTypes::Mempak, ui.save_data.data.mempak.1),
     (SaveTypes::Romsave, ui.save_data.data.romsave.1)]
        .iter()
        .filter(|&&(_, flag)| flag)
        .for_each(|&(save_type, _)| write_save(ui, save_type));
}

fn write_config(ui: &Ui) {
    let f = std::fs::File::create(ui.config.file_path.clone()).unwrap();
    serde_json::to_writer_pretty(f, &ui.config.config).unwrap();
}

impl Ui {
    pub fn new() -> Ui {
        Ui::default()
    }
}
