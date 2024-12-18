use crate::config::Config;
use crate::{
    config::save_config,
    controllers::{get_controllers, get_input_profiles},
    rom::RomInfo,
};
use dirs::config_dir;
use eframe::egui::{self, menu, Button, ComboBox, Grid, RichText, ScrollArea, Window};
use p64_core::{run_game, N64};
use p64_gui::input::configure_profile;
use p64_gui::ui::Ui;
use p64_gui::video::{enable_1x_upscaling, enable_2x_upscaling, enable_4x_upscaling, enable_8x_upscaling, set_viewport};
use std::{collections::HashMap, fs, path::PathBuf};
use std::{
    fs::{read_dir, remove_file, File},
    sync::mpsc,
};

const VALID_ROM_EXTENSIONS: [&str; 3] = ["z64", "n64", "v64"];

pub struct P64Gui {
    configure_profile: bool,
    profile_name: String,
    controllers: Vec<String>,
    selected_controller: [i32; 4],
    selected_profile: [String; 4],
    input_profiles: Vec<String>,
    roms: Vec<PathBuf>,
    rom_folder: Option<PathBuf>,
    rom_info_cache: HashMap<PathBuf, RomInfo>,
    settings_open: bool,
    settings: Settings
}

#[derive(Default)]
pub struct Settings {
    pub graphics: GraphicsSettings,
}

#[derive(Debug)]
pub struct GraphicsSettings {
    pub width: u32,
    pub height: u32,
    pub fullscreen: bool,
    pub vsync: bool,
    pub upscaling: u8,
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

fn execute<F: std::future::Future<Output = ()> + Send + 'static>(f: F) {
    std::thread::spawn(move || futures::executor::block_on(f));
}

impl P64Gui {
    pub fn new() -> Self {
        let game_ui = Ui::new();
        let joystick_subsystem = game_ui
            .input_controllers
            .joystick_subsystem
            .as_ref()
            .unwrap();
        let guids: Vec<String> = (0..joystick_subsystem.num_joysticks().unwrap())
            .map(|i| joystick_subsystem.device_guid(i).unwrap().to_string())
            .collect();

        let selected_controller = game_ui
            .config
            .config
            .input
            .controller_assignment
            .iter()
            .enumerate()
            .map(|(_pos, item)| {
                item.as_ref()
                    .and_then(|guid| guids.iter().position(|g| g == guid).map(|i| i as i32))
                    .unwrap_or(-1)
            })
            .collect::<Vec<_>>()
            .try_into()
            .unwrap_or([-1; 4]);

        let config = Self::load_config();
        let mut rom_info_cache = HashMap::new();
        let roms = config
            .rom_folder
            .as_ref()
            .map(|folder| read_dir(folder).ok())
            .flatten()
            .map(|entries| {
                entries
                    .filter_map(Result::ok)
                    .filter(|e| e.file_type().map(|ft| ft.is_file()).unwrap_or(false))
                    .filter(|e| {
                        e.path()
                            .extension()
                            .and_then(|ext| ext.to_str())
                            .map(|ext| VALID_ROM_EXTENSIONS.contains(&ext))
                            .unwrap_or(false)
                    })
                    .map(|e| e.path())
                    .collect::<Vec<_>>()
            })
            .unwrap_or_default();

        roms.iter().for_each(|path| {
            if let Some(info) = RomInfo::new(path) {
                rom_info_cache.insert(path.clone(), info);
            }
        });

        let settings = Settings::default();

        P64Gui {
            configure_profile: false,
            profile_name: String::new(),
            selected_profile: game_ui.config.config.input.input_profile_binding.clone(),
            selected_controller,
            controllers: get_controllers(&game_ui),
            input_profiles: get_input_profiles(&game_ui),
            rom_folder: config.rom_folder,
            roms,
            settings_open: false,
            rom_info_cache,
            settings
        }
    }

    fn scan_rom_directory(&mut self) {
        if let Some(folder) = &self.rom_folder {
            self.roms.clear();
            self.rom_info_cache.clear();

            if let Ok(entries) = read_dir(folder) {
                self.roms.extend(
                    entries
                        .filter_map(|entry| entry.ok())
                        .filter(|entry| entry.file_type().map(|ft| ft.is_file()).unwrap_or(false))
                        .filter(|entry| {
                            entry
                                .path()
                                .extension()
                                .and_then(|ext| ext.to_str())
                                .map(|ext| VALID_ROM_EXTENSIONS.contains(&ext))
                                .unwrap_or(false)
                        })
                        .map(|entry| entry.path()),
                );

                for rom_path in &self.roms {
                    if let Some(info) = RomInfo::new(rom_path) {
                        self.rom_info_cache.insert(rom_path.clone(), info);
                    }
                }
            }
        }
    }

    fn load_config() -> Config {
        let config_path = match config_dir() {
            Some(dir) => dir.join("p64").join("roms.json"),
            None => return Config::default(),
        };

        match fs::read_to_string(&config_path) {
            Ok(contents) => serde_json::from_str(&contents).unwrap_or_default(),
            Err(_) => Config::default(),
        }
    }

    fn save_config(&self) {
        let config_dir = match config_dir() {
            Some(dir) => dir.join("p64"),
            None => return,
        };

        if let Err(e) = fs::create_dir_all(&config_dir) {
            eprintln!("Failed to create config directory: {}", e);
            return;
        }

        let config = Config {
            rom_folder: self.rom_folder.clone(),
        };

        let config_json = match serde_json::to_string_pretty(&config) {
            Ok(json) => json,
            Err(e) => {
                eprintln!("Failed to serialize config: {}", e);
                return;
            }
        };

        if let Err(e) = fs::write(config_dir.join("roms.json"), config_json) {
            eprintln!("Failed to write config file: {}", e);
        }
    }

    fn select_rom_folder(&mut self) {
        let (tx, rx) = mpsc::channel();

        let task = rfd::AsyncFileDialog::new()
            .set_directory(dirs::home_dir().unwrap_or_default())
            .pick_folder();

        execute(async move {
            if let Some(folder) = task.await {
                let folder_path = folder.path().to_path_buf();
                let _ = tx.send(folder_path);
            }
        });

        // Handle the result in the main thread
        if let Ok(path) = rx.recv() {
            self.rom_folder = Some(path);
            self.scan_rom_directory();
            self.save_config();
        }
    }

    fn show_profile_config(&mut self, ctx: &egui::Context) {
        if !self.configure_profile {
            return;
        }

        Window::new("Configure Input Profile")
            .resizable(false)
            .collapsible(false)
            .min_width(300.0)
            .show(ctx, |ui| {
                egui::Frame::none()
                    .inner_margin(egui::vec2(10.0, 10.0))
                    .rounding(4.0)
                    .show(ui, |ui| {
                        ui.vertical_centered(|ui| {
                            ui.add_space(8.0);

                            ui.horizontal(|ui| {
                                let name_label =
                                    ui.label(RichText::new("Profile Name:").strong().size(16.0));
                                ui.add_space(8.0);
                                ui.text_edit_singleline(&mut self.profile_name)
                                    .labelled_by(name_label.id)
                            });

                            ui.add_space(16.0);

                            ui.horizontal(|ui| {
                                if ui
                                    .add_sized(
                                        [120.0, 30.0],
                                        Button::new(RichText::new("Configure Profile").strong()),
                                    )
                                    .clicked()
                                {
                                    self.handle_profile_config();
                                }

                                ui.add_space(8.0);

                                if ui
                                    .add_sized(
                                        [80.0, 30.0],
                                        Button::new(RichText::new("Close").strong()),
                                    )
                                    .clicked()
                                {
                                    self.configure_profile = false;
                                }
                            });

                            ui.add_space(8.0);
                        });
                    });
            });
    }

    fn show_settings(&mut self, ctx: &egui::Context) {
        if !self.settings_open {
            return;
        }
    
        let mut close = false;
        
        Window::new("Settings").show(ctx, |ui| {
            ui.collapsing("Controller Settings", |ui| {
                ui.label("Controller Config:");
                self.show_controller_grid(ui);
            });

            
            ui.collapsing("Graphics Settings", |ui| {
                let settings = &mut self.settings;
                let resolutions: [(u32, u32); 5] = [
                    (640, 480),
                    (800, 600),
                    (1024, 768),
                    (1280, 720),
                    (1920, 1080)
                ];
                
                ui.horizontal(|ui| {
                    ui.label("Resolution:");
                    egui::ComboBox::from_label("")
                        .selected_text(format!("{}x{}", settings.graphics.width, settings.graphics.height))
                        .show_ui(ui, |ui| {
                            for (width, height) in resolutions.iter() {
                                let selected = settings.graphics.width == *width && settings.graphics.height == *height;
                                if ui.selectable_label(selected, format!("{}x{}", width, height)).clicked() {
                                    set_viewport(*width as i32, *height as i32);
                                    settings.graphics.width = *width;
                                    settings.graphics.height = *height;
                                }
                            }
                        });
                });
    
                ui.checkbox(&mut settings.graphics.fullscreen, "Fullscreen");

                ui.checkbox(&mut settings.graphics.vsync, "VSync");
    
                ui.horizontal(|ui| {
                ui.label("Upscaling:");
                ui.add_space(8.0);
                
                let scales = [1, 2, 4, 8];
                
                for (i, &scale) in scales.iter().enumerate() {
                    if i > 0 {
                        ui.add_space(4.0);
                    }
                    
                    let button_text = format!("{}×", scale);
                    
                    if ui.selectable_value(
                        &mut settings.graphics.upscaling,
                        scale,
                        button_text
                    ).clicked() {
                        match scale {
                            1 => enable_1x_upscaling(),
                            2 => enable_2x_upscaling(),
                            4 => enable_4x_upscaling(),
                            8 => enable_8x_upscaling(),
                            _ => {}
                        }
                    }
                }
            });
            });
    
            if ui.button("Close").clicked() {
                close = true;
                self.settings_open = false;
            }
        });
    
        // if close {
        //     Some(settings)
        // } else {
        //     None
        // }
    }

    fn show_rom_folder_selector(&mut self, ui: &mut egui::Ui) {
        ui.add_space(16.0);
        let folder_text = if let Some(_rom_folder) = &self.rom_folder {
            format!("{} games loaded", self.roms.len())
        } else {
            "Double-click to select ROM folder".to_string()
        };

        if ui
            .add(
                egui::Label::new(folder_text)
                    .wrap()
                    .sense(egui::Sense::click()),
            )
            .double_clicked()
        {
            self.select_rom_folder();
        }

        ui.add_space(16.0);
    }

    fn show_menu_bar(&mut self, ui: &mut egui::Ui) {
        menu::bar(ui, |ui| {
            ui.menu_button("File", |ui| {
                if ui.button("Open ROM").clicked() {
                    self.handle_open_rom();
                }
                if ui.button("Settings").clicked() {
                    self.settings_open = true;
                }
                if ui.button("Configure Input Profile").clicked() {
                    self.handle_configure_profile();
                }
            });
        });

        ui.separator();
        ui.add_space(16.0);
    }

    fn show_action_buttons(&mut self, ui: &mut egui::Ui) {
        ui.vertical_centered(|ui| {
            ui.horizontal_top(|ui| {
                if ui
                    .add_sized([150.0, 30.0], egui::Button::new("Open ROM"))
                    .clicked()
                {
                    self.handle_open_rom();
                }
                ui.add_space(8.0);

                if ui
                    .add_sized([150.0, 30.0], egui::Button::new("Settings"))
                    .clicked()
                {
                    self.settings_open = true;
                }
                ui.add_space(8.0);

                if ui
                    .add_sized([150.0, 30.0], egui::Button::new("Configure Input Profile"))
                    .clicked()
                {
                    self.handle_configure_profile();
                }
            });
        });
    }

    fn show_rom_list(&self, ui: &mut egui::Ui) {
        ScrollArea::both().show(ui, |ui| {
            ui.add_space(16.0);
            ui.heading("ROM Library");
            ui.add_space(16.0);

            Grid::new("rom_table")
                .num_columns(3)
                .spacing([40.0, 4.0])
                .striped(true)
                .show(ui, |ui| {
                    self.render_rom_table_header(ui);
                    self.render_rom_entries(ui);
                });
        });
    }

    fn handle_profile_config(&mut self) {
        let profile_name = self.profile_name.clone();
        execute(async {
            let mut game_ui = Ui::new();
            configure_profile(&mut game_ui, profile_name);
        });
        self.configure_profile = false;
        if !self.profile_name.is_empty() && !self.input_profiles.contains(&self.profile_name) {
            self.input_profiles.push(self.profile_name.clone());
        }
    }

    fn handle_open_rom(&self) {
        let task = rfd::AsyncFileDialog::new().pick_file();
        let selected_controller = self.selected_controller;
        let selected_profile = self.selected_profile.clone();
        let fullscreen = self.settings.graphics.fullscreen;
        let width = self.settings.graphics.width;
        let height = self.settings.graphics.height;

        execute(async move {
            if let Some(file) = task.await {
                let running_file = dirs::cache_dir().unwrap().join("p64").join("game_running");

                if running_file.exists() {
                    return;
                }

                let _ = File::create(running_file.clone());
                let mut device = N64::new();
                save_config(&mut device.ui, selected_controller, selected_profile);
                run_game(std::path::Path::new(file.path()), &mut device, width, height, fullscreen);

                let _ = remove_file(running_file);
            }
        });
    }

    fn handle_configure_profile(&mut self) {
        let running_file = dirs::cache_dir().unwrap().join("p64").join("game_running");

        if !running_file.exists() {
            self.configure_profile = true;
        }
    }

    fn show_controller_grid(&mut self, ui: &mut egui::Ui) {
        egui::Frame::none()
            .fill(ui.style().visuals.extreme_bg_color)
            .rounding(8.0)
            .inner_margin(egui::vec2(10.0, 10.0))
            .show(ui, |ui| {
                Grid::new("controller_grid")
                    .spacing([20.0, 12.0])
                    .striped(true)
                    .show(ui, |ui| {
                        // Header row
                        ui.heading("Port");
                        ui.heading("Profile");
                        ui.heading("Controller");
                        ui.end_row();

                        for i in 0..4 {
                            // Port number with custom styling
                            ui.label(
                                RichText::new(format!("{}", i + 1))
                                    .strong()
                                    .color(ui.style().visuals.widgets.active.fg_stroke.color),
                            );

                            // Profile selection combobox
                            let profile_combo =
                                ComboBox::from_id_salt(format!("profile-combo-{}", i))
                                    .selected_text(
                                        RichText::new(&self.selected_profile[i]).strong(),
                                    )
                                    .width(150.0);
                            profile_combo.show_ui(ui, |ui| {
                                for profile in &self.input_profiles {
                                    ui.selectable_value(
                                        &mut self.selected_profile[i],
                                        profile.clone(),
                                        profile.clone(),
                                    );
                                }
                            });

                            // Controller selection combobox
                            let controller_text = if self.selected_controller[i] == -1 {
                                "None".to_string()
                            } else {
                                self.controllers[self.selected_controller[i] as usize].clone()
                            };

                            let controller_combo =
                                ComboBox::from_id_salt(format!("controller-combo-{}", i))
                                    .selected_text(RichText::new(&controller_text).strong())
                                    .width(200.0);
                            controller_combo.show_ui(ui, |ui| {
                                ui.selectable_value(
                                    &mut self.selected_controller[i],
                                    -1,
                                    RichText::new("None"),
                                );
                                for (j, controller) in self.controllers.iter().enumerate() {
                                    ui.selectable_value(
                                        &mut self.selected_controller[i],
                                        j as i32,
                                        controller.clone(),
                                    );
                                }
                            });
                            ui.end_row();
                        }
                    });
            });
    }

    fn render_rom_table_header(&self, ui: &mut egui::Ui) {
        ui.strong("Filename");
        ui.strong("Internal Name");
        ui.strong("MD5");
        ui.end_row();
    }

    fn render_rom_entries(&self, ui: &mut egui::Ui) {
        for rom_path in &self.roms {
            if let Some(info) = self.rom_info_cache.get(rom_path) {
                if ui.link(info.file_name.clone()).clicked() {
                    let selected_controller = self.selected_controller;
                    let selected_profile = self.selected_profile.clone();
                    let rom_path = rom_path.clone();
                    let fullscreen = self.settings.graphics.fullscreen;
                    let width = self.settings.graphics.width;
                    let height = self.settings.graphics.height;

                    execute(async move {
                        let running_file =
                            dirs::cache_dir().unwrap().join("p64").join("game_running");

                        if running_file.exists() {
                            return;
                        }

                        let _ = File::create(running_file.clone());
                        let mut device = N64::new();
                        save_config(&mut device.ui, selected_controller, selected_profile);

                        run_game(&rom_path, &mut device, width, height, fullscreen);
                        let _ = remove_file(running_file);
                    });
                }
                ui.label(info.internal_name.clone());
                ui.label(info.md5.clone().to_uppercase());
                ui.end_row();
            }
        }
    }
}

impl Drop for P64Gui {
    fn drop(&mut self) {
        let mut game_ui = Ui::new();
        save_config(
            &mut game_ui,
            self.selected_controller,
            self.selected_profile.clone(),
        );
    }
}

impl eframe::App for P64Gui {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.show_profile_config(ctx);
        self.show_settings(ctx);

        egui::CentralPanel::default().show(ctx, |ui| {
            self.show_menu_bar(ui);
            self.show_action_buttons(ui);
            self.show_rom_folder_selector(ui);
            self.show_rom_list(ui);
        });
    }
}
