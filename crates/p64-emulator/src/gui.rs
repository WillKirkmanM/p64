use crate::config::{Config, ControllerSettings};
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
    settings: crate::config::Settings,
    drag_drop_active: bool,
    drag_hover_highlight: bool,
    search_query: String
}

#[derive(Default, Clone)]
pub struct Settings {
    pub graphics: GraphicsSettings,
}

#[derive(Debug, Clone)]
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

        let config = Self::load_config();
        let settings = config.settings.unwrap_or_default();
        
        let selected_controller = config.controller_settings
            .as_ref()
            .map(|cs| cs.selected_controller)
            .unwrap_or_else(|| {
                game_ui.config.config.input.controller_assignment
                    .iter()
                    .enumerate()
                    .map(|(_pos, item)| {
                        item.as_ref()
                            .and_then(|guid| guids.iter().position(|g| g == guid).map(|i| i as i32))
                            .unwrap_or(-1)
                    })
                    .collect::<Vec<_>>()
                    .try_into()
                    .unwrap_or([-1; 4])
            });
        
        let selected_profile = config.controller_settings
            .as_ref()
            .map(|cs| cs.selected_profile.clone())
            .unwrap_or_else(|| game_ui.config.config.input.input_profile_binding.clone());
        
        set_viewport(settings.graphics.width as i32, settings.graphics.height as i32);
        match settings.graphics.upscaling {
            1 => enable_1x_upscaling(),
            2 => enable_2x_upscaling(),
            4 => enable_4x_upscaling(),
            8 => enable_8x_upscaling(),
            _ => {}
        }

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

        P64Gui {
            configure_profile: false,
            profile_name: String::new(),
            selected_profile,
            selected_controller,
            controllers: get_controllers(&game_ui),
            input_profiles: get_input_profiles(&game_ui),
            rom_folder: config.rom_folder,
            roms,
            settings_open: false,
            rom_info_cache,
            settings,
            drag_drop_active: false,
            drag_hover_highlight: false,
            search_query: String::new()
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
            Some(dir) => dir.join("p64").join("config.json"),
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

        let controller_settings = ControllerSettings {
            selected_controller: self.selected_controller,
            selected_profile: self.selected_profile.clone(),
        };

        let config = Config {
            rom_folder: self.rom_folder.clone(),
            settings: Some(self.settings.clone()),
            controller_settings: Some(controller_settings),
        };

        let config_json = match serde_json::to_string_pretty(&config) {
            Ok(json) => json,
            Err(e) => {
                eprintln!("Failed to serialize config: {}", e);
                return;
            }
        };

        if let Err(e) = fs::write(config_dir.join("config.json"), config_json) {
            eprintln!("Failed to write config file: {}", e);
        } else {
            println!("Settings saved successfully");
        }
    }

    fn save_settings(&self) {
        println!("Saving settings: {}x{}, fullscreen={}, upscale={}x", 
            self.settings.graphics.width,
            self.settings.graphics.height,
            self.settings.graphics.fullscreen,
            self.settings.graphics.upscaling
        );
        self.save_config();
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
            
            let mut settings_changed = false;
            let mut controller_settings_changed = false;
            
            Window::new("Settings")
                .resizable(true)
                .default_size([500.0, 400.0])
                .show(ctx, |ui| {
                    ScrollArea::vertical().show(ui, |ui| {
                        egui::CollapsingHeader::new(RichText::new("Controller Settings").heading().strong())
                            .default_open(true)
                            .show(ui, |ui| {
                                ui.add_space(8.0);
                                
                                let old_controllers = self.selected_controller.clone();
                                let old_profiles = self.selected_profile.clone();
                                
                                self.show_controller_grid(ui);
                                
                                if old_controllers != self.selected_controller || old_profiles != self.selected_profile {
                                    controller_settings_changed = true;
                                }
                                
                                ui.add_space(16.0);
                            });
                        
                        egui::CollapsingHeader::new(RichText::new("Graphics Settings").heading().strong())
                            .default_open(true)
                            .show(ui, |ui| {
                                let settings = &mut self.settings;
                                ui.add_space(8.0);
                                
                                egui::Frame::none()
                                    .fill(ui.style().visuals.faint_bg_color)
                                    .rounding(8.0)
                                    .inner_margin(8.0)
                                    .show(ui, |ui| {
                                        ui.heading("Display");
                                        ui.add_space(4.0);
                                        
                                        ui.horizontal(|ui| {
                                            ui.label("Resolution:");
                                            ui.add_space(8.0);
                                            
                                            let resolutions: [(u32, u32); 5] = [
                                                (640, 480),
                                                (800, 600),
                                                (1024, 768),
                                                (1280, 720),
                                                (1920, 1080)
                                            ];
                                            
                                            egui::ComboBox::from_label("")
                                                .selected_text(format!("{}x{}", settings.graphics.width, settings.graphics.height))
                                                .show_ui(ui, |ui| {
                                                    for (width, height) in resolutions.iter() {
                                                        let selected = settings.graphics.width == *width && settings.graphics.height == *height;
                                                        if ui.selectable_label(selected, format!("{}x{}", width, height)).clicked() {
                                                            set_viewport(*width as i32, *height as i32);
                                                            settings.graphics.width = *width;
                                                            settings.graphics.height = *height;
                                                            settings_changed = true;
                                                        }
                                                    }
                                                });
                                        });
                                        
                                        ui.add_space(8.0);
                                        if ui.checkbox(&mut settings.graphics.fullscreen, "Fullscreen").changed() {
                                            settings_changed = true;
                                        }
                                        
                                        ui.add_space(4.0);
                                        if ui.checkbox(&mut settings.graphics.vsync, "VSync").changed() {
                                            settings_changed = true;
                                        }
                                    });
                                    
                                ui.add_space(16.0);
                                
                                egui::Frame::none()
                                    .fill(ui.style().visuals.faint_bg_color)
                                    .rounding(8.0)
                                    .inner_margin(8.0)
                                    .show(ui, |ui| {
                                        ui.heading("Graphics Quality");
                                        ui.add_space(8.0);
                                        
                                        ui.label("Upscaling Factor:");
                                        ui.add_space(4.0);
                                        
                                        ui.horizontal(|ui| {
                                            let scales = [1, 2, 4, 8];
                                            
                                            for (i, &scale) in scales.iter().enumerate() {
                                                if i > 0 {
                                                    ui.add_space(8.0);
                                                }
                                                
                                                if ui.selectable_value(
                                                    &mut settings.graphics.upscaling,
                                                    scale,
                                                    format!("{}×", scale)
                                                ).clicked() {
                                                    match scale {
                                                        1 => enable_1x_upscaling(),
                                                        2 => enable_2x_upscaling(),
                                                        4 => enable_4x_upscaling(),
                                                        8 => enable_8x_upscaling(),
                                                        _ => {}
                                                    }
                                                    settings_changed = true;
                                                }
                                            }
                                        });
                                        
                                        ui.add_space(4.0);
                                        ui.label(RichText::new("Higher values improve graphics quality but require more performance").weak().small());
                                    });
                            });
                            
                        ui.with_layout(egui::Layout::bottom_up(egui::Align::RIGHT), |ui| {
                            if ui.button("Close").clicked() {
                                if settings_changed || controller_settings_changed {
                                    self.save_settings();
                                }
                                self.settings_open = false;
                            }
                            
                            ui.add_space(4.0);
                            
                            if ui.button("Apply Settings").clicked() {
                                set_viewport(self.settings.graphics.width as i32, self.settings.graphics.height as i32);
                                match self.settings.graphics.upscaling {
                                    1 => enable_1x_upscaling(),
                                    2 => enable_2x_upscaling(),
                                    4 => enable_4x_upscaling(),
                                    8 => enable_8x_upscaling(),
                                    _ => {}
                                }
                                
                                self.save_all_settings();
                                self.settings_open = false;
                            }
                        });
                    });
                });
                
            if settings_changed {
                self.save_config();
            }
            
            if controller_settings_changed {
                self.save_controller_config();
            }
        }

        fn save_all_settings(&mut self) {
            self.save_settings();
            
            self.save_controller_config();
            
            println!("All settings saved successfully");
        }
        
        fn save_controller_config(&mut self) {
            println!("Saving controller settings");
            
            let mut game_ui = Ui::new();
            
            save_config(
                &mut game_ui,
                self.selected_controller,
                self.selected_profile.clone(),
            );
            println!("Controller configuration saved successfully");
        }

    fn show_rom_folder_selector(&mut self, ui: &mut egui::Ui) {
        ui.add_space(8.0);
        
        egui::Frame::none()
            .fill(ui.style().visuals.widgets.inactive.bg_fill)
            .stroke(ui.style().visuals.widgets.inactive.bg_stroke)
            .inner_margin(10.0)
            .rounding(8.0)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.vertical(|ui| {
                        ui.label(RichText::new("ROM Directory").strong());
                        
                        let folder_text = if let Some(rom_folder) = &self.rom_folder {
                            rom_folder.display().to_string()
                        } else {
                            "No folder selected".to_string()
                        };
                        
                        ui.label(folder_text);
                    });
                    
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.button("Browse...").clicked() {
                            self.select_rom_folder();
                        }
                    });
                });
                
                ui.add_space(4.0);
                
                ui.horizontal(|ui| {
                    ui.label(format!("{} ROMs found", self.roms.len()));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.weak("Drop ROM files or folders here");
                    });
                });
            });
        
        ui.add_space(8.0);
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

        fn show_rom_list(&mut self, ui: &mut egui::Ui) {
            ui.add_space(16.0);
            
            ui.horizontal(|ui| {
                ui.heading("ROM Library");
                ui.add_space(10.0);
                
                let visible_count = if self.search_query.is_empty() {
                    self.roms.len()
                } else {
                    self.roms.iter()
                        .filter(|path| {
                            if let Some(info) = self.rom_info_cache.get(*path) {
                                info.file_name.to_lowercase().contains(&self.search_query.to_lowercase()) ||
                                info.internal_name.to_lowercase().contains(&self.search_query.to_lowercase())
                            } else {
                                false
                            }
                        })
                        .count()
                };
                
                ui.weak(format!("({} games{})", 
                    visible_count,
                    if !self.search_query.is_empty() { format!(" of {}", self.roms.len()) } else { String::new() }
                ));
            });
            
            ui.add_space(8.0);
            
            ui.horizontal(|ui| {
                ui.label("Search: ");
                let search_response = ui.add_sized(
                    [ui.available_width() - 70.0, 24.0], 
                    egui::TextEdit::singleline(&mut self.search_query)
                        .hint_text("Type to search ROM names...")
                );
                
                if !self.search_query.is_empty() {
                    if ui.small_button("✖").clicked() {
                        self.search_query.clear();
                    }
                }
                
                if ui.input(|i| i.key_pressed(egui::Key::F) && i.modifiers.ctrl) {
                    search_response.request_focus();
                }
            });
            
            ui.add_space(16.0);
            
            let available_width = ui.available_width();
            let col1_width = available_width * 0.4;
            let col2_width = available_width * 0.4;
            let col3_width = available_width * 0.2;
            
            let header_rect = ui.max_rect().shrink2(egui::vec2(0.0, ui.text_style_height(&egui::TextStyle::Body)));
            ui.painter().rect_filled(
                header_rect.intersect(ui.max_rect()), 
                0.0, 
                ui.style().visuals.faint_bg_color
            );
            
            ui.horizontal(|ui| {
                ui.strong(RichText::new("Filename").size(16.0))
                    .on_hover_text("Click on a ROM filename to launch it");
                ui.add_space(col1_width - 80.0);
                
                ui.strong(RichText::new("Internal Name").size(16.0))
                    .on_hover_text("Name stored in the ROM header");
                ui.add_space(col2_width - 100.0);
                
                ui.strong(RichText::new("MD5").size(16.0))
                    .on_hover_text("ROM file checksum");
            });
            
            ui.add_space(8.0);
            ui.separator();
            
            ScrollArea::vertical()
                .auto_shrink([false, false])
                .max_height(ui.available_height() - 40.0)
                .show(ui, |ui| {
                    let search_query_lower = self.search_query.to_lowercase();
                    let filtered_roms: Vec<_> = self.roms.iter()
                        .filter(|rom_path| {
                            if self.search_query.is_empty() {
                                return true;
                            }
                            
                            if let Some(info) = self.rom_info_cache.get(*rom_path) {
                                info.file_name.to_lowercase().contains(&search_query_lower) ||
                                info.internal_name.to_lowercase().contains(&search_query_lower)
                            } else {
                                false
                            }
                        })
                        .collect();
                        
                    if filtered_roms.is_empty() && !self.roms.is_empty() {
                        ui.vertical_centered(|ui| {
                            ui.add_space(40.0);
                            ui.label(RichText::new("No ROMs matching your search").size(16.0).weak());
                            ui.add_space(10.0);
                            ui.label("Try a different search term or clear the search box");
                            ui.add_space(40.0);
                        });
                        return;
                    }
                    
                    for (i, rom_path) in filtered_roms.iter().enumerate() {
                        if let Some(info) = self.rom_info_cache.get(*rom_path) {
                            let row_rect = ui.max_rect().shrink2(egui::vec2(0.0, ui.spacing().item_spacing.y));
                            if i % 2 == 0 {
                                ui.painter().rect_filled(
                                    row_rect.intersect(ui.max_rect()),
                                    0.0,
                                    ui.visuals().faint_bg_color,
                                );
                            }
                            
                            ui.horizontal(|ui| {
                                let response = ui.add_sized(
                                    [col1_width, 20.0],
                                    egui::SelectableLabel::new(false, RichText::new(&info.file_name).color(ui.visuals().hyperlink_color))
                                );
                                
                                if response.clicked() {
                                    self.launch_rom((*rom_path).clone());
                                }
                                
                                ui.add_sized(
                                    [col2_width, 20.0], 
                                    egui::Label::new(&info.internal_name)
                                ).on_hover_ui(|ui| {
                                    ui.label(format!("Name: {}", info.internal_name));
                                    ui.label(format!("Path: {}", rom_path.display()));
                                });
                                
                                let md5_short = if info.md5.len() > 10 {
                                    format!("{}...", &info.md5[0..8])
                                } else {
                                    info.md5.clone()
                                };
                                
                                ui.add_sized([col3_width, 20.0], egui::Label::new(md5_short.to_uppercase()))
                                    .on_hover_text(info.md5.clone().to_uppercase());
                            });
                            
                            ui.add_space(4.0);
                        }
                    }
                    
                    if self.roms.is_empty() {
                        
                    ui.vertical_centered(|ui| {
                            ui.add_space(40.0);
                            ui.label(RichText::new("No ROMs found in the selected directory").size(16.0).weak());
                            ui.add_space(10.0);
                            ui.label("Select a ROM folder using the folder button or drop ROMs here");
                            ui.add_space(40.0);
                        });
                    }
                });
        }
        
        fn launch_rom(&self, rom_path: PathBuf) {
            let selected_controller = self.selected_controller;
            let selected_profile = self.selected_profile.clone();
            let fullscreen = self.settings.graphics.fullscreen;
            let width = self.settings.graphics.width;
            let height = self.settings.graphics.height;
            
            execute(async move {
                let running_file = dirs::cache_dir().unwrap().join("p64").join("game_running");
                
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


    fn handle_drag_and_drop(&mut self, ctx: &egui::Context) {
        self.drag_drop_active = !ctx.input(|i| i.raw.dropped_files.is_empty());
        self.drag_hover_highlight = !ctx.input(|i| i.raw.hovered_files.is_empty());
        
        ctx.input(|i| {
            if !i.raw.dropped_files.is_empty() {
                let files = i.raw.dropped_files.clone();
                for file in files {
                    if let Some(path) = file.path {
                        if path.is_file() {
                            if let Some(ext) = path.extension().and_then(|e| e.to_str()) {
                                if VALID_ROM_EXTENSIONS.contains(&ext) {
                                    let selected_controller = self.selected_controller;
                                    let selected_profile = self.selected_profile.clone();
                                    let rom_path = path.clone();
                                    let fullscreen = self.settings.graphics.fullscreen;
                                    let width = self.settings.graphics.width;
                                    let height = self.settings.graphics.height;
                                    
                                    execute(async move {
                                        let running_file = dirs::cache_dir().unwrap().join("p64").join("game_running");
                                        
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
                            }
                        } else if path.is_dir() {
                            self.rom_folder = Some(path);
                            self.scan_rom_directory();
                            self.save_config();
                        }
                    }
                }
            }
        });
    }
    
    fn show_drag_drop_overlay(&self, ui: &mut egui::Ui, _available_size: egui::Vec2) {
        if !self.drag_hover_highlight {
            return;
        }
        
        let rect = ui.max_rect();
        ui.painter().rect(
            rect,
            4.0,
            egui::Color32::from_rgba_premultiplied(20, 100, 150, 100),
            egui::Stroke::new(2.0, egui::Color32::from_rgb(100, 200, 255)),
        );
        
        let text = "Drop ROM file to play\nDrop folder to set as ROM directory";
            
        let text_rect = rect.shrink(100.0);
        ui.painter().text(
            text_rect.center(),
            egui::Align2::CENTER_CENTER,
            text,
            egui::FontId::proportional(24.0),
            egui::Color32::WHITE,
        );
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
                        ui.heading("Port");
                        ui.heading("Profile");
                        ui.heading("Controller");
                        ui.end_row();

                        for i in 0..4 {
                            ui.label(
                                RichText::new(format!("{}", i + 1))
                                    .strong()
                                    .color(ui.style().visuals.widgets.active.fg_stroke.color),
                            );

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
        self.handle_drag_and_drop(ctx);
        
        self.show_profile_config(ctx);
        self.show_settings(ctx);

        egui::CentralPanel::default().show(ctx, |ui| {
            let available_space = ui.available_size();
            
            self.show_menu_bar(ui);
            
            ScrollArea::vertical().show(ui, |ui| {
                if self.drag_drop_active {
                    self.show_drag_drop_overlay(ui, available_space);
                }
                
                self.show_action_buttons(ui);
                self.show_rom_folder_selector(ui);
                
                let list_width = available_space.x - 20.0;
                ui.allocate_ui(egui::vec2(list_width, 0.0), |ui| {
                    self.show_rom_list(ui);
                });
            });
        });

        if self.drag_drop_active {
            ctx.request_repaint();
        }
    }
}
