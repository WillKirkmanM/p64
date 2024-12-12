use std::{fs::{write, File}, path::PathBuf};

use serde_json::{to_writer, Map};

use crate::ui::Ui;

#[derive(Copy, Clone, PartialEq)]
pub enum SaveTypes {
    Eeprom4k,
    Eeprom16k,
    Flash,
    Mempak,
    Romsave,
    Sram,
}

#[derive(Default)]
pub struct SaveData {
    pub eeprom: (Vec<u8>, bool),
    pub sram: (Vec<u8>, bool),
    pub flash: (Vec<u8>, bool),
    pub mempak: (Vec<u8>, bool),
    pub romsave: (Map<String, serde_json::Value>, bool),
    pub eeprom_path: PathBuf,
    pub sram_path: PathBuf,
    pub flash_path: PathBuf,
    pub mempak_path: PathBuf,
    pub romsave_path: PathBuf,
}

pub fn write_save(ui: &Ui, save_type: SaveTypes) {
    let (path, data) = match save_type {
        SaveTypes::Romsave => {
            let f = File::create(ui.save_data.data.romsave_path.clone()).unwrap();
            to_writer(f, &ui.save_data.data.romsave).unwrap();
            return;
        }
        SaveTypes::Eeprom4k | SaveTypes::Eeprom16k => (&ui.save_data.data.eeprom_path, &ui.save_data.data.eeprom.0),
        SaveTypes::Sram => (&ui.save_data.data.sram_path, &ui.save_data.data.sram.0),
        SaveTypes::Flash => (&ui.save_data.data.flash_path, &ui.save_data.data.flash.0),
        SaveTypes::Mempak => (&ui.save_data.data.mempak_path, &ui.save_data.data.mempak.0),
    };

    write(path, data).expect("Could not write save file! Do we have the write permissions?");
}