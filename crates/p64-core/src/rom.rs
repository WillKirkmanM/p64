use std::{
    fs,
    io::{self, Error, ErrorKind, Read},
    path::Path,
};

use zip::ZipArchive;

const Z64_MAGIC: u32 = 0x80371240;
const V64_MAGIC: u32 = 0x37804012;
const N64_MAGIC: u32 = 0x40123780;

// N64 ROMs can come in 3 formats based on byte ordering:
//
// 1. Z64 (.z64) - Big-endian (native N64 format)
//    Bytes are in order: [1,2,3,4]
//    No swapping needed
//
// 2. V64 (.v64) - Byte-swapped in pairs
//    Bytes are stored as: [2,1,4,3]
//    Need to swap each pair of bytes
//
// 3. N64 (.n64) - Little-endian/word-swapped
//    Bytes are stored as: [4,3,2,1]
//    Need to swap each group of 4 bytes
//
// The first 4 bytes of the ROM contain a magic number that identifies the format.
// This function detects the format and converts it to native Z64 format.
//
// Example for word size = 0x12345678:
// Z64: 12 34 56 78 (native/big-endian)
// V64: 21 43 65 87 (byte-swapped)
// N64: 78 56 34 12 (word-swapped/little-endian)

fn set_correct_rom_format(contents: Vec<u8>) -> io::Result<Vec<u8>> {
    let magic = u32::from_be_bytes(
        contents[..4]
            .try_into()
            .map_err(|_| Error::new(ErrorKind::InvalidData, "Invalid ROM header"))?,
    );

    match magic {
        Z64_MAGIC => Ok(contents),
        V64_MAGIC => {
            let swapped: Vec<u8> = contents
                .chunks_exact(2)
                .flat_map(|chunk| chunk.iter().rev().cloned())
                .collect();
            Ok(swapped)
        }
        N64_MAGIC => {
            // n64 format - bytes need to be swapped in groups of four
            let swapped: Vec<u8> = contents
                .chunks_exact(4)
                .flat_map(|chunk| chunk.iter().rev().cloned())
                .collect();
            Ok(swapped)
        }
        _ => Err(Error::new(ErrorKind::InvalidData, "Unknown ROM format")),
    }
}

pub fn read_rom(file_path: &Path) -> Vec<u8> {
    let mut contents = vec![];
    if file_path.extension().unwrap().to_ascii_lowercase() == "zip" {
        let zip_file = fs::File::open(file_path).unwrap();
        let mut archive = ZipArchive::new(zip_file).unwrap();
        for i in 0..archive.len() {
            let mut file = archive.by_index(i).unwrap();
            let extension = file
                .enclosed_name()
                .unwrap()
                .extension()
                .unwrap()
                .to_ascii_lowercase();
            if extension == "z64" || extension == "n64" || extension == "v64" {
                file.read_to_end(&mut contents)
                    .expect("This file is not an accepted N64 ROM format");
                break;
            }
        }
    } else {
        contents = fs::read(file_path).expect("Unable to read the ROM file");
    }

    set_correct_rom_format(contents).expect("Could not read the ROM format? is it a z64, n64, or v64?")
}
