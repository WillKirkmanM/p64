use std::fs::remove_file;

use dirs::{cache_dir, config_dir};

pub fn ensure_directories() {
    let dirs = (
        config_dir().unwrap().join("p64"),
        cache_dir().unwrap().join("p64"),
    );
    for dir in [&dirs.0, &dirs.1] {
        let _ = std::fs::create_dir_all(dir);
    }
    let _ = remove_file(dirs.1.join("game_running"));
}
