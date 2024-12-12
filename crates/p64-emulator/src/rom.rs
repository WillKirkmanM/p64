use std::{io::Read, path::Path};

#[derive(Clone)]
pub struct RomInfo {
    pub file_name: String,
    pub internal_name: String,
    pub md5: String,
}

impl RomInfo {
    pub fn new(path: &Path) -> Option<Self> {
        let file_name = path.file_name()?.to_str()?
            .rsplit_once('.')
            .map_or("", |t| t.0)
            .to_string();
    
        let mut file = std::fs::File::open(path).ok()?;
        
        let mut header = vec![0; 80];
        file.read_exact(&mut header).ok()?;
        
        let internal_name = header[32..]
            .iter()
            .take_while(|&&byte| byte != 0)
            .map(|&byte| byte as char)
            .collect::<String>()
            .trim()
            .to_string();
    
        let mut buffer = Vec::new();
        file.read_to_end(&mut buffer).ok()?;
        let md5 = format!("{:x}", md5::compute(&buffer));
    
        Some(RomInfo {
            file_name,
            internal_name,
            md5,
        })
    }
}