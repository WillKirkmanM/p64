use std::alloc::{alloc_zeroed, Layout};

use crate::mempak::MEMPAK_SIZE;

pub type WriteCallback = fn(address: u64, value: u32) -> bool;

// Create wrapper type

#[derive(Default, Clone)]
pub struct Memory {
    pub mask: Option<usize>,
    pub size: usize,
    pub data: Vec<u8>,
    pub alignment: usize,
    pub is_little_endian: bool,
    pub is_rdram: bool,
    pub is_mempak: bool,
    pub cycles: Option<u64>,
    pub calculate_cycles_from_addr_size: bool,
    pub write_callback: Option<WriteCallback>,
}

impl Memory {
    pub fn build(self) -> Memory {
        Memory {
            mask: self.mask,
            size: self.size,
            data: self.data,
            alignment: self.alignment,
            is_little_endian: self.is_little_endian,
            is_rdram: self.is_rdram,
            is_mempak: self.is_mempak,
            cycles: self.cycles,
            calculate_cycles_from_addr_size: self.calculate_cycles_from_addr_size,
            write_callback: None,
        }
    }
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_mask(mut self, mask: usize) -> Self {
        self.mask = Some(mask);
        self
    }

    pub fn with_size(mut self, size: usize) -> Self {
        if self.alignment > 0 {
            // Use aligned allocation when alignment is specified
            let layout = Layout::from_size_align(size, self.alignment)
                .expect("Invalid layout");
            let ptr = unsafe { alloc_zeroed(layout) };
            self.data = unsafe { Vec::from_raw_parts(ptr, size, size) };
        } else {
            // Use standard Vec allocation when no alignment needed
            self.data = vec![0; size];
        }
        self.size = size;
        self
    }

    pub fn with_alignment(mut self, alignment: usize) -> Self {
        self.alignment = alignment;
        self
    }

    pub fn is_little_endian(mut self) -> Self {
        self.is_little_endian = true;
        self
    }

    pub fn is_rdram(mut self) -> Self {
        self.is_rdram = true;
        self
    }

    pub fn calculate_cycles_from_addr_size(mut self) -> Self {
        self.calculate_cycles_from_addr_size = true;
        self
    }

    pub fn data_mut(&mut self) -> &mut [u8] {
        &mut self.data
    }

    pub fn len(&self) -> usize {
        self.data.len()
    }

    // pub fn read_mem(&self, address: u64, access_size: AccessSize, n64 Option<&mut N64>) -> u32 {
    //     if self.is_mempak {
    //         let channel = (address >> 16) as usize;
    //         let addr = (address & 0xFFFF) as u16;
            
    //         if (addr as usize) < MEMPAK_SIZE {
    //             let offset = (channel * MEMPAK_SIZE) + addr as usize;
    //             let bytes = &self.data[offset..offset + 4];
    //             return u32::from_be_bytes(bytes.try_into().unwrap());
    //         }
    //         return 0;
    //     }

    //     let masked_address = match self.mask {
    //         Some(mask) => (address as usize & mask) as u64,
    //         None => address,
    //     };

    //     if masked_address + 4 > self.size as u64 {
    //         panic!("Memory access out of bounds");
    //     }

    //     // Handle cycles
    //     if n64.is_some() {
    //         let N64 = n64.unwrap();

    //         if self.calculate_cycles_from_addr_size {
    //             if self.is_rdram {
    //                 add_cycles(n64, rdram_calculate_cycles(address) / (access_size as u64 / 4));
    //             }
    //         } else if let Some(cycles) = self.cycles {
    //             add_cycles(n64, cycles);
    //         }
    //     }

    //     let bytes = &self.data[masked_address as usize..masked_address as usize + 4];
    //     if self.is_little_endian {
    //         u32::from_ne_bytes(bytes.try_into().unwrap())
    //     } else {
    //         u32::from_be_bytes(bytes.try_into().unwrap())
    //     }
    // }

    pub fn read_mem_fast(&self, address: u64) -> u32 {
        if self.is_mempak {
            let channel = (address >> 16) as usize;
            let addr = (address & 0xFFFF) as u16;
            
            if (addr as usize) < MEMPAK_SIZE {
                let offset = (channel * MEMPAK_SIZE) + addr as usize;
                let bytes = &self.data[offset..offset + 4];
                return u32::from_be_bytes(bytes.try_into().unwrap());
            }
            return 0;
        }
        let masked_address = address as usize & self.mask.unwrap_or(0);
        let bytes = self.data[masked_address..masked_address + 4]
            .try_into()
            .unwrap();
        
        if self.is_little_endian {
            u32::from_le_bytes(bytes)
        } else {
            u32::from_be_bytes(bytes)
        }
    }

    pub fn write_mem(&mut self, address: u64, value: u32) {
                if self.is_mempak {
            let channel = (address >> 16) as usize;
            let addr = (address & 0xFFFF) as u16;
            
            if (addr as usize) < MEMPAK_SIZE {
                // Needs N64 to Function. We can't without a shared reference.
                // format_mempak();
                let offset = (channel * MEMPAK_SIZE) + addr as usize;
                let bytes = value.to_be_bytes();
                self.data[offset..offset + 4].copy_from_slice(&bytes);
            }
            return;
        }
        
        if !address < self.size as u64 { return }
        // Try callback first
        if let Some(callback) = self.write_callback {
            if callback(address, value) {
                return;
            }
        }

        let masked_address = match self.mask {
            Some(mask) => (address as usize & mask) as u64,
            None => address,
        };

        if masked_address + 4 > self.size as u64 {
            panic!("Memory access out of bounds");
        }

        let bytes = if self.is_little_endian {
            value.to_ne_bytes()
        } else {
            value.to_be_bytes()
        };

        self.data[masked_address as usize..masked_address as usize + 4].copy_from_slice(&bytes);
    }

    pub fn read_u32(&self, address: u64) -> u32 {
        if self.is_mempak {
            let channel = (address >> 16) as usize;
            let addr = (address & 0xFFFF) as u16;
            
            if (addr as usize) < MEMPAK_SIZE {
                let offset = (channel * MEMPAK_SIZE) + addr as usize;
                let bytes = &self.data[offset..offset + 4];
                return u32::from_be_bytes(bytes.try_into().unwrap());
            }
            return 0;
        }
        let masked_address = address as usize & self.mask.unwrap_or(0);
        let bytes = self.data[masked_address..masked_address + 4]
            .try_into()
            .unwrap();
        
        if self.is_little_endian {
            u32::from_le_bytes(bytes)
        } else {
            u32::from_be_bytes(bytes)
        }
    }

    pub fn write_u32(&mut self, address: u64, value: u32, mask: u32) {
        if self.is_mempak {
            let channel = (address >> 16) as usize;
            let addr = (address & 0xFFFF) as u16;
            
            if (addr as usize) < MEMPAK_SIZE {
                let offset = (channel * MEMPAK_SIZE) + addr as usize;
                let bytes = value.to_be_bytes();
                self.data[offset..offset + 4].copy_from_slice(&bytes);
            }
            return;
        }

        if address >= self.size as u64 { return }

        if let Some(callback) = self.write_callback {
            if callback(address, value) {
                return;
            }
        }

        let masked_address = address as usize & self.mask.unwrap_or(0);
        let bytes = if self.is_little_endian {
            value.to_le_bytes()
        } else {
            value.to_be_bytes()
        };
        
        // Apply mask if needed
        if mask != 0xFFFFFFFF {
            let mut current = self.read_u32(address);
            current = (current & !mask) | (value & mask);
            let bytes = if self.is_little_endian {
                current.to_le_bytes()
            } else {
                current.to_be_bytes()
            };
            self.data[masked_address..masked_address + 4].copy_from_slice(&bytes);
        } else {
            self.data[masked_address..masked_address + 4].copy_from_slice(&bytes);
        }
    }

    // pub fn read_str(&self, address: u64, length: usize) -> String {
    //     let start = address as usize & self.mask.unwrap_or(0);
    //     let end = start + length;
        
    //     if end > self.size {
    //         return String::new();
    //     }

    //     String::from_utf8_lossy(&self.data[start..end]).to_string()
    // }
}

impl From<Vec<u8>> for Memory {
    fn from(data: Vec<u8>) -> Self {
        let size = data.len();
        Memory {
            data,
            size,
            mask: None,
            alignment: 0,
            is_little_endian: true,
            is_rdram: false,
            is_mempak: false,
            cycles: None,
            calculate_cycles_from_addr_size: false,
            write_callback: None,
        }
    }
}

impl std::ops::Index<usize> for Memory {
    type Output = u8;
    
    fn index(&self, index: usize) -> &Self::Output {
        &self.data[index]
    }
}

impl std::ops::IndexMut<usize> for Memory {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.data[index]
    }
}

// Keep existing range implementations
impl std::ops::Index<std::ops::Range<usize>> for Memory {
    type Output = [u8];
    
    fn index(&self, range: std::ops::Range<usize>) -> &Self::Output {
        &self.data[range]
    }
}

impl std::ops::IndexMut<std::ops::Range<usize>> for Memory {
    fn index_mut(&mut self, range: std::ops::Range<usize>) -> &mut Self::Output {
        &mut self.data[range]
    }
}