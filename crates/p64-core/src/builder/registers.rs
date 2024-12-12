use crate::{memory::AccessSize, N64};
use std::ops::{Index, IndexMut, Range};

type RegReadCallback = fn(&N64, u64, AccessSize) -> u32;
type RegWriteCallback = fn(&mut N64,u64, u32, u32);

#[derive(Default, Debug)]
pub struct Registers<T = u32> {
    pub size: usize,
    pub regs: Vec<T>,
    pub read_mask: Option<T>,
    pub write_mask: Option<T>,
    pub read_callback: Option<RegReadCallback>,
    pub write_callback: Option<RegWriteCallback>,
}

impl<T: Default + Copy> Registers<T> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn build(self) -> Registers<T> {
        Registers {
            size: self.size,
            regs: vec![T::default(); self.size],
            read_mask: self.read_mask,
            write_mask: self.write_mask,
            read_callback: self.read_callback,
            write_callback: self.write_callback,
        }
    }

    pub fn with_size(mut self, size: usize) -> Self {
        self.regs = vec![T::default(); size];
        self.size = size;
        self
    }

    pub fn with_write_mask(mut self, mask: T) -> Self {
        self.write_mask = Some(mask);
        self
    }
}

// Implement specific methods for u32 registers
impl Registers<u32> {
    pub fn try_as_array_mut(&mut self) -> Option<&mut [u32; 8]> {
        if self.regs.len() != 8 {
            return None;
        }
        self.regs.as_mut_slice().try_into().ok()
    }
}

// Implement Index/IndexMut for both types
impl<T> Index<usize> for Registers<T> {
    type Output = T;
    
    fn index(&self, index: usize) -> &Self::Output {
        &self.regs[index]
    }
}

impl<T> IndexMut<usize> for Registers<T> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.regs[index]
    }
}

impl<T> Index<Range<usize>> for Registers<T> {
    type Output = [T];
    
    fn index(&self, range: Range<usize>) -> &Self::Output {
        &self.regs[range]
    }
}

impl<T> IndexMut<Range<usize>> for Registers<T> {
    fn index_mut(&mut self, range: Range<usize>) -> &mut Self::Output {
        &mut self.regs[range]
    }
}

// From implementations for both types
impl From<Vec<u32>> for Registers<u32> {
    fn from(regs: Vec<u32>) -> Self {
        let size = regs.len();
        Registers {
            size,
            regs,
            read_mask: None,
            write_mask: None,
            read_callback: None,
            write_callback: None,
        }
    }
}

impl From<Vec<u64>> for Registers<u64> {
    fn from(regs: Vec<u64>) -> Self {
        let size = regs.len();
        Registers {
            size,
            regs,
            read_mask: None,
            write_mask: None,
            read_callback: None,
            write_callback: None,
        }
    }
}

// AsRef/AsMut implementations for both types
impl<T> AsRef<[T]> for Registers<T> {
    fn as_ref(&self) -> &[T] {
        &self.regs
    }
}

impl<T> AsMut<[T]> for Registers<T> {
    fn as_mut(&mut self) -> &mut [T] {
        &mut self.regs
    }
}

impl<T> Registers<T> 
where 
    T: Default + Copy + std::ops::BitAnd<Output = T> + std::ops::BitOr<Output = T> + 
        std::ops::Not<Output = T> + From<u32> + Into<u32>
{
    pub fn read(&self, n64: &N64, address: u64, access_size: AccessSize) -> T {
        // Calculate register index from address
        let reg = ((address & 0xFFFF) >> 2) as usize;
        
        // Check bounds
        if reg >= self.size {
            return T::default();
        }

        // Get base value
        let value = self.regs[reg];

        // Apply callback or mask
        if let Some(callback) = self.read_callback {
            T::from(callback(n64, address, access_size))
        } else if let Some(mask) = self.read_mask {
            value & mask
        } else {
            value
        }
    }

}

