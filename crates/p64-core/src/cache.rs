use p64_proc_macros::{add_cycles, add_rdram_cycles};
use super::mmu::add_cycles;

use super::memory::AccessSize;
use super::{cpu, memory, N64};

const WORD_SIZE: u64 = 4;
const CACHE_LINE_SIZE_ICACHE: usize = 8;
const CACHE_LINE_SIZE_DCACHE: usize = 4;
const TAG_MASK: u64 = !0xFFF;
const ADDR_MASK: u32 = 0x1ffffffc;
const LINE_INDEX_SHIFT_ICACHE: u64 = 5;
const LINE_INDEX_MASK_ICACHE: u64 = 0x1FF;
const WORD_INDEX_SHIFT_ICACHE: u64 = 2;
const WORD_INDEX_MASK_ICACHE: u64 = 7;
const LINE_INDEX_SHIFT_DCACHE: u64 = 4;
const LINE_INDEX_MASK_DCACHE: u64 = 0x1FF;
const WORD_INDEX_SHIFT_DCACHE: u64 = 2;
const WORD_INDEX_MASK_DCACHE: u64 = 3;

#[derive(Copy, Clone)]
pub struct ICache {
    pub valid: bool,
    pub tag: u32,
    pub index: u16,
    pub words: [u32; 8],
    pub instruction: [fn(&mut N64, u32); 8],
}

impl Default for ICache {
    fn default() -> Self {
        fn default_instruction(_device: &mut N64, _value: u32) {}
        
        ICache {
            valid: false,
            tag: 0,
            index: 0,
            words: [0; 8],
            instruction: [default_instruction; 8],
        }
    }
}

#[derive(Copy, Clone, Default)]
pub struct DCache {
    pub valid: bool,
    pub dirty: bool,
    pub tag: u32,
    pub index: u16,
    pub words: [u32; 4],
}

fn calculate_cache_address(tag: u32, index: u16) -> u64 { ((tag | index as u32) & ADDR_MASK) as u64 }

fn read_cache_line(n64: &mut N64,cache_address: u64, size: usize, access_size: AccessSize) -> Vec<u32> {
    let read_fn = n64.memory.memory_map_read[(cache_address >> 16) as usize];
    (0..size).map(|i| {
        let addr = cache_address | (i as u64 * WORD_SIZE);
        read_fn(n64, addr, access_size)
    }).collect()
}

fn write_cache_line(n64: &mut N64,cache_address: u64, words: &[u32]) {
    let write_fn = n64.memory.memory_map_write[(cache_address >> 16) as usize];
    for (i, &word) in words.iter().enumerate() {
        let addr = cache_address | (i as u64 * WORD_SIZE);
        write_fn(n64, addr, word, 0xFFFFFFFF);
    }
}

#[add_cycles(7)]
fn dcache_fill(n64: &mut N64,line_index: usize, phys_address: u64) {
    let tag = (phys_address & TAG_MASK) as u32;
    let cache_address = calculate_cache_address(tag, n64.memory.dcache[line_index].index);
    let words = read_cache_line(n64, cache_address, CACHE_LINE_SIZE_DCACHE, AccessSize::Dcache);

    let dcache_line = &mut n64.memory.dcache[line_index];
    dcache_line.valid = true;
    dcache_line.dirty = false;
    dcache_line.tag = tag;
    dcache_line.words.copy_from_slice(&words);
}

#[add_rdram_cycles(32)]
pub fn icache_writeback(n64: &mut N64,line_index: usize) {
    let tag = n64.memory.icache[line_index].tag;
    let index = n64.memory.icache[line_index].index;
    let words = n64.memory.icache[line_index].words.clone();
    
    let cache_address = calculate_cache_address(tag, index);
    write_cache_line(n64, cache_address, &words);
}

pub fn icache_fetch(n64: &mut N64,phys_address: u64) {
    let line_index = ((phys_address >> LINE_INDEX_SHIFT_ICACHE) & LINE_INDEX_MASK_ICACHE) as usize;

    if !icache_hit(n64, line_index, phys_address) {
        icache_fill(n64, line_index, phys_address);
    }

    let word_index = ((phys_address >> WORD_INDEX_SHIFT_ICACHE) & WORD_INDEX_MASK_ICACHE) as usize;
    let cache_line = &n64.memory.icache[line_index];
    let instruction = cache_line.instruction[word_index];
    let word = cache_line.words[word_index];
    
    instruction(n64, word);
}

pub fn icache_hit(n64: &mut N64,line_index: usize, phys_address: u64) -> bool {
    let cache_line = &n64.memory.icache[line_index];
    cache_line.valid && (cache_line.tag & ADDR_MASK) == (phys_address & TAG_MASK) as u32
}

pub fn dcache_hit(n64: &N64, line_index: usize, phys_address: u64) -> bool {
    let cache_line = &n64.memory.dcache[line_index];
    let masked_tag = cache_line.tag & ADDR_MASK;
    let masked_addr = (phys_address & TAG_MASK) as u32;

    cache_line.valid && masked_tag == masked_addr
}

#[add_rdram_cycles(16)]
pub fn dcache_writeback(n64: &mut N64,line_index: usize) {
    let cache_address = calculate_cache_address(n64.memory.dcache[line_index].tag, n64.memory.dcache[line_index].index);
    let cached_words = n64.memory.dcache[line_index].words.to_vec();
    n64.memory.dcache[line_index].dirty = false;
    write_cache_line(n64, cache_address, &cached_words);
}

#[add_cycles(8)]
pub fn icache_fill(n64: &mut N64,line_index: usize, phys_address: u64) {
    let tag = (phys_address & TAG_MASK) as u32;
    let cache_address = calculate_cache_address(tag, n64.memory.icache[line_index].index);
    let words = read_cache_line(n64, cache_address, CACHE_LINE_SIZE_ICACHE, memory::AccessSize::Icache);

    let mut instructions = [None; CACHE_LINE_SIZE_ICACHE];
    for i in 0..CACHE_LINE_SIZE_ICACHE {
        instructions[i] = Some(cpu::decode_opcode(n64, words[i]));
    }

    let icache_line = &mut n64.memory.icache[line_index];
    icache_line.valid = true;
    icache_line.tag = tag;
    icache_line.words.copy_from_slice(&words);
    for i in 0..CACHE_LINE_SIZE_ICACHE {
        icache_line.instruction[i] = instructions[i].unwrap();
    }
}

pub fn dcache_read(n64: &mut N64,phys_address: u64) -> u32 {
    let line_index = ((phys_address >> LINE_INDEX_SHIFT_DCACHE) & LINE_INDEX_MASK_DCACHE) as usize;
    let word_index = ((phys_address >> WORD_INDEX_SHIFT_DCACHE) & WORD_INDEX_MASK_DCACHE) as usize;

    let cache_line = &n64.memory.dcache[line_index];
    if !dcache_hit(n64, line_index, phys_address) {
        if cache_line.valid && cache_line.dirty {
            dcache_writeback(n64, line_index);
        }
        dcache_fill(n64, line_index, phys_address);
    } else {
        add_cycles(n64, 1);
    }

    n64.memory.dcache[line_index].words[word_index]
}

pub fn dcache_write(n64: &mut N64,phys_address: u64, value: u32, mask: u32) {
    let line_index = ((phys_address >> LINE_INDEX_SHIFT_DCACHE) & LINE_INDEX_MASK_DCACHE) as usize;
    let word_index = ((phys_address >> WORD_INDEX_SHIFT_DCACHE) & WORD_INDEX_MASK_DCACHE) as usize;

    let needs_fill = !dcache_hit(n64, line_index, phys_address);
    let needs_writeback = {
        let cache_line = &n64.memory.dcache[line_index];
        needs_fill && cache_line.valid && cache_line.dirty
    };

    if needs_writeback { dcache_writeback(n64, line_index); }

    if needs_fill { 
        dcache_fill(n64, line_index, phys_address);
    } else {
        add_cycles(n64, 1);
    }

    let cache_line = &mut n64.memory.dcache[line_index];
    memory::masked_write_32(&mut cache_line.words[word_index], value, mask);
    cache_line.dirty = true;
}

pub fn init(n64: &mut N64) {
    for (pos, i) in n64.memory.icache.iter_mut().enumerate() { 
        i.index = (pos << LINE_INDEX_SHIFT_ICACHE) as u16 & 0xFE0
    }
    for (pos, i) in n64.memory.dcache.iter_mut().enumerate() {
        i.index = (pos << LINE_INDEX_SHIFT_DCACHE) as u16 & 0xFF0
    }
}