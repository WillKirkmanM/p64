use super::{mmu::MmuRegister, memory, N64};

#[derive(Copy, Clone, Default)]
pub struct TlbEntry {
    pub mask: u64,
    pub vpn2: u64,
    pub region: u8,
    pub g: u8,
    pub asid: u8,
    pub pfn_even: u64,
    pub c_even: u8,
    pub d_even: u8,
    pub v_even: u8,
    pub pfn_odd: u64,
    pub c_odd: u8,
    pub d_odd: u8,
    pub v_odd: u8,
    pub start_even: u64,
    pub end_even: u64,
    pub phys_even: u64,
    pub start_odd: u64,
    pub end_odd: u64,
    pub phys_odd: u64,
}

#[derive(Copy, Clone)]
pub struct TlbLookupTable {
    pub address: u64,
    pub cached: bool,
}

pub fn read(n64: &mut N64,index: u64) {
    if index > 31 { return; }
    let entry = &n64.cpu.mmu.tlb_entries[index as usize];
    n64.cpu.mmu.regs[MmuRegister::PageMask as usize] = entry.mask << 13;
    n64.cpu.mmu.regs[MmuRegister::EntryHi as usize] = (entry.region as u64) << 62 | entry.vpn2 << 13 | entry.asid as u64;
    n64.cpu.mmu.regs[MmuRegister::EntryLo0 as usize] = (entry.pfn_even << 6) | (entry.c_even << 3) as u64 | (entry.d_even << 2) as u64 | (entry.v_even << 1) as u64 | entry.g as u64;
    n64.cpu.mmu.regs[MmuRegister::EntryLo1 as usize] = (entry.pfn_odd << 6) | (entry.c_odd << 3) as u64 | (entry.d_odd << 2) as u64 | (entry.v_odd << 1) as u64 | entry.g as u64;
}

pub fn write(n64: &mut N64,index: u64) {
    if index > 31 { return; }
    tlb_unmap(n64, index);
    let entry = &mut n64.cpu.mmu.tlb_entries[index as usize];
    entry.g = (n64.cpu.mmu.regs[MmuRegister::EntryLo0 as usize] & n64.cpu.mmu.regs[MmuRegister::EntryLo1 as usize] & 1) as u8;
    entry.pfn_even = (n64.cpu.mmu.regs[MmuRegister::EntryLo0 as usize] >> 6) & 0xFFFFF;
    entry.pfn_odd = (n64.cpu.mmu.regs[MmuRegister::EntryLo1 as usize] >> 6) & 0xFFFFF;
    entry.c_even = ((n64.cpu.mmu.regs[MmuRegister::EntryLo0 as usize] >> 3) & 7) as u8;
    entry.c_odd = ((n64.cpu.mmu.regs[MmuRegister::EntryLo1 as usize] >> 3) & 7) as u8;
    entry.d_even = ((n64.cpu.mmu.regs[MmuRegister::EntryLo0 as usize] >> 2) & 1) as u8;
    entry.d_odd = ((n64.cpu.mmu.regs[MmuRegister::EntryLo1 as usize] >> 2) & 1) as u8;
    entry.v_even = ((n64.cpu.mmu.regs[MmuRegister::EntryLo0 as usize] >> 1) & 1) as u8;
    entry.v_odd = ((n64.cpu.mmu.regs[MmuRegister::EntryLo1 as usize] >> 1) & 1) as u8;
    entry.asid = n64.cpu.mmu.regs[MmuRegister::EntryHi as usize] as u8;
    entry.mask = (n64.cpu.mmu.regs[MmuRegister::PageMask as usize] >> 13) & 0xFFF;
    entry.vpn2 = (n64.cpu.mmu.regs[MmuRegister::EntryHi as usize] >> 13) & 0x7FFFFFF;
    entry.region = (n64.cpu.mmu.regs[MmuRegister::EntryHi as usize] >> 62) as u8;
    entry.mask &= 0b101010101010;
    entry.mask |= entry.mask >> 1;
    entry.vpn2 &= !entry.mask;
    entry.start_even = (entry.vpn2 << 13) & 0xFFFFFFFF;
    entry.end_even = entry.start_even + (entry.mask << 12) + 0xFFF;
    entry.phys_even = entry.pfn_even << 12;
    entry.start_odd = entry.end_even + 1;
    entry.end_odd = entry.start_odd + (entry.mask << 12) + 0xFFF;
    entry.phys_odd = entry.pfn_odd << 12;
    tlb_map(n64, index);
}

pub fn probe(n64: &mut N64) {
    n64.cpu.mmu.regs[MmuRegister::Index as usize] = 0x80000000;
    for (pos, e) in n64.cpu.mmu.tlb_entries.iter().enumerate() {
        if e.vpn2 & !e.mask != ((n64.cpu.mmu.regs[MmuRegister::EntryHi as usize] >> 13) & 0x7FFFFFF) & !e.mask { continue; }
        if e.region != (n64.cpu.mmu.regs[MmuRegister::EntryHi as usize] >> 62) as u8 { continue; }
        if e.g == 0 && e.asid != n64.cpu.mmu.regs[MmuRegister::EntryHi as usize] as u8 { continue; }
        n64.cpu.mmu.regs[MmuRegister::Index as usize] = pos as u64;
        break;
    }
}

pub fn get_physical_address(n64: &mut N64,mut address: u64, access_type: memory::AccessType) -> (u64, bool, bool) {
    address &= 0xffffffff;
    if access_type == memory::AccessType::Write {
        if n64.cpu.mmu.tlb_lut_w[(address >> 12) as usize].address != 0 {
            return ((n64.cpu.mmu.tlb_lut_w[(address >> 12) as usize].address & 0x1FFFF000) | (address & 0xFFF), n64.cpu.mmu.tlb_lut_w[(address >> 12) as usize].cached, false);
        }
    } else if n64.cpu.mmu.tlb_lut_r[(address >> 12) as usize].address != 0 {
        return ((n64.cpu.mmu.tlb_lut_r[(address >> 12) as usize].address & 0x1FFFF000) | (address & 0xFFF), n64.cpu.mmu.tlb_lut_r[(address >> 12) as usize].cached, false);
    }
    crate::exceptions::tlb_miss_exception(n64, address, access_type);
    (0, false, true)
}

fn tlb_unmap(n64: &mut N64,index: u64) {
    let e = &mut n64.cpu.mmu.tlb_entries[index as usize];
    if e.v_even != 0 {
        for i in (e.start_even..e.end_even).step_by(0x1000) {
            n64.cpu.mmu.tlb_lut_r[(i >> 12) as usize].address = 0;
            n64.cpu.mmu.tlb_lut_r[(i >> 12) as usize].cached = false;
            if e.d_even != 0 {
                n64.cpu.mmu.tlb_lut_w[(i >> 12) as usize].address = 0;
                n64.cpu.mmu.tlb_lut_w[(i >> 12) as usize].cached = false;
            }
        }
    }
    if e.v_odd != 0 {
        for i in (e.start_odd..e.end_odd).step_by(0x1000) {
            n64.cpu.mmu.tlb_lut_r[(i >> 12) as usize].address = 0;
            n64.cpu.mmu.tlb_lut_r[(i >> 12) as usize].cached = false;
            if e.d_odd != 0 {
                n64.cpu.mmu.tlb_lut_w[(i >> 12) as usize].address = 0;
                n64.cpu.mmu.tlb_lut_w[(i >> 12) as usize].cached = false;
            }
        }
    }
}

fn tlb_map(n64: &mut N64,index: u64) {
    let e = &mut n64.cpu.mmu.tlb_entries[index as usize];
    if e.v_even != 0 && e.start_even < e.end_even && !(e.start_even >= 0x80000000 && e.end_even < 0xC0000000) && e.phys_even < 0x20000000 {
        for i in (e.start_even..e.end_even).step_by(0x1000) {
            n64.cpu.mmu.tlb_lut_r[(i >> 12) as usize].address = 0x80000000 | (e.phys_even + (i - e.start_even) + 0xFFF);
            n64.cpu.mmu.tlb_lut_r[(i >> 12) as usize].cached = e.c_even != 2;
            if e.d_even != 0 {
                n64.cpu.mmu.tlb_lut_w[(i >> 12) as usize].address = 0x80000000 | (e.phys_even + (i - e.start_even) + 0xFFF);
                n64.cpu.mmu.tlb_lut_w[(i >> 12) as usize].cached = e.c_even != 2;
            }
        }
    }
    if e.v_odd != 0 && e.start_odd < e.end_odd && !(e.start_odd >= 0x80000000 && e.end_odd < 0xC0000000) && e.phys_odd < 0x20000000 {
        for i in (e.start_odd..e.end_odd).step_by(0x1000) {
            n64.cpu.mmu.tlb_lut_r[(i >> 12) as usize].address = 0x80000000 | (e.phys_odd + (i - e.start_odd) + 0xFFF);
            n64.cpu.mmu.tlb_lut_r[(i >> 12) as usize].cached = e.c_odd != 2;
            if e.d_odd != 0 {
                n64.cpu.mmu.tlb_lut_w[(i >> 12) as usize].address = 0x80000000 | (e.phys_odd + (i - e.start_odd) + 0xFFF);
                n64.cpu.mmu.tlb_lut_w[(i >> 12) as usize].cached = e.c_odd != 2;
            }
        }
    }
}