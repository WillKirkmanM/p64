use super::N64;

pub const MEMPAK_SIZE: usize = 0x8000;
pub const MPK_PAGE_SIZE: usize = 256;

const LABEL_AREA: [u8; 32] = [
    0x81, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e,
    0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
    0x1e, 0x1f,
];

const MAIN_ID_AREA: [u8; 32] = [
    0xff, 0xff, 0xff, 0xff, 0x05, 0x1a, 0x5f, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0xff, 0x66, 0x25,
    0x99, 0xcd,
];

const UNUSED_AREA: [u8; 32] = [0x00; 32];

const ID_AREA_BACKUP: [u8; 32] = [
    0xff, 0xff, 0xff, 0xff, 0x05, 0x1a, 0x5f, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0xff, 0x66, 0x25,
    0x99, 0xcd,
];

fn copy_slice(dest: &mut [u8], src: &[u8], start: usize) {
    let mut i = 0;
    while i < src.len() {
        dest[start + i] = src[i];
        i += 1;
    }
}

fn initialize_page_0() -> [u8; MPK_PAGE_SIZE] {
    let mut page = [0u8; MPK_PAGE_SIZE];
    copy_slice(&mut page, &LABEL_AREA, 0);
    copy_slice(&mut page, &MAIN_ID_AREA, 32);
    copy_slice(&mut page, &UNUSED_AREA, 64);
    copy_slice(&mut page, &ID_AREA_BACKUP, 96);
    copy_slice(&mut page, &ID_AREA_BACKUP, 128);
    copy_slice(&mut page, &UNUSED_AREA, 160);
    copy_slice(&mut page, &ID_AREA_BACKUP, 192);
    copy_slice(&mut page, &UNUSED_AREA, 224);
    page
}

fn fill_page_0(n64: &mut N64,offset: usize) {
    let page_0 = initialize_page_0();
    n64.ui.save_data.data.mempak.0[offset..offset + MPK_PAGE_SIZE].copy_from_slice(&page_0);
}

fn fill_inode_page(n64: &mut N64,offset: usize) {
    let start_page = 5;
    for i in MPK_PAGE_SIZE..2 * start_page {
        n64.ui.save_data.data.mempak.0[offset + i] = 0;
    }
    for i in (MPK_PAGE_SIZE + 2 * start_page..2 * MPK_PAGE_SIZE).step_by(2) {
        n64.ui.save_data.data.mempak.0[offset + i] = 0x00;
        n64.ui.save_data.data.mempak.0[offset + i + 1] = 0x03;
    }
    n64.ui.save_data.data.mempak.0[offset + (MPK_PAGE_SIZE + 1)] = 0x71;
}

fn fill_remaining_pages(n64: &mut N64,offset: usize) {
    for i in 3 * MPK_PAGE_SIZE..MEMPAK_SIZE - 3 * MPK_PAGE_SIZE {
        n64.ui.save_data.data.mempak.0[offset + i] = 0;
    }
}

fn format_mempak(n64: &mut N64) {
    if n64.ui.save_data.data.mempak.0.len() < MEMPAK_SIZE * 4 {
        n64.ui.save_data.data.mempak.0.resize(MEMPAK_SIZE * 4, 0);

        for i in 0..4 {
            let offset = i * MEMPAK_SIZE;

            fill_page_0(n64, offset);
            fill_inode_page(n64, offset);

            let page1 = offset + MPK_PAGE_SIZE;
            let page2 = offset + 2 * MPK_PAGE_SIZE;
            let page1data = n64.ui.save_data.data.mempak.0[page1..page1 + MPK_PAGE_SIZE].to_vec();
            n64.ui.save_data.data.mempak.0[page2..page2 + MPK_PAGE_SIZE].copy_from_slice(&page1data);

            fill_remaining_pages(n64, offset);
        }
    }
}

fn handle_read_write(n64: &mut N64, channel: usize, address: u16, data: usize, size: usize, is_write: bool) {
    if (address as usize) < MEMPAK_SIZE {
        format_mempak(n64);

        let offset = (channel * MEMPAK_SIZE) + address as usize;
        if is_write {
            n64.ui.save_data.data.mempak.0[offset..offset + size]
                .copy_from_slice(&n64.pif.ram[data..data + size]);
            n64.ui.save_data.data.mempak.1 = true;
        } else {
            n64.pif.ram[data..data + size]
                .copy_from_slice(&n64.ui.save_data.data.mempak.0[offset..offset + size]);
        }
    } else {
        for i in 0..size {
            n64.pif.ram[data + i] = 0;
        }
    }
}

pub fn read(n64: &mut N64,channel: usize, address: u16, data: usize, size: usize) {
    handle_read_write(n64, channel, address, data, size, false);
}

pub fn write(n64: &mut N64,channel: usize, address: u16, data: usize, size: usize) {
    handle_read_write(n64, channel, address, data, size, true);
}