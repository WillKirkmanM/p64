use crate::N64;

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum EventType {
    AI, SI, VI, PI, DP, SP,
    InterruptCheck, SPDma,
    Compare, Count,
}

#[derive(PartialEq, Copy, Clone)]
pub struct Event {
    pub enabled: bool,
    pub count: u64,
    pub handler: fn(&mut N64),
}

impl Default for Event {
    fn default() -> Self {
        fn default_handler(_n64: &mut N64) {}

        Event {
            enabled: false,
            count: 0,
            handler: default_handler,
        }
    }
}

impl Event {
    fn new(count: u64, handler: fn(&mut N64)) -> Self {
        Self { enabled: true, count, handler }
    }
}

/// Event management functions
pub fn create_event(n64: &mut N64,event_type: EventType, when: u64, handler: fn(&mut N64)) {
    n64.cpu.events[event_type as usize] = Event::new(when, handler);
    set_next_event(n64);
}

pub fn get_event(n64: &mut N64,event_type: EventType) -> Option<&mut Event> {
    let event = &mut n64.cpu.events[event_type as usize];
    event.enabled.then_some(event)
}

pub fn remove_event(n64: &mut N64,event_type: EventType) {
    n64.cpu.events[event_type as usize].enabled = false;
}

pub fn trigger_event(n64: &mut N64) {
    let event_idx = n64.cpu.next_event;
    let event = &mut n64.cpu.events[event_idx];
    
    event.enabled = false;
    (event.handler)(n64);
    set_next_event(n64);
}

pub fn set_next_event(n64: &mut N64) {
    let (pos, count) = n64.cpu.events.iter()
        .enumerate()
        .filter(|(_, e)| e.enabled)
        .min_by_key(|(_, e)| e.count)
        .map(|(pos, e)| (pos, e.count))
        .unwrap_or((0, u64::MAX));

    n64.cpu.next_event = pos;
    n64.cpu.next_event_count = count;
}

pub fn translate_events(n64: &mut N64,old_count: u64, new_count: u64) {
    n64.cpu.events.iter_mut()
        .filter(|e| e.enabled)
        .for_each(|e| e.count = e.count - old_count + new_count);
    set_next_event(n64);
}

#[inline(never)]
pub fn dummy_event(_device: &mut N64) {
    panic!("dummy event");
}