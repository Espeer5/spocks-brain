//! Provides software timers for the RP2040. Software timers are multiplexed on
//! ALARM0, and we allow custom low-level timer functionality on ALARM1-3.

// Public API -- not all functions must be used
#![allow(dead_code)]

use core::cell::RefCell;
use core::cmp::Ordering;

use cortex_m::interrupt::{self, Mutex};
use heapless::binary_heap::{BinaryHeap, Min};
use rp_pico::hal::pac::TIMER;

use super::constants::SW_TIMER_MAX_HEAP;

////////////////////////////////////////////////////////////////////////////////
// TIMER CONSTANTS
////////////////////////////////////////////////////////////////////////////////

/// Microsecond ticks per second (RP2040 timer counts at 1 MHz).
pub const TIMER_TICKS_PER_SECOND: u32 = 1_000_000;

////////////////////////////////////////////////////////////////////////////////
// TIMER TYPES
////////////////////////////////////////////////////////////////////////////////

/// Handle for a generic software timer ([`schedule_soft`]).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct SoftTimerId(pub u16);

/// One-shot or periodic mode for [`schedule_soft`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SoftMode {
    OneShot,
    Periodic { period_us: u32 },
}

/// Dedicated hardware alarm (never ALARM0).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DedicatedAlarm {
    Alarm1,
    Alarm2,
    Alarm3,
}

impl DedicatedAlarm {
    const fn slot_index(self) -> usize {
        match self {
            DedicatedAlarm::Alarm1 => 0,
            DedicatedAlarm::Alarm2 => 1,
            DedicatedAlarm::Alarm3 => 2,
        }
    }

    const fn hw_alarm(self) -> u8 {
        match self {
            DedicatedAlarm::Alarm1 => 1,
            DedicatedAlarm::Alarm2 => 2,
            DedicatedAlarm::Alarm3 => 3,
        }
    }

    fn from_slot(i: usize) -> Self {
        match i {
            0 => DedicatedAlarm::Alarm1,
            1 => DedicatedAlarm::Alarm2,
            _ => DedicatedAlarm::Alarm3,
        }
    }
}

/// Mode for [`schedule_dedicated`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DedicatedMode {
    OneShot,
    Periodic { period_us: u32 },
}

#[derive(Clone, Copy)]
struct SoftSlot {
    deadline: u32,
    mode: SoftMode,
    callback: fn(SoftTimerId),
}

/// Min-heap key: earliest [`deadline`], tie-break on [`slot`] index.
#[derive(Clone, Copy, Eq, PartialEq)]
struct HeapEntry {
    deadline: u32,
    slot: u16,
}

impl Ord for HeapEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        self.deadline
            .cmp(&other.deadline)
            .then_with(|| self.slot.cmp(&other.slot))
    }
}

impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

struct SoftTimerStore {
    slots: [Option<SoftSlot>; SW_TIMER_MAX_HEAP],
    heap: BinaryHeap<HeapEntry, Min, SW_TIMER_MAX_HEAP>,
}

impl SoftTimerStore {
    const fn new() -> Self {
        Self {
            slots: [None; SW_TIMER_MAX_HEAP],
            heap: BinaryHeap::new(),
        }
    }

    /// Rebuild the min-heap from [`slots`] (e.g. after [`cancel_soft`]).
    fn rebuild_heap(&mut self) {
        self.heap.clear();
        for (i, slot) in self.slots.iter().enumerate() {
            if let Some(sl) = slot {
                // Invariant: heap capacity matches slot count
                let _ = self.heap.push(HeapEntry {
                    deadline: sl.deadline,
                    slot: i as u16,
                });
            }
        }
    }
}

struct DedicatedSlot {
    mode: Option<DedicatedMode>,
    callback: Option<fn(DedicatedAlarm)>,
}

////////////////////////////////////////////////////////////////////////////////
// STATIC TIMER STORAGE
////////////////////////////////////////////////////////////////////////////////

static SOFT_TIMERS: Mutex<RefCell<SoftTimerStore>> =
    Mutex::new(RefCell::new(SoftTimerStore::new()));

static DEDICATED: Mutex<RefCell<[DedicatedSlot; 3]>> = Mutex::new(RefCell::new([
    DedicatedSlot {
        mode: None,
        callback: None,
    },
    DedicatedSlot {
        mode: None,
        callback: None,
    },
    DedicatedSlot {
        mode: None,
        callback: None,
    },
]));

////////////////////////////////////////////////////////////////////////////////
// TIMER HELPER ROUTINES
////////////////////////////////////////////////////////////////////////////////

/// Clear pending timer IRQ bits and disable all timer interrupt enables. Does
// not touch NVIC.
pub fn init_timer_hardware(timer: &mut TIMER) {
    timer.intr().write(|w| {
        w.alarm_0().clear_bit_by_one();
        w.alarm_1().clear_bit_by_one();
        w.alarm_2().clear_bit_by_one();
        w.alarm_3().clear_bit_by_one()
    });
    timer.inte().write(|w| unsafe { w.bits(0) });
}

/// Configure NVIC priorities so timer IRQs **preempt** UART0 RX.
///
/// On Cortex-M0+, interrupts at the same priority cannot preempt each other. A
/// floating or noisy UART RX line can re-enter `UART0_IRQ` continuously; that
/// delays [`TIMER_IRQ_0`] and makes periodic callbacks (e.g. LED blink) appear
/// to run at a much slower rate than requested.
///
/// Call **before** [`enable_timer_interrupts`] or [`crate::platform::uart::enable_uart0_interrupt`].
pub fn configure_irq_priorities() {
    use rp_pico::hal::pac::Interrupt;

    let mut cp = cortex_m::Peripherals::take().expect("cortex_m::Peripherals");

    // 0 = highest priority on RP2040 (2-bit NVIC field → 4 levels).
    unsafe {
        cp.NVIC.set_priority(Interrupt::TIMER_IRQ_0, 0);
        cp.NVIC.set_priority(Interrupt::TIMER_IRQ_1, 0);
        cp.NVIC.set_priority(Interrupt::TIMER_IRQ_2, 0);
        cp.NVIC.set_priority(Interrupt::TIMER_IRQ_3, 0);
        cp.NVIC.set_priority(Interrupt::UART0_IRQ, 2);
    }
}

/// Unmask `TIMER_IRQ_0` … `TIMER_IRQ_3` in the NVIC. Call after
// [`init_timer_hardware`].
pub fn enable_timer_interrupts() {
    use cortex_m::asm;
    use cortex_m::peripheral::NVIC;
    use rp_pico::hal::pac::Interrupt;

    // Drop stale pending state from earlier runs or spurious edges before unmask.
    NVIC::unpend(Interrupt::TIMER_IRQ_0);
    NVIC::unpend(Interrupt::TIMER_IRQ_1);
    NVIC::unpend(Interrupt::TIMER_IRQ_2);
    NVIC::unpend(Interrupt::TIMER_IRQ_3);

    unsafe {
        NVIC::unmask(Interrupt::TIMER_IRQ_0);
        NVIC::unmask(Interrupt::TIMER_IRQ_1);
        NVIC::unmask(Interrupt::TIMER_IRQ_2);
        NVIC::unmask(Interrupt::TIMER_IRQ_3);
        // Ensure NVIC writes take effect before IRQ can fire (ARM recommendation).
        asm::dsb();
        asm::isb();
        // Bootloader / startup can leave PRIMASK set; IRQs must be globally enabled.
        cortex_m::interrupt::enable();
    }
}

/// Busy-wait using the microsecond timer (no interrupts). For short delays after clocks are up.
pub fn delay_us_blocking(timer: &mut TIMER, us: u32) {
    let start = read_timelr(timer);
    let target = start.wrapping_add(us);
    while !is_deadline_reached(read_timelr(timer), target) {
        cortex_m::asm::nop();
    }
}

/// Read full 64-bit time in microseconds (read `timelr` then `timehr` per
// datasheet).
pub fn read_time_us(timer: &TIMER) -> u64 {
    let low = timer.timelr().read().bits() as u64;
    let high = timer.timehr().read().bits() as u64;
    (high << 32) | low
}

fn read_timelr(timer: &TIMER) -> u32 {
    timer.timelr().read().bits()
}

/// Low 32 bits of the microsecond free-running counter (`TIMELR`).
pub fn timelr_now(timer: &TIMER) -> u32 {
    read_timelr(timer)
}

/// `true` if `now` is at or after `deadline` in 32-bit wrap-safe sense.
// Designed to match Linux's `time_after_eq`.
fn is_deadline_reached(now: u32, deadline: u32) -> bool {
    (now.wrapping_sub(deadline) as i32) >= 0
}

/// Public wrapper for comparing `TIMELR` values (wrap-safe).
pub fn time_reached(now: u32, deadline: u32) -> bool {
    is_deadline_reached(now, deadline)
}

/// Arm the given hardware alarm at the given target time.
fn arm_at_timelr(timer: &mut TIMER, hw_alarm: u8, target: u32) {
    match hw_alarm {
        0 => timer.alarm0().write(|w| unsafe { w.bits(target) }),
        1 => timer.alarm1().write(|w| unsafe { w.bits(target) }),
        2 => timer.alarm2().write(|w| unsafe { w.bits(target) }),
        3 => timer.alarm3().write(|w| unsafe { w.bits(target) }),
        _ => {}
    }
}

/// Clear the pending interrupt for the given hardware alarm.
fn clear_intr(timer: &mut TIMER, hw_alarm: u8) {
    timer.intr().write(|w| match hw_alarm {
        0 => w.alarm_0().clear_bit_by_one(),
        1 => w.alarm_1().clear_bit_by_one(),
        2 => w.alarm_2().clear_bit_by_one(),
        3 => w.alarm_3().clear_bit_by_one(),
        _ => w,
    });
}

/// Disarm the given hardware alarm.
fn disarm_hw(timer: &mut TIMER, hw_alarm: u8) {
    let mask = 1u32 << hw_alarm;
    timer.armed().write(|w| unsafe { w.bits(mask) });
}

/// Synchronize **ALARM0** / `INTE` bit 0 with the min-heap root (next expiry).
fn sync_alarm0(timer: &mut TIMER, store: &SoftTimerStore) {
    if let Some(e) = store.heap.peek() {
        timer.inte().modify(|r, w| unsafe { w.bits(r.bits() | 1) });
        arm_at_timelr(timer, 0, e.deadline);
    } else {
        timer.inte().modify(|r, w| unsafe { w.bits(r.bits() & !1) });
    }
}

/// Calculate the mask for the given hardware alarm.
fn mask(hw: u8) -> u32 {
    1u32 << hw
}

/// Handle the inner logic of a dedicated alarm.
fn handle_dedicated_inner(timer: &mut TIMER, slot_idx: usize) {
    let hw = (slot_idx + 1) as u8;
    clear_intr(timer, hw);

    let (cb, mode) = interrupt::free(|cs| {
        let store = DEDICATED.borrow(cs).borrow();
        let d = &*store;
        let slot = &d[slot_idx];
        (slot.callback, slot.mode)
    });

    let Some(f) = cb else {
        return;
    };
    let Some(m) = mode else {
        return;
    };

    f(DedicatedAlarm::from_slot(slot_idx));

    match m {
        DedicatedMode::OneShot => {
            interrupt::free(|cs| {
                let mut store = DEDICATED.borrow(cs).borrow_mut();
                let d = &mut *store;
                d[slot_idx].mode = None;
                d[slot_idx].callback = None;
            });
            timer
                .inte()
                .modify(|r, w| unsafe { w.bits(r.bits() & !mask(hw)) });
        }
        DedicatedMode::Periodic { period_us } => {
            let now = read_timelr(timer);
            let next = now.wrapping_add(period_us);
            arm_at_timelr(timer, hw, next);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// PUB SOFT TIMER ROUTINES
////////////////////////////////////////////////////////////////////////////////

/// Schedule a generic software timer on ALARM0. Fails if the table is full.
pub fn schedule_soft(
    timer: &mut TIMER,
    delay_us: u32,
    mode: SoftMode,
    callback: fn(SoftTimerId),
) -> Result<SoftTimerId, ()> {
    interrupt::free(|cs| {
        let now = read_timelr(timer);
        let deadline = now.wrapping_add(delay_us);

        let mut store = SOFT_TIMERS.borrow(cs).borrow_mut();
        let s = &mut *store;

        let idx = s.slots.iter().position(|o| o.is_none()).ok_or(())?;
        s.slots[idx] = Some(SoftSlot {
            deadline,
            mode,
            callback,
        });

        let entry = HeapEntry {
            deadline,
            slot: idx as u16,
        };
        if s.heap.push(entry).is_err() {
            s.slots[idx] = None;
            return Err(());
        }

        sync_alarm0(timer, s);

        Ok(SoftTimerId(idx as u16))
    })
}

/// Cancel a generic software timer. Safe if the id was never used or already fired.
pub fn cancel_soft(timer: &mut TIMER, id: SoftTimerId) {
    interrupt::free(|cs| {
        let mut store = SOFT_TIMERS.borrow(cs).borrow_mut();
        let s = &mut *store;
        let i = id.0 as usize;
        if i < s.slots.len() {
            s.slots[i] = None;
        }
        s.rebuild_heap();
        sync_alarm0(timer, s);
    });
}

/// `TIMER_IRQ_0` handler: drain due timers from the min-heap, re-arm **ALARM0**
pub fn handle_alarm0_multiplex(timer: &mut TIMER) {
    clear_intr(timer, 0);

    loop {
        let now = read_timelr(timer);

        let step = interrupt::free(|cs| {
            let mut store = SOFT_TIMERS.borrow(cs).borrow_mut();
            let s = &mut *store;

            loop {
                let entry = match s.heap.peek().copied() {
                    None => {
                        sync_alarm0(timer, s);
                        return None;
                    }
                    Some(e) => e,
                };

                if !is_deadline_reached(now, entry.deadline) {
                    sync_alarm0(timer, s);
                    return None;
                }

                let e = s.heap.pop().expect("peek implied non-empty");

                let valid = match &s.slots[e.slot as usize] {
                    None => false,
                    Some(sl) => sl.deadline == e.deadline,
                };
                if !valid {
                    continue;
                }

                let idx = e.slot as usize;
                let slot = s.slots[idx].take().expect("validated slot");
                let id = SoftTimerId(idx as u16);
                let cb = slot.callback;
                let mode_after = match slot.mode {
                    SoftMode::OneShot => None,
                    SoftMode::Periodic { period_us } => {
                        let next_dl = slot.deadline.wrapping_add(period_us);
                        Some(SoftSlot {
                            deadline: next_dl,
                            mode: SoftMode::Periodic { period_us },
                            callback: slot.callback,
                        })
                    }
                };
                return Some((cb, id, idx, mode_after));
            }
        });

        let Some((cb, id, idx, mode_after)) = step else {
            return;
        };

        cb(id);

        interrupt::free(|cs| {
            let mut store = SOFT_TIMERS.borrow(cs).borrow_mut();
            let s = &mut *store;
            if let Some(ns) = mode_after {
                s.slots[idx] = Some(ns);
                let _ = s.heap.push(HeapEntry {
                    deadline: ns.deadline,
                    slot: idx as u16,
                });
            }
            sync_alarm0(timer, s);
        });
    }
}

/// Schedule a dedicated alarm on **ALARM1**, **ALARM2**, or **ALARM3**
// (not multiplexed).
pub fn schedule_dedicated(
    timer: &mut TIMER,
    alarm: DedicatedAlarm,
    delay_us: u32,
    mode: DedicatedMode,
    callback: fn(DedicatedAlarm),
) -> Result<(), ()> {
    let idx = alarm.slot_index();
    let hw = alarm.hw_alarm();

    interrupt::free(|cs| {
        let mut store = DEDICATED.borrow(cs).borrow_mut();
        let d = &mut *store;
        d[idx].mode = Some(mode);
        d[idx].callback = Some(callback);

        let now = read_timelr(timer);
        let deadline = now.wrapping_add(delay_us);

        let m = 1u32 << hw;
        timer.inte().modify(|r, w| unsafe { w.bits(r.bits() | m) });
        arm_at_timelr(timer, hw, deadline);
    });

    Ok(())
}

/// Disarm and clear a dedicated alarm slot.
pub fn cancel_dedicated(timer: &mut TIMER, alarm: DedicatedAlarm) {
    let hw = alarm.hw_alarm();
    let idx = alarm.slot_index();
    disarm_hw(timer, hw);
    interrupt::free(|cs| {
        let mut store = DEDICATED.borrow(cs).borrow_mut();
        let d = &mut *store;
        d[idx].mode = None;
        d[idx].callback = None;
    });
    timer
        .inte()
        .modify(|r, w| unsafe { w.bits(r.bits() & !mask(hw)) });
}

/// `TIMER_IRQ_1` handler body.
pub fn handle_dedicated_alarm1(timer: &mut TIMER) {
    handle_dedicated_inner(timer, 0);
}

/// `TIMER_IRQ_2` handler body.
pub fn handle_dedicated_alarm2(timer: &mut TIMER) {
    handle_dedicated_inner(timer, 1);
}

/// `TIMER_IRQ_3` handler body.
pub fn handle_dedicated_alarm3(timer: &mut TIMER) {
    handle_dedicated_inner(timer, 2);
}
