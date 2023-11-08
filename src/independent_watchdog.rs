//! Independent Watchdog
//!
//! This module implements the embedded-hal
//! [Watchdog](https://docs.rs/embedded-hal/latest/embedded_hal/watchdog/trait.Watchdog.html)
//! trait for the Independent Watchdog peripheral.
//!
//! The Independent Watchdog peripheral triggers a system reset when its internal counter expires.
//!
//! # Examples
//!
//! - [IWDG Example](todo-insert-link-here)
//!
//! Originally from stm32h7-hal, adapted for stm32g4xx-hal
use crate::{
    stm32::{iwdg::pr::PR_A, IWDG},
    time::MilliSecond,
};

/// The implementation of the hardware IWDG
pub struct IndependentWatchdog {
    iwdg: IWDG,
}

impl IndependentWatchdog {
    const CLOCK_SPEED: u32 = 32000;
    const MAX_COUNTER_VALUE: u32 = 0x00000FFF;
    const MAX_MILLIS_FOR_PRESCALER: [(PR_A, u32); 8] = [
        (
            PR_A::DivideBy4,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 4),
        ),
        (
            PR_A::DivideBy8,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 8),
        ),
        (
            PR_A::DivideBy16,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 16),
        ),
        (
            PR_A::DivideBy32,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 32),
        ),
        (
            PR_A::DivideBy64,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 64),
        ),
        (
            PR_A::DivideBy128,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 128),
        ),
        (
            PR_A::DivideBy256,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 256),
        ),
        (
            PR_A::DivideBy256bis,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 256),
        ),
    ];

    /// Create a new instance
    pub fn new(iwdg: IWDG) -> Self {
        Self { iwdg }
    }

    /// Feed the watchdog, resetting the timer to 0
    pub fn feed(&mut self) {
        self.iwdg.kr.write(|w| w.key().reset());
    }

    /// Start the watchdog where it must be fed before the max time is over and
    /// not before the min time has passed
    pub fn start_windowed<T: Into<MilliSecond>>(&mut self, min_window_time: T, max_window_time: T) {
        let min_window_time: MilliSecond = min_window_time.into();
        let max_window_time: MilliSecond = max_window_time.into();

        // Start the watchdog
        self.iwdg.kr.write(|w| w.key().start());
        // Enable register access
        self.iwdg.kr.write(|w| w.key().enable());

        // Set the prescaler
        let (prescaler, _) = Self::MAX_MILLIS_FOR_PRESCALER
            .iter()
            .find(|(_, max_millis)| *max_millis >= max_window_time.to_millis())
            .expect("IWDG max time is greater than is possible");
        while self.iwdg.sr.read().pvu().bit_is_set() {
            cortex_m::asm::nop();
        }
        self.iwdg.pr.write(|w| w.pr().variant(*prescaler));

        // Reset the window value
        while self.iwdg.sr.read().wvu().bit_is_set() {
            cortex_m::asm::nop();
        }
        self.iwdg
            .winr
            .write(|w| w.win().bits(Self::MAX_COUNTER_VALUE as u16));

        // Calculate the counter values
        let reload_value = max_window_time.to_millis() * (Self::CLOCK_SPEED / 1000)
            / Self::get_prescaler_divider(prescaler);
        let window_value = min_window_time.to_millis() * (Self::CLOCK_SPEED / 1000)
            / Self::get_prescaler_divider(prescaler);

        // Set the reload value
        while self.iwdg.sr.read().rvu().bit_is_set() {
            cortex_m::asm::nop();
        }
        self.iwdg.rlr.write(|w| w.rl().bits(reload_value as u16));

        self.feed();
        // Enable register access
        self.iwdg.kr.write(|w| w.key().enable());

        // Set the window value
        while self.iwdg.sr.read().wvu().bit_is_set() {
            cortex_m::asm::nop();
        }
        self.iwdg
            .winr
            .write(|w| w.win().bits((reload_value - window_value) as u16));

        // Wait until everything is set
        while self.iwdg.sr.read().bits() != 0 {
            cortex_m::asm::nop();
        }

        self.feed();
    }

    /// Start the watchdog with the given max time and no minimal time
    pub fn start<T: Into<MilliSecond>>(&mut self, max_time: T) {
        use crate::time::ExtU32;

        self.start_windowed(0_u32.millis(), max_time.into());
    }

    fn get_prescaler_divider(prescaler: &PR_A) -> u32 {
        match prescaler {
            PR_A::DivideBy4 => 4,
            PR_A::DivideBy8 => 8,
            PR_A::DivideBy16 => 16,
            PR_A::DivideBy32 => 32,
            PR_A::DivideBy64 => 64,
            PR_A::DivideBy128 => 128,
            PR_A::DivideBy256 => 256,
            PR_A::DivideBy256bis => 256,
        }
    }
}
