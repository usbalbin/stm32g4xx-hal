//This example puts the timer in PWM mode using the specified pin with a frequency of 100Hz and a duty cycle of 50%.
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use fugit::ExtU32;
use hal::gpio::gpioa::PA8;
use hal::gpio::Alternate;
use hal::gpio::AF13;
use hal::prelude::*;
use hal::pwm::hrtim::EventSource;
use hal::pwm::hrtim::FaultAction;
use hal::pwm::hrtim::HrCompareRegister;
use hal::pwm::hrtim::HrPwmAdvExt;
use hal::pwm::hrtim::HrTimer;
use hal::pwm::hrtim::Pscl4;
use hal::pwm::hrtim::{HrControltExt, HrOutput};
use hal::pwm::FaultMonitor;
use hal::rcc;
use hal::stm32;
use stm32g4xx_hal as hal;
//mod utils;

use defmt_rtt as _; // global logger
use panic_probe as _;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let cp = stm32::CorePeripherals::take().expect("cannot take core");
    // Set system frequency to 16MHz * 75/4/2 = 150MHz
    // This would lead to HrTim running at 150MHz * 32 = 4.8GHz...
    let mut rcc = dp.RCC.freeze(rcc::Config::pll().pll_cfg(rcc::PllConfig {
        mux: rcc::PLLSrc::HSI,
        n: rcc::PllNMul::MUL_75,
        m: rcc::PllMDiv::DIV_4,
        r: Some(rcc::PllRDiv::DIV_2),
        ..Default::default()
    }));

    let mut delay = cp.SYST.delay(&rcc.clocks);

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let (mut fault_control, flt_inputs, eev_inputs) =
        dp.HRTIM_COMMON.hr_control(&mut rcc).wait_for_calibration();

    let eev_input3 = eev_inputs
        .eev_input3
        .bind_pin(gpiob.pb7.into_pull_down_input())
        .polarity(hal::pwm::Polarity::ActiveHigh)
        .finalize(&mut fault_control);

    // ...with a prescaler of 4 this gives us a HrTimer with a tick rate of 1.2GHz
    // With max the max period set, this would be 1.2GHz/2^16 ~= 18kHz...
    let prescaler = Pscl4;

    let pin_a: PA8<Alternate<AF13>> = gpioa.pa8.into_alternate();

    //        .               .  *            .
    //        .  33%          .  *            .               .               .
    //        .-----.         .--*            .-----.         .-----.         .-----
    //out1    |     |         |  |            |     |         |     |         |
    //        |     |         |  *            |     |         |     |         |
    //   ------     -----------  --------------     -----------     -----------
    //        .               .  *            .               .               .
    //        .               .  *            .               .               .
    //        .               .  *--------*   .               .               .
    //eev     .               .  |        |   .               .               .
    //        .               .  |        |   .               .               .
    //   -------------------------        ------------------------------------------
    //        .               .  *            .               .               .
    //        .               .  *            .               .               .
    let (timer, (mut cr1, _cr2, _cr3, _cr4), mut out1) = dp
        .HRTIM_TIMA
        .pwm_advanced(pin_a, &mut rcc)
        .prescaler(prescaler)
        .period(0xFFFF)
        .finalize(&mut fault_control);

    out1.enable_rst_event(EventSource::Cr1); // Set low on compare match with cr1
    out1.enable_rst_event(&eev_input3);
    out1.enable_set_event(EventSource::Period); // Set high at new period
    cr1.set_duty(timer.get_period() / 3);
    //unsafe {((HRTIM_COMMON::ptr() as *mut u8).offset(0x14) as *mut u32).write_volatile(1); }
    out1.enable();

    defmt::info!("Started");

    loop {
        for _ in 0..5 {
            delay.delay(500_u32.millis());
            defmt::info!("State: {}", out1.get_state());
        }
        if fault_control.fault_3.is_fault_active() {
            fault_control.fault_3.clear_fault(); // Clear fault every 5s
            out1.enable();
            defmt::info!("failt cleared, and output reenabled");
        }
    }
}
