#![no_std]
#![no_main]

#[path = "../utils/mod.rs"]
mod utils;
use utils::logger::info;

use cortex_m_rt::entry;
use stm32_hrtim::{
    compare_register::HrCompareRegister, deadtime::DeadtimeConfig, output::HrOutput,
    timer::{HrSlaveTimer, HrTimer}, HrParts, HrPwmAdvExt, Pscl1,
};
use stm32g4xx_hal::{
    adc::{self, AdcClaim, AdcCommonExt},
    delay::{DelayExt, SYSTDelayExt},
    gpio::GpioExt,
    hrtim::{HrControltExt, HrPwmBuilderExt},
    pwr::PwrExt,
    rcc::{self, RccExt},
    stm32::{CorePeripherals, Peripherals},
    time::ExtU32,
};

#[entry]
fn main() -> ! {
    info!("Initializing...");

    let dp = Peripherals::take().expect("cannot take peripherals");
    let cp = CorePeripherals::take().expect("cannot take core");
    // Set system frequency to 16MHz * 15/1/2 = 120MHz
    // This would lead to HrTim running at 120MHz * 32 = 3.84GHz...
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = dp.RCC.freeze(
        rcc::Config::pll().pll_cfg(rcc::PllConfig {
            mux: rcc::PllSrc::HSI,
            n: rcc::PllNMul::MUL_15,
            m: rcc::PllMDiv::DIV_1,
            r: Some(rcc::PllRDiv::DIV_2),

            ..Default::default()
        }),
        pwr,
    );

    let mut delay = cp.SYST.delay(&rcc.clocks);

    // ...with a prescaler of 1 this gives us a HrTimer with a tick rate of 960MHz
    // With max the max period set, this would be 960MHz/2^16 ~= 14.6kHz...
    let prescaler = Pscl1;

    let gpioa = dp.GPIOA.split(&mut rcc);
    let pin_a = gpioa.pa8; // D7 On nucleo-G474 - LI Blue-White
    let pin_b = gpioa.pa9; // D8 On nucleo-G474 - HI Green
    let pa0 = gpioa.pa0.into_analog(); // CN7.28 On nucleo-G474 - Isense Brown

    //        .               .               .               .
    //        .  30%          .               .               .
    //         ----           .               .----           .
    //out1    |    |          .               |    |          .
    //        |    |          .               |    |          .
    // --------    ----------------------------    --------------------
    //        .               .----           .               .----
    //out2    .               |    |          .               |    |
    //        .               |    |          .               |    |
    // ------------------------    ----------------------------    ----
    //        .               .               .               .
    //        .               .               .               .
    let (hr_control, ..) = dp
        .HRTIM_COMMON
        .hr_control(&mut rcc)
        .set_adc1_trigger_psc(stm32_hrtim::control::AdcTriggerPostscaler::Div32)
        .wait_for_calibration();
    let mut hr_control = hr_control.constrain();

    let deadtime = DeadtimeConfig::default()
        .prescaler(stm32_hrtim::deadtime::DeadtimePrescaler::ThrtimDiv8)
        .deadtime_rising_value(20) // 20 / (8 * 120MHz) = ~21ns
        .deadtime_falling_value(20); // 20 / (8 * 120MHz) = ~21ns
    let period = 0xF00;
    let HrParts {
        mut timer,
        mut cr1,
        mut cr3,
        out: (mut out1, mut out2),
        ..
    } = dp
        .HRTIM_TIMA
        .pwm_advanced((pin_a, pin_b))
        .prescaler(prescaler)
        .period(period) // (120MHz * 32) / 0xF00 = 1MHz
        .preload(stm32_hrtim::PreloadSource::OnCounterReset)
        .deadtime(deadtime)
        .out1_polarity(stm32_hrtim::Polarity::ActiveHigh)
        .out2_polarity(stm32_hrtim::Polarity::ActiveHigh)
        .finalize(&mut hr_control);

    out1.enable_rst_event(&cr1); // Set low on compare match with cr1
    out2.enable_rst_event(&cr1);

    out1.enable_set_event(&timer); // Set high at new period
    out2.enable_set_event(&timer);

    cr1.set_duty(period / 10);
    cr3.set_duty(period / 10);

    hr_control.adc_trigger1.enable_source(&cr3);

    info!("Setup Adc1");
    let adc12_common = dp
        .ADC12_COMMON
        .claim(adc::config::ClockMode::AdcHclkDiv4, &mut rcc);
    let adc = adc12_common.claim(dp.ADC1, &mut delay);

    let mut adc = adc.power_down().into_dynamic_adc();
    adc.enable();
    adc.set_external_trigger((
        adc::config::TriggerMode::RisingEdge,
        (&hr_control.adc_trigger1).into(),
    ));
    adc.reset_sequence();
    adc.configure_channel(
        &pa0,
        adc::config::Sequence::One,
        adc::config::SampleTime::Cycles_12_5,
    );

    out1.enable();
    out2.enable();
    timer.start(&mut hr_control.control);

    info!("Start");
    adc.start_conversion();

    loop {
        let start = (50 * u32::from(period) / 100) as u16;
        let end = (51 * u32::from(period) / 100) as u16;
        let duty = start;
        //for duty in start..end {
            let mut results = [0; 99];
            let ad_sample_point = 1 * period / 100;
            cr3.set_duty(ad_sample_point);
            //info!("wait_for_conversion_sequence");
            /*adc.wait_for_conversion_sequence();

            /for i in 2..100 {
                let ad_sample_point = (i * u32::from(period) / 100) as u16;

                adc.wait_for_conversion_sequence();
                cr3.set_duty(ad_sample_point);
                let index: usize = (i - 2) as usize;
                results[index] = adc.current_sample();
            }*/
            /*for r in results {
                print_bar(r);
            }*/
            if (100 * u32::from(duty)) % u32::from(period) == 0 {
                info!("Duty: {}%", 100 * u32::from(duty) / u32::from(period));
            }
            cr1.set_duty(period - duty); // Invert duty since HI is what we consider the main switch

            delay.delay(4_u32.millis());
        //}
    }
}

fn print_bar(x: u16) {
    //let x = (4096 / 2) - x as i16;
    let s = ['-'; 100];
    let s = &s[0..(x as usize / 41)];
    defmt::println!("{}", x);
}
