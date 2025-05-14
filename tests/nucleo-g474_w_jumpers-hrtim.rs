#![no_std]
#![no_main]

/// Requires a jumper from A1<->A2 (arduino naming) aka PA1<->PA4

#[path = "../examples/utils/mod.rs"]
mod utils;

use stm32g4xx_hal::adc::{self, AdcClaim};
use stm32g4xx_hal::comparator::{self, ComparatorSplit};
use stm32g4xx_hal::dac::{self, DacExt, DacOut};
use stm32g4xx_hal::delay::{self, SYSTDelayExt};
use stm32g4xx_hal::gpio::{self, GpioExt};
use stm32g4xx_hal::opamp::{self, Opamp1, OpampEx};
use stm32g4xx_hal::rcc::{self, RccExt};

use hal::stm32;
use stm32g4xx_hal as hal;

#[defmt_test::tests]
mod tests {
    use embedded_hal::delay::DelayNs;
    use stm32g4xx_hal::{
        adc::{self},
        comparator::{self, ComparatorExt},
        dac::DacOut,
        opamp::{self, IntoFollower, IntoPga},
        stasis::Freeze,
    };

    use crate::VREF_ADC_BITS;

    #[test]
    fn hrtim() {
        const VREF: f32 = 3.3;

        defmt::info!("start");

        let dp = Peripherals::take().unwrap();
        let cp = CorePeripherals::take().expect("cannot take core peripherals");

        // Set system frequency to 16MHz * 15/1/2 = 120MHz
        // This would lead to HrTim running at 120MHz * 32 = 3.84...
        defmt::info!("rcc");
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

        let dma::channel::Channels { ch1: dma1ch1, .. } = dp.DMA1.split(&rcc);
        let config = DmaConfig::default()
            .transfer_complete_interrupt(true)
            .circular_buffer(true)
            .memory_increment(true);

        defmt::info!("Setup Gpio");
        let gpioa = dp.GPIOA.split(&mut rcc);
        let pa0 = gpioa.pa0.into_analog();

        let pin_a = gpioa.pa8;
        let pin_b = gpioa.pa9;

        // ...with a prescaler of 4 this gives us a HrTimer with a tick rate of 960MHz
        // With max the max period set, this would be 960MHz/2^16 ~= 15kHz...
        let prescaler = Pscl4;

        //        .               .
        //        .  50%          .
        //         ------          ------
        //out1    |      |        |      |
        //        |      |        |      |
        // --------      ----------      --------
        //        .    ^     ^
        //        .    |     |
        //AD samlp    pa0   temp
        let period = 0xFFFF;
        let (hr_control, ..) = dp.HRTIM_COMMON.hr_control(&mut rcc).wait_for_calibration();
        let mut hr_control = hr_control.constrain();
        let HrParts {
            mut timer,
            mut cr1,
            mut cr3,
            mut cr4,
            out: (mut out1, mut out2),
            ..
        } = dp
            .HRTIM_TIMA
            .pwm_advanced((pin_a, pin_b))
            .prescaler(prescaler)
            .period(period)
            .finalize(&mut hr_control);

        cr1.set_duty(period / 2);
        cr3.set_duty(period / 3);
        cr4.set_duty((2 * u32::from(period) / 3) as u16);

        hr_control.adc_trigger1.enable_source(&cr3);
        hr_control.adc_trigger1.enable_source(&cr4);

        out1.enable_rst_event(&cr1); // Set low on compare match with cr1
        out2.enable_rst_event(&cr1);

        out1.enable_set_event(&timer); // Set high at new period
        out2.enable_set_event(&timer);

        let pa1 = gpioa.pa1.into_analog();
        let pa2 = gpioa.pa2.into_analog();
        let pa4 = gpioa.pa4.into_floating_input();

        let dac1ch1 = dp.DAC1.constrain(pa4, &mut rcc);
        let dac3ch1 = dp.DAC3.constrain(dac::Dac3IntSig1, &mut rcc);

        let mut value_dac = dac1ch1.calibrate_buffer(&mut delay).enable(&mut rcc);

        defmt::info!("Setup Adc1");
        let mut adc = dp
            .ADC1
            .claim(ClockSource::SystemClock, &rcc, &mut delay, true);

        adc.set_external_trigger((
            adc::config::TriggerMode::RisingEdge,
            (&hr_control.adc_trigger1).into(),
        ));
        adc.enable_temperature(&dp.ADC12_COMMON);
        adc.set_continuous(adc::config::Continuous::Discontinuous);
        adc.reset_sequence();
        adc.configure_channel(
            &pa0,
            adc::config::Sequence::One,
            adc::config::SampleTime::Cycles_640_5,
        );
        adc.configure_channel(
            &Temperature,
            adc::config::Sequence::Two,
            adc::config::SampleTime::Cycles_640_5,
        );

        defmt::info!("Setup DMA");
        let first_buffer = cortex_m::singleton!(: [u16; 10] = [0; 10]).unwrap();

        let mut transfer = dma1ch1.into_circ_peripheral_to_memory_transfer(
            adc.enable_dma(adc::config::Dma::Continuous),
            &mut first_buffer[..],
            config,
        );

        transfer.start(|adc| adc.start_conversion());

        out1.enable();
        out2.enable();

        timer.start(&mut hr_control.control);

        loop {
            let mut b = [0_u16; 4];
            let r = transfer.read_exact(&mut b);

            defmt::info!("read: {}", r);
            assert!(r == b.len());

            let millivolts = Vref::sample_to_millivolts((b[0] + b[2]) / 2);
            defmt::info!("pa3: {}mV", millivolts);
            let temp = Temperature::temperature_to_degrees_centigrade(
                (b[1] + b[3]) / 2,
                VREF,
                adc::config::Resolution::Twelve,
            );
            defmt::info!("temp: {}℃C", temp);
        }
    }
}
