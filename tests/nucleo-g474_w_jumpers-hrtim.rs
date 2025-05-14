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
    use stm32_hrtim::{compare_register::HrCompareRegister, output::HrOutput, timer::HrTimer, HrParts, HrPwmAdvExt, Pscl4};
    use stm32g4xx_hal::{adc::Vref, dac::SawtoothConfig, dma::{self, channel::DMAExt, config::DmaConfig, TransferExt}, hrtim::{HrControltExt, HrPwmBuilderExt}, pwr::PwrExt};


    #[test]
    fn hrtim() {
        use super::*;
        const VREF: f32 = 3.3;

        defmt::info!("start");

        let dp = stm32::Peripherals::take().unwrap();
        let cp = stm32::CorePeripherals::take().expect("cannot take core peripherals");

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

        let pa1 = gpioa.pa1.into_analog();
        let pa2 = gpioa.pa2.into_analog();
        let pa4 = gpioa.pa4.into_floating_input();

        let pin_a = gpioa.pa8;
        let pin_b = gpioa.pa9;

        // ...with a prescaler of 4 this gives us a HrTimer with a tick rate of 960MHz
        // With max the max period set, this would be 960MHz/2^16 ~= 15kHz...
        let prescaler = Pscl4;

        //        |\              |\              |
        //        |  \            |  \            |
        //        |    \          |    \          |
        //DAC out |      \        |      \        |
        //        |        \      |        \      |
        //        |          \    |          \    |
        //        |            \  |            \  |
        //        |              \|              \|
        //        .               .
        //        .  50%          .
        //         ------          ------
        //out1    |      |        |      |
        //        |      |        |      |
        // --------      ----------      ---------
        //             ^     ^    ^
        //             |     |    |
        //AD samlp    pa0   temp Vref

        let dac_step_size = 16;
        let dac_reset_value = 4000;

        let period = 0xFFFF;
        let (hr_control, ..) = dp.HRTIM_COMMON.hr_control(&mut rcc).wait_for_calibration();
        let mut hr_control = hr_control.constrain();
        let HrParts {
            mut timer,
            mut cr1,
            cr2: mut dac_trigger,
            cr3: mut adc_trigger1,
            cr4: mut adc_trigger2,
            out: (mut out1, mut out2),
            ..
        } = dp
            .HRTIM_TIMA
            .pwm_advanced((pin_a, pin_b))
            .prescaler(prescaler)
            .period(period)
            .finalize(&mut hr_control);

        cr1.set_duty(period / 2);
        adc_trigger1.set_duty(period / 3);
        adc_trigger2.set_duty((2 * u32::from(period) / 3) as u16);
        let adc_trigger3 = timer.as_period_adc_trigger();

        hr_control.adc_trigger1.enable_source(&adc_trigger1);
        hr_control.adc_trigger1.enable_source(&adc_trigger2);
        hr_control.adc_trigger1.enable_source(&adc_trigger3);

        out1.enable_rst_event(&cr1); // Set low on compare match with cr1
        out2.enable_rst_event(&cr1);

        out1.enable_set_event(&timer); // Set high at new period
        out2.enable_set_event(&timer);

        let dac1ch1 = dp.DAC1.constrain(pa4, &mut rcc);
        // dac_generator will have its value set automatically from its internal sawtooth generator
        let mut dac_generator = dac1ch1.enable_sawtooth_generator(
            SawtoothConfig::with_slope(dac::CountingDirection::Decrement, dac_step_size)
                .reset_value(dac_reset_value)
                .inc_trigger(&cr2)
                .reset_trigger(&timer),
            &mut rcc,
        );


        let mut value_dac = dac1ch1.calibrate_buffer(&mut delay).enable(&mut rcc);

        defmt::info!("Setup Adc1");
        let mut adc = dp
            .ADC1
            .claim(Default::default(), &rcc, &mut delay, true);

        adc.set_external_trigger((
            adc::config::TriggerMode::RisingEdge,
            (&hr_control.adc_trigger1).into(),
        ));
        adc.enable_temperature(&dp.ADC12_COMMON);
        adc.set_continuous(adc::config::Continuous::Discontinuous);
        adc.reset_sequence();
        adc.configure_channel(
            &pa1,
            adc::config::Sequence::One,
            adc::config::SampleTime::Cycles_640_5,
        );
        adc.configure_channel(
            &pa1,
            adc::config::Sequence::Two,
            adc::config::SampleTime::Cycles_640_5,
        );
        adc.configure_channel(
            &Vref,
            adc::config::Sequence::Three,
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
            let mut b = [0_u16; 3];
            let r = transfer.read_exact(&mut b);

            defmt::info!("read: {}", r);
            assert!(r == b.len());

            let [dac_smpl1, dac_smpl2, vref] = b;

            let millivolts = Vref::sample_to_millivolts_ext(vref, 3300, adc::config::Resolution::Twelve);
            defmt::info!("pa3: {}mV", millivolts);
        }
    }
}
