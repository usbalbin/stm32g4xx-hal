#![no_std]
#![no_main]

use cortex_m_rt::entry;
use hal::adc::{self, config::ExternalTrigger12};

use crate::hal::{
    adc::{
        config::{Continuous, Dma as AdcDma, SampleTime, Sequence},
        AdcClaim, ClockSource, Temperature, Vref,
    },
    delay::SYSTDelayExt,
    dma::{config::DmaConfig, stream::DMAExt, TransferExt},
    gpio::GpioExt,
    rcc::{Config, RccExt},
    stm32::Peripherals,
};
use stm32g4xx_hal as hal;

use log::info;

#[macro_use]
mod utils;

#[entry]
fn main() -> ! {
    utils::logger::init();

    info!("start");

    let dp = Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

    info!("rcc");
    let rcc = dp.RCC.constrain();
    let mut rcc = rcc.freeze(Config::hsi());

    let streams = dp.DMA1.split(&rcc);
    let config = DmaConfig::default()
        .transfer_complete_interrupt(false)
        .circular_buffer(true)
        .memory_increment(true);

    info!("Setup Gpio");
    let gpioa = dp.GPIOA.split(&mut rcc);
    let pa0 = gpioa.pa0.into_analog();

    info!("Setup Adc1");
    let mut delay = cp.SYST.delay(&rcc.clocks);
    let mut adc = dp
        .ADC1
        .claim(ClockSource::SystemClock, &rcc, &mut delay, true);

    adc.set_external_trigger((
        adc::config::TriggerMode::RisingEdge,
        ExternalTrigger12::Hrtim_adc_trg_1,
    ));
    adc.enable_temperature(&dp.ADC12_COMMON);
    adc.set_continuous(Continuous::Discontinuous);
    adc.reset_sequence();
    adc.configure_channel(&pa0, Sequence::One, SampleTime::Cycles_640_5);
    adc.configure_channel(&Temperature, Sequence::Two, SampleTime::Cycles_640_5);

    info!("Setup DMA");
    let first_buffer = cortex_m::singleton!(: [u16; 10] = [0; 10]).unwrap();
    let mut transfer = streams.0.into_circ_peripheral_to_memory_transfer(
        adc.enable_dma(AdcDma::Continuous),
        &mut first_buffer[..],
        config,
    );

    transfer.start(|adc| adc.start_conversion());

    //        .               .
    //        .  50%          .
    //         ------          ------
    //out1    |      |        |      |
    //        |      |        |      |
    // --------      ----------      --------
    let (mut fault_control, _) = dp
        .HRTIM_COMMON
        .hr_control(&mut rcc)
        .set_adc_trigger1(Adc13Trigger::)
        .wait_for_calibration();
    let (mut timer, (mut cr1, _cr2, _cr3, _cr4), (mut out1, mut out2)) = dp
        .HRTIM_TIMA
        .pwm_advanced((pin_a, pin_b), &mut rcc)
        .prescaler(prescaler)
        .period(0xFFFF)
        // alternated every period with one being
        // inactive and the other getting to output its wave form
        // as normal
        .finalize(&mut fault_control);

    out1.enable_rst_event(EventSource::Cr1); // Set low on compare match with cr1
    out2.enable_rst_event(EventSource::Cr1);

    out1.enable_set_event(EventSource::Period); // Set high at new period
    out2.enable_set_event(EventSource::Period);

    out1.enable();
    out2.enable();

    loop {
        let mut b = [0_u16; 4];
        let r = transfer.read_exact(&mut b);

        info!("read: {}", r);
        assert!(r == b.len());

        let millivolts = Vref::sample_to_millivolts((b[0] + b[2]) / 2);
        info!("pa3: {}mV", millivolts);
        let temp = Temperature::temperature_to_degrees_centigrade((b[1] + b[3]) / 2);
        info!("temp: {}℃C", temp); // Note: Temperature seems quite low...
    }
}
