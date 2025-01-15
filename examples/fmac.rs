#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate stm32g4xx_hal as hal;

use hal::fmac::{
    FmacExt, Gain, IirConfig, I1F15
};
use hal::prelude::*;
use hal::pwr::PwrExt;
use hal::rcc::Config;
use hal::stm32;
use rt::entry;

#[macro_use]
mod utils;

use utils::logger::println;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = dp.RCC.freeze(Config::hsi(), pwr);
    
    const ZERO: I1F15 = I1F15::lit("0.0");
    const ONE: I1F15 = I1F15::lit("0.9999");
    
    const W_NEW: I1F15 = I1F15::lit("0.1");
    const W_OLD: I1F15 = ONE.saturating_sub(W_NEW);

    const X1: &[I1F15] = &[ZERO,ZERO];
    const B: &[I1F15] = &[W_NEW];
    const A: &[I1F15] = &[W_OLD];
    const Y: &[I1F15] = &[ZERO, ZERO];

    let cfg = const { 
        IirConfig::new(X1, B, A, Y, Gain::X1)
    };
    let mut iir = dp.FMAC.iir(cfg, &mut rcc);

    for i in 0..100 {
        let result = iir.compute_blocking(I1F15::from_num(1.0));
        println!("i: {} - {}", i, f32::from(result));
    }

    println!("done");

    #[allow(clippy::empty_loop)]
    loop {}
}
