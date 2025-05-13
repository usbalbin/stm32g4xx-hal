//! Analog to digital converter configuration.
//! <https://github.com/stm32-rs/stm32l4xx-hal/blob/master/src/adc.rs>

#![deny(missing_docs)]

mod g4;

/// Holds config related types for setting up the ADC's
pub mod config;

/// Temperature related types used for reading the device's internal temperature
pub mod temperature;

use crate::stm32::rcc::ccipr;
pub use crate::time::U32Ext as _;
use crate::{
    dma::{mux::DmaMuxResources, traits::TargetAddress, PeripheralToMemory},
    rcc::Rcc,
    signature::VDDA_CALIB,
    stm32,
};
use crate::{rcc, stasis};
use core::marker::PhantomData;
use core::{fmt, ops::Deref};
use embedded_hal::delay::DelayNs;
use embedded_hal_old::adc::{Channel, OneShot};

/// Extension trait for `ADC12_COMMON` and `ADC345_COMMON`
///
/// These types hold some configuration which is common to all
/// ADCs in that group such as clock configuration.
pub trait AdcCommonExt:
    Deref<Target = stm32::adc12_common::RegisterBlock> + Sized + rcc::Enable + rcc::Reset
{
    /// Pointer to the underlaying register block
    const PTR: *const stm32::adc12_common::RegisterBlock;

    /// Setup and initialize the adc group
    fn claim(self, cfg: config::ClockMode, rcc: &mut Rcc) -> AdcCommon<Self>;
}

fn init_adc_common<ADCC: AdcCommonExt>(
    adc_common: ADCC,
    rcc: &mut rcc::Rcc,
    cfg: config::ClockMode,
    adcsel: impl FnOnce(
        &mut stm32g4::raw::W<stm32::rcc::ccipr::CCIPRrs>,
    ) -> stm32::rcc::ccipr::ADC12SEL_W<stm32::rcc::ccipr::CCIPRrs>,
) -> AdcCommon<ADCC> {
    let cfg = cfg.to_bits(rcc);

    unsafe {
        let rcc_ptr = &(*stm32::RCC::ptr());
        ADCC::enable(rcc_ptr);
        ADCC::reset(rcc_ptr);
    }

    // Select system clock as ADC clock source
    rcc.rb
        .ccipr()
        .modify(|_, w: &mut stm32g4::raw::W<ccipr::CCIPRrs>| {
            // This is sound, as `0b10` is a valid value for this field.
            unsafe {
                adcsel(w).bits(cfg.adcsel);
            }

            w
        });

    adc_common
        .ccr()
        .modify(|_, w| unsafe { w.ckmode().bits(cfg.ckmode) });
    adc_common
        .ccr()
        .modify(|_, w| unsafe { w.presc().bits(cfg.presc) });

    AdcCommon { reg: adc_common }
}

impl AdcCommonExt for stm32::ADC12_COMMON {
    const PTR: *const stm32::adc12_common::RegisterBlock = Self::ptr();

    fn claim(self, cfg: config::ClockMode, rcc: &mut Rcc) -> AdcCommon<Self> {
        init_adc_common(self, rcc, cfg, |w| w.adc12sel())
    }
}

#[cfg(feature = "adc3")]
impl AdcCommonExt for stm32::ADC345_COMMON {
    const PTR: *const stm32::adc12_common::RegisterBlock = Self::ptr();

    fn claim(self, cfg: config::ClockMode, rcc: &mut Rcc) -> AdcCommon<Self> {
        init_adc_common(self, rcc, cfg, |w| w.adc345sel())
    }
}

/// Type for initialized `ADC12_COMMON` or `ADC345_COMMON`
///
/// See [`AdcCommon::claim`]
pub struct AdcCommon<ADCC> {
    reg: ADCC,
}

/// Marker trait for all ADC peripherals
pub trait Instance: crate::Sealed + Deref<Target = stm32::adc1::RegisterBlock> {
    /// Specifies adc common register block for configuration common to this and other ADC in the same group
    type Common: AdcCommonExt;

    /// Specifies what External trigger type the ADC uses
    type ExternalTrigger: fmt::Debug + Default + Copy + Into<u8>;

    /// Specifies what dma mux resource this ADC is
    const DMA_MUX_RESOURCE: DmaMuxResources;
}

impl Instance for stm32::ADC1 {
    type Common = stm32::ADC12_COMMON;
    type ExternalTrigger = config::ExternalTrigger12;
    const DMA_MUX_RESOURCE: DmaMuxResources = DmaMuxResources::ADC1;
}
impl Instance for stm32::ADC2 {
    type Common = stm32::ADC12_COMMON;
    type ExternalTrigger = config::ExternalTrigger12;
    const DMA_MUX_RESOURCE: DmaMuxResources = DmaMuxResources::ADC2;
}
#[cfg(feature = "adc3")]
impl Instance for stm32::ADC3 {
    type Common = stm32::ADC345_COMMON;
    type ExternalTrigger = config::ExternalTrigger345;
    const DMA_MUX_RESOURCE: DmaMuxResources = DmaMuxResources::ADC3;
}
#[cfg(feature = "adc4")]
impl Instance for stm32::ADC4 {
    type Common = stm32::ADC345_COMMON;
    type ExternalTrigger = config::ExternalTrigger345;
    const DMA_MUX_RESOURCE: DmaMuxResources = DmaMuxResources::ADC4;
}

#[cfg(feature = "adc5")]
impl Instance for stm32::ADC5 {
    type Common = stm32::ADC345_COMMON;
    type ExternalTrigger = config::ExternalTrigger345;
    const DMA_MUX_RESOURCE: DmaMuxResources = DmaMuxResources::ADC5;
}

/// Vref internal signal, used for calibration
pub struct Vref;
impl Vref {
    /// Converts a sample value to millivolts using calibrated VDDA and configured resolution
    #[inline(always)]
    pub fn sample_to_millivolts_ext(sample: u16, vdda: u32, resolution: config::Resolution) -> u16 {
        let mx_s = resolution.to_max_sample();
        ((u32::from(sample) * vdda) / mx_s) as u16
    }
    /// Converts a sample value to millivolts using calibrated VDDA and configured resolution
    #[inline(always)]
    pub fn sample_to_millivolts(sample: u16) -> u16 {
        Self::sample_to_millivolts_ext(sample, VDDA_CALIB, config::Resolution::Twelve)
    }
}
impl stasis::Freeze for Vref {}

/// Vbat internal signal, used for monitoring the battery (if used)
pub struct Vbat;
impl stasis::Freeze for Vbat {}

// TODO: Is there any way to avoid this wrapper to overcome the orphan rule
/// Wrapper to side step orphan rule
pub struct Ad<T>(T);
macro_rules! adc_channel_helper {
    ($adc:ident, $chan:expr, $r:ty, $($a:ident),*) => {
        impl<$($a,)*> embedded_hal_old::adc::Channel<super::Ad<crate::stm32::$adc>> for crate::stasis::Entitlement<$r> {
            type ID = u8;
            fn channel() -> u8 {
                $chan
            }
        }

        impl<const N: usize, $($a,)*> embedded_hal_old::adc::Channel<super::Ad<crate::stm32::$adc>> for crate::stasis::Frozen<$r, N> {
            type ID = u8;
            fn channel() -> u8 {
                $chan
            }
        }

        impl<$($a,)*> embedded_hal_old::adc::Channel<super::Ad<crate::stm32::$adc>> for $r {
            type ID = u8;
            fn channel() -> u8 {
                $chan
            }
        }
    };
}

use adc_channel_helper;

macro_rules! adc_pins {
    ($($pin:ty => ($adc:ident, $chan:expr)),+ $(,)*) => {
        $(
            adc_channel_helper!($adc, $chan, $pin,);
        )+
    };
}

use adc_pins;

macro_rules! adc_opamp {
    ($($opamp:ty => ($adc:ident, $chan:expr)),+ $(,)*) => {
        $(
            adc_channel_helper!($adc, $chan, opamp::Follower<$opamp, A, opamp::InternalOutput>, A);
            adc_channel_helper!($adc, $chan, opamp::OpenLoop<$opamp, A, B, opamp::InternalOutput>, A, B);
            adc_channel_helper!($adc, $chan, opamp::Pga<$opamp, A, opamp::InternalOutput>, A);
            adc_channel_helper!($adc, $chan, opamp::Locked<$opamp, opamp::InternalOutput>,);
        )+
    };
}

use adc_opamp;

/// Type-State for Adc, indicating a deep-powered-down-pheripheral
#[derive(Debug)]
pub struct PoweredDown;
/// Type-State for Adc, indicating a non-configured peripheral
#[derive(Debug)]
pub struct Disabled;
/// Type-State for Adc, indicating a configured peripheral
#[derive(Debug)]
pub struct Configured;
/// Type-State for Adc, indicating an peripheral configured for DMA
#[derive(Debug)]
pub struct DMA;
/// Type-State for Adc, indicating am active measuring peripheral
#[derive(Debug)]
pub struct Active;

/// Enum for the wait_for_conversion_sequence function,
/// which can return either a stopped ADC typestate or a
/// continuing ADC typestate.
pub enum Conversion<ADC: Instance> {
    /// Contains an Active Conversion ADC
    Active(Adc<ADC, Active>),
    /// Contains an Stopped ADC
    Stopped(Adc<ADC, Configured>),
}
impl<ADC: Instance> Conversion<ADC> {
    /// unwraps the enum and panics if the result is not an Active ADC
    #[inline(always)]
    pub fn unwrap_active(self) -> Adc<ADC, Active> {
        match self {
            Conversion::Active(adc) => adc,
            _ => {
                panic!("Conversion is Stopped, not Active!")
            }
        }
    }

    /// unwraps the enum and panics if the result is not a Stopped ADC
    #[inline(always)]
    pub fn unwrap_stopped(self) -> Adc<ADC, Configured> {
        match self {
            Conversion::Stopped(adc) => adc,
            _ => {
                panic!("Conversion is Continuing, not Stopped!")
            }
        }
    }

    /// Returns true if the adc is Active.
    #[inline(always)]
    pub fn is_active(&self) -> bool {
        match *self {
            Conversion::Active(..) => true,
            Conversion::Stopped(..) => false,
        }
    }

    /// Returns true if the adc is Stopped.
    #[inline(always)]
    pub fn is_stopped(&self) -> bool {
        !self.is_active()
    }

    /// Converts from `Conversion<C, E>` to `Option<C>`.
    ///
    /// Converts self into an `Option<C>`, consuming self, and discarding the adc, if it is stopped.
    #[inline(always)]
    pub fn active(self) -> Option<Adc<ADC, Active>> {
        match self {
            Conversion::Active(adc) => Some(adc),
            Conversion::Stopped(..) => None,
        }
    }

    /// Converts from `Conversion<C, E>` to `Option<E>`.
    ///
    /// Converts self into an `Option<E>`, consuming self, and discarding the adc, if it is still active.
    #[inline(always)]
    pub fn stopped(self) -> Option<Adc<ADC, Configured>> {
        match self {
            Conversion::Active(..) => None,
            Conversion::Stopped(adc) => Some(adc),
        }
    }
}

/// Analog to Digital Converter
/// # Status
/// Most options relating to regular conversions are implemented. One-shot and sequences of conversions
/// have been tested and work as expected.
///
/// GPIO to channel mapping should be correct for all supported F4 devices. The mappings were taken from
/// CubeMX. The mappings are feature gated per 4xx device but there are actually sub variants for some
/// devices and some pins may be missing on some variants. The implementation has been split up and commented
/// to show which pins are available on certain device variants but currently the library doesn't enforce this.
/// To fully support the right pins would require 10+ more features for the various variants.
/// ## Todo
/// * Injected conversions
/// * Analog watchdog config
/// * Discontinuous mode
/// # Examples
/// ## One-shot conversion
/// ```
/// use stm32f4xx_hal::{
///   gpio::gpioa,
///   adc::{
///     Adc,
///     config::AdcConfig,
///     config::SampleTime,
///   },
/// };
///
/// let mut adc = Adc::adc1(device.ADC1, true, AdcConfig::default());
/// let pa3 = gpioa.pa3.into_analog();
/// let sample = adc.convert(&pa3, SampleTime::Cycles_480);
/// let millivolts = adc.sample_to_millivolts(sample);
/// info!("pa3: {}mV", millivolts);
/// ```
///
/// ## Sequence conversion
/// ```
/// use stm32f4xx_hal::{
///   gpio::gpioa,
///   adc::{
///     Adc,
///     config::AdcConfig,
///     config::SampleTime,
///     config::Sequence,
///     config::Eoc,
///     config::Clock,
///   },
/// };
///
/// let config = AdcConfig::default()
///     //We'll either need DMA or an interrupt per conversion to convert
///     //multiple values in a sequence
///     .end_of_conversion_interrupt(Eoc::Conversion)
///     //And since we're looking for one interrupt per conversion the
///     //clock will need to be fairly slow to avoid overruns breaking
///     //the sequence. If you are running in debug mode and logging in
///     //the interrupt, good luck... try setting pclk2 really low.
///     //(Better yet use DMA)
///     .clock(Clock::Pclk2_div_8);
/// let mut adc = Adc::adc1(device.ADC1, true, config);
/// let pa0 = gpioa.pa0.into_analog();
/// let pa3 = gpioa.pa3.into_analog();
/// adc.configure_channel(&pa0, Sequence::One, SampleTime::Cycles_112);
/// adc.configure_channel(&pa3, Sequence::Two, SampleTime::Cycles_480);
/// adc.configure_channel(&pa0, Sequence::Three, SampleTime::Cycles_112);
/// adc.start_conversion();
/// ```
///
/// ## External trigger
///
/// A common mistake on STM forums is enabling continuous mode but that causes it to start
/// capturing on the first trigger and capture as fast as possible forever, regardless of
/// future triggers. Continuous mode is disabled by default but I thought it was worth
/// highlighting.
///
/// Getting the timer config right to make sure it's sending the event the ADC is listening
/// to can be a bit of a pain but the key fields are highlighted below. Try hooking a timer
/// channel up to an external pin with an LED or oscilloscope attached to check it's really
/// generating pulses if the ADC doesn't seem to be triggering.
/// ```
/// use stm32f4xx_hal::{
///   gpio::gpioa,
///   adc::{
///     Adc,
///     config::AdcConfig,
///     config::SampleTime,
///     config::Sequence,
///     config::Eoc,
///     config::Clock,
///   },
/// };
///
///  let config = AdcConfig::default()
///      //Set the trigger you want
///      .external_trigger(TriggerMode::RisingEdge, ExternalTrigger::Tim_1_cc_1);
///  let mut adc = Adc::adc1(device.ADC1, true, config);
///  let pa0 = gpioa.pa0.into_analog();
///  adc.configure_channel(&pa0, Sequence::One, SampleTime::Cycles_112);
///  //Make sure it's enabled but don't start the conversion
///  adc.enable();
///
/// //Configure the timer
/// let mut tim = Timer::tim1(device.TIM1, 1.hz(), clocks);
/// unsafe {
///     let tim = &(*TIM1::ptr());
///
///     //Channel 1
///     //Disable the channel before configuring it
///     tim.ccer().modify(|_, w| w.cc1e().clear_bit());
///
///     tim.ccmr1_output().modify(|_, w| w
///       //Preload enable for channel
///       .oc1pe().set_bit()
///
///       //Set mode for channel, the default mode is "frozen" which won't work
///       .oc1m().pwm_mode1()
///     );
///
///     //Set the duty cycle, 0 won't work in pwm mode but might be ok in
///     //toggle mode or match mode
///     let max_duty = tim.arr().read().arr().bits() as u16;
///     tim.ccr1().modify(|_, w| w.ccr().bits(max_duty / 2));
///
///     //Enable the channel
///     tim.ccer().modify(|_, w| w.cc1e().set_bit());
///
///     //Enable the TIM main Output
///     tim.bdtr().modify(|_, w| w.moe().set_bit());
/// }
/// ```
pub struct Adc<ADC: Instance, STATUS> {
    adc_reg: ADC,
    _status: PhantomData<STATUS>,
}
/*impl<ADC: Instance, STATUS> fmt::Debug for Adc<ADC, STATUS>
where
    STATUS: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "Adc<{:?}>: {{ calibrated_vdda: {:?}, {:?}, ... }}",
            self._status,
        )
    }
}*/

/// used to create an ADC instance from the stm32::Adc
pub trait AdcClaim<ADC: Instance> {
    /// create a disabled ADC instance from the stm32::Adc
    fn claim(&self, adc: ADC, delay: &mut impl DelayNs) -> Adc<ADC, Disabled>;

    /// create an enabled ADC instance from the stm32::Adc
    fn claim_and_configure(
        &self,
        adc: ADC,
        config: config::AdcConfig<ADC::ExternalTrigger>,
        delay: &mut impl DelayNs,
    ) -> Adc<ADC, Configured>;
}

impl<ADC: Instance> AdcClaim<ADC> for AdcCommon<ADC::Common> {
    /// Runs calibration and applies the supplied config
    /// # Arguments
    ///
    /// TODO: fix needing SYST
    #[inline(always)]
    fn claim(&self, adc: ADC, delay: &mut impl DelayNs) -> Adc<ADC, Disabled> {
        let adc: Adc<ADC, PoweredDown> = Adc {
            adc_reg: adc,
            _status: PhantomData,
        };

        adc.power_up(delay)
    }

    /// claims and configures the Adc
    #[inline(always)]
    fn claim_and_configure(
        &self,
        adc: ADC,
        config: config::AdcConfig<ADC::ExternalTrigger>,
        delay: &mut impl DelayNs,
    ) -> Adc<ADC, Configured> {
        let mut adc = self.claim(adc, delay);
        adc.adc.config = config;

        // If the user specified a VDDA, use that over the internally determined value.
        if let Some(vdda) = config.vdda {
            adc.adc.calibrated_vdda = vdda;
        }

        adc.enable()
    }
}

impl<ADCC: AdcCommonExt> AdcCommon<ADCC> {
    /// Enables the vbat internal channel
    #[inline(always)]
    pub fn enable_vbat(&mut self) {
        self.reg.ccr().modify(|_, w| w.vbatsel().set_bit());
    }

    /// Enables the vbat internal channel
    #[inline(always)]
    pub fn disable_vbat(&mut self) {
        self.reg.ccr().modify(|_, w| w.vbatsel().clear_bit());
    }

    /// Returns if the vbat internal channel is enabled
    #[inline(always)]
    pub fn is_vbat_enabled(&mut self) -> bool {
        self.reg.ccr().read().vbatsel().bit_is_set()
    }

    /// Enables the temp internal channel.
    #[inline(always)]
    pub fn enable_temperature(&mut self) {
        self.reg.ccr().modify(|_, w| w.vsensesel().set_bit());
    }

    /// Disables the temp internal channel
    #[inline(always)]
    pub fn disable_temperature(&mut self) {
        self.reg.ccr().modify(|_, w| w.vsensesel().clear_bit());
    }

    /// Returns if the temp internal channel is enabled
    #[inline(always)]
    pub fn is_temperature_enabled(&mut self) -> bool {
        self.reg.ccr().read().vsensesel().bit_is_set()
    }

    /// Enables the vref internal channel.
    #[inline(always)]
    pub fn enable_vref(&mut self) {
        self.reg.ccr().modify(|_, w| w.vrefen().set_bit());
    }

    /// Disables the vref internal channel
    #[inline(always)]
    pub fn disable_vref(&mut self) {
        self.reg.ccr().modify(|_, w| w.vrefen().clear_bit());
    }

    /// Returns if the vref internal channel is enabled
    #[inline(always)]
    pub fn is_vref_enabled(&mut self) -> bool {
        self.reg.ccr().read().vrefen().bit_is_set()
    }
}

impl<ADC: Instance, MODE> Adc<ADC, MODE> {
    #[inline(always)]
    fn start_ad_conversion(&mut self) {
        //Start conversion
        self.adc_reg.cr().modify(|_, w| w.adstart().set_bit());
    }

    /// Cancels an ongoing conversion
    #[inline(always)]
    fn cancel_ad_conversion(&mut self) {
        self.adc_reg.cr().modify(|_, w| w.adstp().set_bit());
        while self.adc_reg.cr().read().adstart().bit_is_set() {}
    }

    fn cancel_and_disable(&mut self) {
        // Disable any ongoing conversions
        self.cancel_ad_conversion();

        // Turn off ADC
        self.adc_reg.cr().modify(|_, w| w.addis().set_bit());
        while self.adc_reg.cr().read().addis().bit_is_set() {}

        // Wait until the ADC has turned off
        while self.adc_reg.cr().read().aden().bit_is_set() {}
    }

    fn disable_vreg(&mut self) {
        self.adc_reg.cr().modify(|_, w| w.advregen().clear_bit());
    }
}

impl<ADC: Instance> Adc<ADC, PoweredDown> {
    fn apply_config(&mut self, config: config::AdcConfig<ADC::ExternalTrigger>) {
        self.set_resolution(config.resolution);
        self.set_align(config.align);
        self.set_external_trigger(config.external_trigger);
        self.set_continuous(config.continuous);
        self.set_subgroup_len(config.subgroup_len);
        self.set_dma(config.dma);
        self.set_end_of_conversion_interrupt(config.end_of_conversion_interrupt);
        self.set_overrun_interrupt(config.overrun_interrupt);
        self.set_default_sample_time(config.default_sample_time);
        self.set_channel_input_type(config.difsel);
        self.set_auto_delay(config.auto_delay);
    }

    fn set_resolution(&mut self, resolution: config::Resolution) {
        unsafe {
            self.adc_reg
                .cfgr()
                .modify(|_, w| w.res().bits(resolution.into()));
        }
    }

    /// Powers-up an powered-down Adc
    #[inline(always)]
    pub fn power_up(mut self, delay: &mut impl DelayNs) -> Adc<ADC, Disabled> {
        self.adc.power_up(delay);

        Adc {
            adc: self.adc,
            _status: PhantomData,
        }
    }

    /// Puts a Disabled Adc into Powered Mode
    #[inline(always)]
    pub fn power_down(mut adc: Adc<ADC, Disabled>) -> Self {
        adc.adc.power_down();

        Adc {
            adc: adc.adc,
            _status: PhantomData,
        }
    }

    /// Disables the Voltage Regulator and release the ADC
    #[inline(always)]
    pub fn release(self) -> ADC {
        self.adc.release()
    }

    /// Enables the Deep Power Down Modus
    #[inline(always)]
    pub fn enable_deeppwd_down(&mut self) {
        self.adc.enable_deeppwd_down()
    }

    /// Disables the Deep Power Down Modus
    #[inline(always)]
    pub fn disable_deeppwd_down(&mut self) {
        self.adc.disable_deeppwd_down()
    }
}

impl<ADC: Instance> Adc<ADC, Disabled> {
    //adc!(additionals: $adc_type => ($common_type));
    //adc!(additionals_checks: $adc_type => ($common_type));

    /// Enables the adc
    #[inline(always)]
    pub fn enable(mut self) -> Adc<ADC, Configured> {
        self.adc.enable();

        Adc {
            adc: self.adc,
            _status: PhantomData,
        }
    }

    /// Enables the adc
    #[inline(always)]
    pub fn configure_and_enable(
        mut self,
        config: config::AdcConfig<ADC::ExternalTrigger>,
    ) -> Adc<ADC, Configured> {
        self.adc.apply_config(config);
        self.enable()
    }

    /// enable the adc and configure for DMA.
    /// panics if set to Dma::Disabled
    #[inline(always)]
    pub fn enable_dma(mut self, dma: config::Dma) -> Adc<ADC, DMA> {
        let dds = match dma {
            config::Dma::Single => false,
            config::Dma::Continuous => true,
        };
        self.adc_reg.cfgr().modify(|_, w| {
            w
                //DDS stands for "DMA disable selection"
                //0 means do one DMA then stop
                //1 means keep sending DMA requests as long as DMA=1
                .dmacfg()
                .bit(dds)
                .dmaen()
                .enabled()
        });

        Adc {
            adc_reg: self.adc_reg,
            _status: PhantomData,
        }
    }

    /// Puts a disabled Adc into PoweredDown Mode
    #[inline(always)]
    pub fn power_down(mut self) -> Adc<ADC, PoweredDown> {
        self.disable_vreg();

        Adc {
            adc_reg: self.adc_reg,
            _status: PhantomData,
        }
    }

    /// Sets the oversampling
    #[inline(always)]
    pub fn set_oversampling(
        &mut self,
        oversampling: config::OverSampling,
        shift: config::OverSamplingShift,
    ) {
        self.adc_reg.cfgr2().modify(|_, w| unsafe {
            w.ovsr()
                .bits(oversampling.into())
                .ovss()
                .bits(shift.into())
                .rovse()
                .set_bit()
        });
    }

    /// Sets the sampling resolution
    #[inline(always)]
    pub fn set_resolution(&mut self, resolution: config::Resolution) {
        self.adc.set_resolution(resolution)
    }

    /// Sets the DR register alignment to left or right
    #[inline(always)]
    pub fn set_align(&mut self, align: config::Align) {
        self.adc_reg
            .cfgr()
            .modify(|_, w| w.align().bit(align.into()));
    }

    /// Sets which external trigger to use and if it is disabled, rising, falling or both
    #[inline(always)]
    pub fn set_external_trigger(
        &mut self,
        (edge, extsel): (config::TriggerMode, ADC::ExternalTrigger),
    ) {
        self.adc_reg
            .cfgr()
            .modify(|_, w| unsafe { w.extsel().bits(extsel.into()).exten().bits(edge.into()) });
    }

    /// Sets auto delay to true or false
    #[inline(always)]
    pub fn set_auto_delay(&mut self, delay: bool) {
        self.adc_reg.cfgr().modify(|_, w| w.autdly().bit(delay));
    }

    /// Enables and disables continuous mode
    #[inline(always)]
    pub fn set_continuous(&mut self, continuous: config::Continuous) {
        self.adc_reg.cfgr().modify(|_, w| {
            w.cont()
                .bit(continuous == config::Continuous::Continuous)
                .discen()
                .bit(continuous == config::Continuous::Discontinuous)
        });
    }

    /// Set subgroup length, number of AD readings per trigger event (only relevant in Discontinuous mode)
    pub fn set_subgroup_len(&mut self, subgroup_len: config::SubGroupLength) {
        unsafe {
            self.adc_reg
                .cfgr()
                .modify(|_, w| w.discnum().bits(subgroup_len as u8));
        }
    }

    /// Sets if the end-of-conversion behaviour.
    /// The end-of-conversion interrupt occur either per conversion or for the whole sequence.
    #[inline(always)]
    pub fn set_end_of_conversion_interrupt(&mut self, eoc: config::Eoc) {
        self.config.end_of_conversion_interrupt = eoc;
        let (en, eocs) = match eoc {
            config::Eoc::Disabled => (false, false),
            config::Eoc::Conversion => (true, true),
            config::Eoc::Sequence => (true, false),
        };
        self.adc_reg
            .ier()
            .modify(|_, w| w.eosie().bit(eocs).eocie().bit(en));
    }

    /// Enable/disable overrun interrupt
    ///
    /// This is triggered when the AD finishes a conversion before the last value was read by CPU/DMA
    #[inline(always)]
    pub fn set_overrun_interrupt(&mut self, enable: bool) {
        self.adc_reg.ier().modify(|_, w| w.ovrie().bit(enable));
    }

    /// Sets the differential selection per channel.
    #[inline(always)]
    pub fn set_channel_input_type(&mut self, df: config::DifferentialSelection) {
        self.adc_reg.difsel().modify(|_, w| {
            for i in 0..19 {
                w.difsel(i).bit(df.get_channel(i).into());
            }
            w
        });
    }

    /// Reset the sequence
    #[inline(always)]
    pub fn reset_sequence(&mut self) {
        //The reset state is One conversion selected
        self.adc_reg
            .sqr1()
            .modify(|_, w| unsafe { w.l().bits(config::Sequence::One.into()) });
    }

    /// Returns the current sequence length. Primarily useful for configuring DMA.
    #[inline(always)]
    pub fn sequence_length(&mut self) -> u8 {
        self.adc_reg.sqr1().read().l().bits() + 1
    }

    /// Calibrate the adc for <Input Type>
    #[inline(always)]
    pub fn calibrate(&mut self, it: config::InputType) {
        match it {
            config::InputType::SingleEnded => {
                self.adc_reg.cr().modify(|_, w| w.adcaldif().clear_bit());
            }
            config::InputType::Differential => {
                self.adc_reg.cr().modify(|_, w| w.adcaldif().set_bit());
            }
        }

        self.adc_reg.cr().modify(|_, w| w.adcal().set_bit());
        while self.adc_reg.cr().read().adcal().bit_is_set() {}
    }

    /// Calibrate the Adc for all Input Types
    #[inline(always)]
    pub fn calibrate_all(&mut self) {
        self.calibrate(config::InputType::Differential);
        self.calibrate(config::InputType::SingleEnded);
    }

    /// Configure a channel for sampling.
    /// It will make sure the sequence is at least as long as the `sequence` provided.
    /// # Arguments
    /// * `channel` - channel to configure
    /// * `sequence` - where in the sequence to sample the channel. Also called rank in some STM docs/code
    /// * `sample_time` - how long to sample for. See datasheet and ref manual to work out how long you need\
    ///   to sample for at a given ADC clock frequency
    #[inline(always)]
    pub fn configure_channel<CHANNEL>(
        &mut self,
        channel: &CHANNEL,
        sequence: config::Sequence,
        sample_time: config::SampleTime,
    ) where
        CHANNEL: Channel<Ad<ADC>, ID = u8>,
    {
        self.adc.configure_channel(channel, sequence, sample_time)
    }

    /// Synchronously convert a single sample
    /// Note that it reconfigures the adc sequence and doesn't restore it
    #[inline(always)]
    pub fn convert<PIN>(&mut self, pin: &PIN, sample_time: config::SampleTime) -> u16
    where
        PIN: Channel<Ad<ADC>, ID = u8>,
    {
        self.adc.convert(pin, sample_time)
    }
}

impl<ADC: Instance> Adc<ADC, Configured> {
    //adc!(additionals_checks: $adc_type => ($common_type));

    /// Disables the adc
    #[inline(always)]
    pub fn disable(mut self) -> Adc<ADC, Disabled> {
        self.adc.disable();

        Adc {
            adc: self.adc,
            _status: PhantomData,
        }
    }

    /// Starts conversion sequence. Waits for the hardware to indicate it's actually started.
    #[inline(always)]
    pub fn start_conversion(mut self) -> Adc<ADC, Active> {
        self.adc.clear_end_of_conversion_flag();
        self.adc.start_conversion();

        Adc {
            adc: self.adc,
            _status: PhantomData,
        }
    }

    /// Returns the current sample stored in the ADC data register
    #[inline(always)]
    pub fn current_sample(&self) -> u16 {
        self.adc.current_sample()
    }

    /// Synchronously convert a single sample
    /// Note that it reconfigures the adc sequence and doesn't restore it
    #[inline(always)]
    pub fn convert<PIN>(&mut self, pin: &PIN, sample_time: config::SampleTime) -> u16
    where
        PIN: Channel<Ad<ADC>, ID = u8>,
    {
        self.adc.reset_sequence();
        self.adc
            .configure_channel(pin, config::Sequence::One, sample_time);
        self.adc.start_conversion();

        //Wait for the sequence to complete
        self.adc.wait_for_conversion_sequence();

        self.adc.current_sample()
    }
}

impl<ADC: Instance> Conversion<ADC> {
    /// Wait in a potential infite loop untill the ADC has stopped the conversion.
    /// Everytime an sample is retrieved 'func' is called.
    /// Note: when the ADC has stopped the conversion, for the last sample, func is NOT run.
    pub fn wait_untill_stopped<F>(mut self, mut func: F) -> Adc<ADC, Configured>
    where
        F: FnMut(u16, &Adc<ADC, Active>),
    {
        loop {
            match self {
                Conversion::Stopped(adc) => return adc,
                Conversion::Active(adc) => {
                    self = adc.wait_for_conversion_sequence();
                    if let Conversion::Active(adc) = self {
                        let sample = adc.current_sample();
                        func(sample, &adc);

                        self = Conversion::Active(adc);
                    }
                }
            }
        }
    }
}

impl<ADC: Instance> Adc<ADC, Active> {
    /// Block until the conversion is completed and return to configured
    pub fn wait_for_conversion_sequence(mut self) -> Conversion<ADC> {
        self.adc.wait_for_conversion_sequence();

        if !self.adc.is_conversion_active() {
            let inactive: Adc<_, Configured> = Adc {
                adc: self.adc,
                _status: PhantomData,
            };

            Conversion::Stopped(inactive)
        } else {
            Conversion::Active(self)
        }
    }

    /// Returns if a conversion has been completed
    /// Calling this before `wait_for_conversion_sequence`
    /// should make that function return immediatly
    pub fn is_conversion_done(&self) -> bool {
        !self.adc.is_conversion_active()
    }

    /// Cancels an ongoing conversion
    #[inline(always)]
    pub fn cancel_conversion(mut self) -> Adc<ADC, Configured> {
        self.adc.cancel_conversion();

        Adc {
            adc: self.adc,
            _status: PhantomData,
        }
    }

    /// get current sample
    #[inline(always)]
    pub fn current_sample(&self) -> u16 {
        self.adc.current_sample()
    }

    /// clear end conversion flag
    #[inline(always)]
    pub fn clear_end_conversion_flag(&mut self) {
        self.adc.clear_end_of_conversion_flag();
    }
}

impl<ADC: Instance> Adc<ADC, DMA> {
    /// Starts conversion sequence. Waits for the hardware to indicate it's actually started.
    #[inline(always)]
    pub fn start_conversion(&mut self) {
        self.start_ad_conversion()
    }

    /// Cancels an ongoing conversion
    #[inline(always)]
    pub fn cancel_conversion(&mut self) {
        self.cancel_ad_conversion()
    }

    /// Stop the Adc
    //#[inline(always)]
    //pub fn stop(&mut self) {
    //    self.cancel_and_disable();
    //}

    /// Disable the Adc
    #[inline(always)]
    pub fn disable(mut self) -> Adc<ADC, Disabled> {
        self.adc_reg.cfgr().modify(|_, w| w.dmaen().disabled());

        self.cancel_and_disable();

        Adc {
            adc_reg: self.adc_reg,
            _status: PhantomData,
        }
    }

    /// Read overrun flag
    #[inline(always)]
    pub fn get_overrun_flag(&self) -> bool {
        self.adc_reg.isr().read().ovr().bit()
    }

    /// Resets the overrun flag
    #[inline(always)]
    pub fn clear_overrun_flag(&mut self) {
        self.adc_reg.isr().modify(|_, w| w.ovr().clear());
    }
}

unsafe impl<ADC: Instance> TargetAddress<PeripheralToMemory> for Adc<ADC, DMA> {
    #[inline(always)]
    fn address(&self) -> u32 {
        self.adc_reg.dr().as_ptr() as u32
    }

    type MemSize = u16;

    const REQUEST_LINE: Option<u8> = Some(ADC::DMA_MUX_RESOURCE as u8);
}

impl<ADC: Instance, PIN> OneShot<Ad<ADC>, u16, PIN> for Adc<ADC, Disabled>
where
    PIN: Channel<Ad<ADC>, ID = u8>,
{
    type Error = ();

    fn read(&mut self, pin: &mut PIN) -> nb::Result<u16, Self::Error> {
        Ok(self.convert(pin, self.adc.config.default_sample_time))
    }
}
