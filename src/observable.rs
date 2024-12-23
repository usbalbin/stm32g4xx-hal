use crate::Sealed;
use core::{marker::PhantomData, ops};

pub trait ObservationLock: Sized + crate::Sealed {
    type Peripheral: Observable;
}

/// A struct to hold peripherals which are to be observed.
///
/// This prevents the observed peripheral from being consumed. Thus
/// preventing things like a an observed gpio pin changing mode or an opamp from
/// being disabled. This makes sure the underlaying peripheral will not
/// change mode into something that is not compatible with what ever may be observing it.
pub struct Observed<P: Observable, const OBSERVER_COUNT: usize> {
    peripheral: P,
}

impl<P: Observable, const OBSERVER_COUNT: usize> Observed<P, OBSERVER_COUNT> {
    /// Release the observation of this peripheral
    ///
    /// This returns the underlaying perpheral type. Since it is no longer
    /// observed, you are once again free to do what you want with it.
    pub fn release(self, _data: [ObservationToken<P>; OBSERVER_COUNT]) -> P {
        self.peripheral
    }
}

/// A struct to represent a registered observation of a peripheral of type `P`
///
/// The existence of this type guarantees that the observed peripheral will not
/// change mode into something that is not compatibe with what ever is observing it
pub struct ObservationToken<P: Observable> {
    _p: PhantomData<P>,
}

/// A trait providing an interface to make peripherals observed
///
/// See [`Observable::observe`], [`Observed`] and [`ObservationToken`]
pub trait Observable: Sized {
    /// Observe this peripheral to split it into a [`Observed<Self>`] and a set of [`ObservationToken`]'s
    ///
    /// This is useful when you need the same peripherals for multiple things at the same time.
    ///
    /// For example let's say you want to keep track of the voltage of a pin. You want to log it
    /// every second but if it rises above a threshold then you need to react really fast.
    ///
    /// This can be solved by connecting the pin to a comparator that compares the pins
    /// voltage to a reference. If the voltage rises above the reference then the comparator
    /// will quickly detect this and an interrupt can be generated or similar (not shown here).
    ///
    /// ```
    /// let dp = stm32::Peripherals::take().unwrap();
    /// let mut rcc = dp.RCC.constrain();
    ///
    /// let gpioa = dp.GPIOA.split(&mut rcc);
    ///
    /// let (comp1, comp2, ..) = dp.COMP.split(&mut rcc);
    ///
    /// let pa1 = gpioa.pa1.into_analog(); // <- The pin to keep track of
    /// let pa0 = gpioa.pa0.into_analog(); // <- Reference voltage
    ///
    /// // Pins consumed here
    /// let comp1 = comp1.comparator(pa1, pa0, Config::default(), &rcc.clocks);
    /// let comp1 = comp1.enable();
    ///
    /// // Can not use pa0 and pa1 for AD readings
    /// ```
    ///
    /// However we still want to perform AD readings every second. Since the pins are consumed
    /// by the comparator this is impossible.
    ///
    /// It turns ut that to construct the comparator we do not actually need a pin. We
    /// just need proof that there is a pin that is setup in the correct mode and which
    /// will stay in that mode as long as the comparator lives.
    ///
    /// This is where [`Observable::observe`] comes in. It splits the peripheral, in this case
    /// a pin, into an [`Observed<Self>`] and a set of [`ObservationToken`]'s. The `Observed`
    /// type can be used just like the peripheral would normally be used. For our pin we can
    /// use it to perform AD readings etc. There is however one vital difference, we can not
    /// reconfigure the observed peripheral. The `ObservationToken`'s on the other hand
    /// are tokens that proove that the peripheral will not be reconfigured. These can then
    /// be used instead of the peripheral to pass as arguments to other peripherals.
    ///
    /// ```
    /// let cp = cortex_m::Peripherals::take().unwrap();
    /// let dp = stm32::Peripherals::take().unwrap();
    /// let mut rcc = dp.RCC.constrain();
    ///
    /// let gpioa = dp.GPIOA.split(&mut rcc);
    ///
    /// let (comp1, ..) = dp.COMP.split(&mut rcc);
    ///
    /// let (pa1, [pa1_token]) = gpioa // <- The pin to keep track of
    ///     .pa1
    ///     .into_analog()
    ///     .observe();
    /// let pa0 = gpioa.pa0.into_analog(); // <- Reference voltage
    ///
    /// // Only pa1_token and pa0 consumed here
    /// let comp1 = comp1.comparator(pa1_token, pa0, Config::default(), &rcc.clocks);
    /// let _comp1 = comp1.enable(); // <-- TODO: Do things with comparator
    ///
    /// let mut delay = cp.SYST.delay(&rcc.clocks);
    /// let mut adc = dp.ADC1.claim_and_configure(
    ///     stm32g4xx_hal::adc::ClockSource::SystemClock,
    ///     &rcc,
    ///     stm32g4xx_hal::adc::config::AdcConfig::default(),
    ///     &mut delay,
    ///     false,
    /// );
    ///
    /// // Can not reconfigure pa1 here
    /// loop {
    ///     // Can still use pa1 here
    ///     let sample = adc.convert(pa1.as_ref(), SampleTime::Cycles_640_5);
    ///     defmt::info!("Reading: {}", sample);
    ///     delay.delay(1000.millis());
    /// }
    /// ```
    fn observe<const N: usize>(self) -> (Observed<Self, N>, [ObservationToken<Self>; N]) {
        (
            Observed { peripheral: self },
            core::array::from_fn(|_| ObservationToken { _p: PhantomData }),
        )
    }
}

impl<P: Observable + Sealed> ObservationLock for P {
    type Peripheral = P;
}

impl<P: Observable + Sealed> Sealed for ObservationToken<P> {}
impl<P: Observable + Sealed> ObservationLock for ObservationToken<P> {
    type Peripheral = P;
}

impl<P: Observable, const N: usize> AsRef<P> for Observed<P, N> {
    fn as_ref(&self) -> &P {
        &self.peripheral
    }
}

impl<P: Observable, const N: usize> AsMut<P> for Observed<P, N> {
    fn as_mut(&mut self) -> &mut P {
        &mut self.peripheral
    }
}

impl<P: Observable, const N: usize> ops::Deref for Observed<P, N> {
    type Target = P;

    fn deref(&self) -> &Self::Target {
        &self.peripheral
    }
}

impl<P: Observable, const N: usize> ops::DerefMut for Observed<P, N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.peripheral
    }
}
