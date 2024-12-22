use crate::Sealed;
use core::marker::PhantomData;

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
/// See [`Observed`] and [`ObservationToken`]
pub trait Observable: Sized {
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
