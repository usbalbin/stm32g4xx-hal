//! I2C
use embedded_hal::i2c::{ErrorKind, Operation, SevenBitAddress, TenBitAddress};
use embedded_hal_old::blocking::i2c::{Read, Write, WriteRead};

use crate::gpio::{gpioa::*, gpiob::*, gpioc::*, gpiof::*};
#[cfg(any(
    feature = "stm32g471",
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484"
))]
use crate::gpio::{gpiog::*, AF3};
use crate::gpio::{AlternateOD, AF2, AF4, AF8};
use crate::rcc::{Enable, GetBusFreq, Rcc, RccBus, Reset};
#[cfg(any(
    feature = "stm32g471",
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484"
))]
use crate::stm32::I2C4;
use crate::stm32::{I2C1, I2C2, I2C3, RCC};
use crate::time::Hertz;
use core::cmp;

/// I2C bus configuration.
pub struct Config {
    speed: Option<Hertz>,
    timing: Option<u32>,
    analog_filter: bool,
    digital_filter: u8,
}

impl Config {
    /// Creates a default configuration for the given bus frequency.
    pub fn new<T>(speed: T) -> Self
    where
        T: Into<Hertz>,
    {
        Config {
            speed: Some(speed.into()),
            timing: None,
            analog_filter: true,
            digital_filter: 0,
        }
    }

    /// Creates a default configuration with fully-customized timing.
    ///
    /// The `timing` parameter represents the value of the `I2C_TIMINGR` register:
    /// - Bits 31-28 contain a prescaler value by which the input clock to the I2C peripheral is
    ///   divided. The following fields are given as a multiple of the clock period generated by
    ///   this prescaler.
    /// - Bits 23-20 contain the data setup time.
    /// - Bits 19-16 contain the data hold time.
    /// - Bits 15-8 contain the SCL high period.
    /// - Bits 7-0 contain the SCL low period.
    pub fn with_timing(timing: u32) -> Self {
        Config {
            timing: Some(timing),
            speed: None,
            analog_filter: true,
            digital_filter: 0,
        }
    }

    /// Disables the analog noise filter.
    pub fn disable_analog_filter(mut self) -> Self {
        self.analog_filter = false;
        self
    }

    /// Enables the digital noise filter.
    pub fn enable_digital_filter(mut self, cycles: u8) -> Self {
        assert!(cycles <= 16);
        self.digital_filter = cycles;
        self
    }

    fn timing_bits(&self, i2c_clk: Hertz) -> u32 {
        if let Some(bits) = self.timing {
            return bits;
        }
        let speed = self.speed.unwrap();
        let (psc, scll, sclh, sdadel, scldel) = if speed.raw() <= 100_000 {
            let psc = 3;
            let scll = cmp::min((((i2c_clk.raw() >> 1) / (psc + 1)) / speed.raw()) - 1, 255);
            let sclh = scll - 4;
            let sdadel = 2;
            let scldel = 4;
            (psc, scll, sclh, sdadel, scldel)
        } else {
            let psc = 1;
            let scll = cmp::min((((i2c_clk.raw() >> 1) / (psc + 1)) / speed.raw()) - 1, 255);
            let sclh = scll - 6;
            let sdadel = 1;
            let scldel = 3;
            (psc, scll, sclh, sdadel, scldel)
        };
        psc << 28 | scldel << 20 | sdadel << 16 | sclh << 8 | scll
    }
}

/// I2C abstraction
pub struct I2c<I2C, SDA, SCL> {
    i2c: I2C,
    sda: SDA,
    scl: SCL,
}

/// I2C SDA pin
pub trait SDAPin<I2C> {}

/// I2C SCL pin
pub trait SCLPin<I2C> {}

/// I2C error
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub enum Error {
    Overrun,
    // TODO: store NACK source (address/data)
    Nack,
    PECError,
    BusError,
    ArbitrationLost,
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            Self::Overrun => ErrorKind::Overrun,
            Self::Nack => ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Unknown),
            Self::PECError => ErrorKind::Other,
            Self::BusError => ErrorKind::Bus,
            Self::ArbitrationLost => ErrorKind::ArbitrationLoss,
        }
    }
}

pub trait I2cExt<I2C> {
    fn i2c<SDA, SCL>(self, sda: SDA, scl: SCL, config: Config, rcc: &mut Rcc) -> I2c<I2C, SDA, SCL>
    where
        SDA: SDAPin<I2C>,
        SCL: SCLPin<I2C>;
}

/// Sequence to flush the TXDR register. This resets the TXIS and TXE flags
macro_rules! flush_txdr {
    ($i2c:expr) => {
        // If a pending TXIS flag is set, write dummy data to TXDR
        if $i2c.isr().read().txis().bit_is_set() {
            unsafe {
                $i2c.txdr().write(|w| w.txdata().bits(0));
            }
        }

        // If TXDR is not flagged as empty, write 1 to flush it
        if $i2c.isr().read().txe().bit_is_set() {
            $i2c.isr().write(|w| w.txe().set_bit());
        }
    };
}

macro_rules! busy_wait {
    ($i2c:expr, $flag:ident, $variant:ident) => {
        loop {
            let isr = $i2c.isr().read();

            if isr.$flag().$variant() {
                break;
            } else if isr.berr().bit_is_set() {
                $i2c.icr().write(|w| w.berrcf().set_bit());
                return Err(Error::BusError);
            } else if isr.arlo().bit_is_set() {
                $i2c.icr().write(|w| w.arlocf().set_bit());
                return Err(Error::ArbitrationLost);
            } else if isr.nackf().bit_is_set() {
                $i2c.icr()
                    .write(|w| w.stopcf().set_bit().nackcf().set_bit());
                flush_txdr!($i2c);
                return Err(Error::Nack);
            } else {
                // try again
            }
        }
    };
}

macro_rules! i2c {
    ($I2CX:ident, $i2cx:ident,
        sda: [ $($( #[ $pmetasda:meta ] )* $PSDA:ty,)+ ],
        scl: [ $($( #[ $pmetascl:meta ] )* $PSCL:ty,)+ ],
    ) => {
        $(
            $( #[ $pmetasda ] )*
            impl SDAPin<$I2CX> for $PSDA {}
        )+

        $(
            $( #[ $pmetascl ] )*
            impl SCLPin<$I2CX> for $PSCL {}
        )+

        impl I2cExt<$I2CX> for $I2CX {
            fn i2c<SDA, SCL>(
                self,
                sda: SDA,
                scl: SCL,
                config: Config,
                rcc: &mut Rcc,
            ) -> I2c<$I2CX, SDA, SCL>
            where
                SDA: SDAPin<$I2CX>,
                SCL: SCLPin<$I2CX>,
            {
                I2c::$i2cx(self, sda, scl, config, rcc)
            }
        }

        impl<SDA, SCL> I2c<$I2CX, SDA, SCL> where
            SDA: SDAPin<$I2CX>,
            SCL: SCLPin<$I2CX>
        {
            /// Initializes the I2C peripheral.
            pub fn $i2cx(i2c: $I2CX, sda: SDA, scl: SCL, config: Config, rcc: &mut Rcc) -> Self
            where
                SDA: SDAPin<$I2CX>,
                SCL: SCLPin<$I2CX>,
            {
                // Enable and reset I2C
                unsafe {
                    let rcc_ptr = &(*RCC::ptr());
                    $I2CX::enable(rcc_ptr);
                    $I2CX::reset(rcc_ptr);
                }

                // Make sure the I2C unit is disabled so we can configure it
                i2c.cr1().modify(|_, w| w.pe().clear_bit());

                // Setup protocol timings
                let timing_bits = config.timing_bits(<$I2CX as RccBus>::Bus::get_frequency(&rcc.clocks));
                i2c.timingr().write(|w| unsafe { w.bits(timing_bits) });

                // Enable the I2C processing
                unsafe {
                    i2c.cr1().modify(|_, w| {
                        w.pe()
                            .set_bit()
                            .dnf()
                            .bits(config.digital_filter)
                            .anfoff()
                            .bit(!config.analog_filter)
                    });
                }

                I2c { i2c, sda, scl }
            }

            /// Disables I2C and releases the peripheral as well as the pins.
            pub fn release(self) -> ($I2CX, SDA, SCL) {
                // Disable I2C.
                unsafe {
                    let rcc_ptr = &(*RCC::ptr());
                    $I2CX::reset(rcc_ptr);
                    $I2CX::disable(rcc_ptr);
                }

                (self.i2c, self.sda, self.scl)
            }
        }

        impl<SDA, SCL> I2c<$I2CX, SDA, SCL> {
            // copied from f3 hal
            fn read_inner(&mut self, mut addr: u16, addr_10b: bool, buffer: &mut [u8]) -> Result<(), Error> {
                if !addr_10b { addr <<= 1 };
                let end = buffer.len() / 0xFF;

                // Process 255 bytes at a time
                for (i, buffer) in buffer.chunks_mut(0xFF).enumerate() {
                    // Prepare to receive `bytes`
                    self.i2c.cr2().modify(|_, w| {
                        if i == 0 {
                            w.add10().bit(addr_10b);
                            w.sadd().set(addr);
                            w.rd_wrn().read();
                            w.start().start();
                        }
                        w.nbytes().set(buffer.len() as u8);
                        if i == end {
                            w.reload().completed().autoend().automatic()
                        } else {
                            w.reload().not_completed()
                        }
                    });

                    for byte in buffer {
                        // Wait until we have received something
                        busy_wait!(self.i2c, rxne, is_not_empty);
                        *byte = self.i2c.rxdr().read().rxdata().bits();
                    }

                    if i != end {
                        // Wait until the last transmission is finished
                        busy_wait!(self.i2c, tcr, is_complete);
                    }
                }

                // Wait until the last transmission is finished
                // auto stop is set
                busy_wait!(self.i2c, stopf, is_stop);
                self.i2c.icr().write(|w| w.stopcf().clear());

                Ok(())
            }

            fn write_inner(&mut self, mut addr: u16, addr_10b: bool, buffer: &[u8]) -> Result<(), Error> {
                if !addr_10b { addr <<= 1 };
                let end = buffer.len() / 0xFF;

                if buffer.is_empty() {
                    // 0 byte write
                    self.i2c.cr2().modify(|_, w| {
                        w.add10().bit(addr_10b);
                        w.sadd().set(addr);
                        w.rd_wrn().write();
                        w.nbytes().set(0);
                        w.reload().completed();
                        w.autoend().automatic();
                        w.start().start()
                    });
                    return Ok(())
                }
                // Process 255 bytes at a time
                for (i, buffer) in buffer.chunks(0xFF).enumerate() {
                    // Prepare to receive `bytes`
                    self.i2c.cr2().modify(|_, w| {
                        if i == 0 {
                            w.add10().bit(addr_10b);
                            w.sadd().set(addr);
                            w.rd_wrn().write();
                            w.start().start();
                        }
                        w.nbytes().set(buffer.len() as u8);
                        if i == end {
                            w.reload().completed().autoend().automatic()
                        } else {
                            w.reload().not_completed()
                        }
                    });

                    for byte in buffer {
                        // Wait until we are allowed to send data
                        // (START has been ACKed or last byte went through)
                        busy_wait!(self.i2c, txis, is_empty);
                        self.i2c.txdr().write(|w| w.txdata().set(*byte));
                    }

                    if i != end {
                        // Wait until the last transmission is finished
                        busy_wait!(self.i2c, tcr, is_complete);
                    }
                }

                // Wait until the last transmission is finished
                // auto stop is set
                busy_wait!(self.i2c, stopf, is_stop);
                self.i2c.icr().write(|w| w.stopcf().clear());
                Ok(())
            }
        }

        impl<SDA, SCL> embedded_hal::i2c::ErrorType for I2c<$I2CX, SDA, SCL> {
            type Error = Error;
        }

        // TODO: custom read/write/read_write impl with hardware stop logic
        impl<SDA, SCL> embedded_hal::i2c::I2c for I2c<$I2CX, SDA, SCL> {
            fn transaction(
                &mut self,
                address: SevenBitAddress,
                operation: &mut [Operation<'_>]
            ) -> Result<(), Self::Error> {
                Ok(for op in operation {
                    // Wait for any operation on the bus to finish
                    // for example in the case of another bus master having claimed the bus
                    while self.i2c.isr().read().busy().bit_is_set() {};
                    match op {
                        Operation::Read(data) => self.read_inner(address as u16, false, data)?,
                        Operation::Write(data) => self.write_inner(address as u16, false, data)?,
                    }
                })
            }
        }
        impl<SDA, SCL> embedded_hal::i2c::I2c<TenBitAddress> for I2c<$I2CX, SDA, SCL> {
            fn transaction(
                &mut self,
                address: TenBitAddress,
                operation: &mut [Operation<'_>]
            ) -> Result<(), Self::Error> {
                Ok(for op in operation {
                    // Wait for any operation on the bus to finish
                    // for example in the case of another bus master having claimed the bus
                    while self.i2c.isr().read().busy().bit_is_set() {};
                    match op {
                        Operation::Read(data) => self.read_inner(address, true, data)?,
                        Operation::Write(data) => self.write_inner(address, true, data)?,
                    }
                })
            }
        }

        impl<SDA, SCL> WriteRead for I2c<$I2CX, SDA, SCL> {
            type Error = Error;

            fn write_read(
                &mut self,
                addr: u8,
                bytes: &[u8],
                buffer: &mut [u8],
            ) -> Result<(), Self::Error> {
                self.write_inner(addr as u16, false, bytes)?;
                self.read_inner(addr as u16, false, buffer)?;
                Ok(())
            }
        }

        impl<SDA, SCL> Write for I2c<$I2CX, SDA, SCL> {
            type Error = Error;

            fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
                self.write_inner(addr as u16, false, bytes)?;
                Ok(())
            }
        }

        impl<SDA, SCL> Read for I2c<$I2CX, SDA, SCL> {
            type Error = Error;

            fn read(&mut self, addr: u8, bytes: &mut [u8]) -> Result<(), Self::Error> {
                self.read_inner(addr as u16, false, bytes)?;
                Ok(())
            }
        }
    }
}

i2c!(
    I2C1,
    i2c1,
    sda: [
        PA14<AlternateOD<AF4>>,
        PB7<AlternateOD<AF4>>,
        PB9<AlternateOD<AF4>>,
    ],
    scl: [
        PA13<AlternateOD<AF4>>,
        PA15<AlternateOD<AF4>>,
        PB8<AlternateOD<AF4>>,
    ],
);

i2c!(
    I2C2,
    i2c2,
    sda: [
        PA8<AlternateOD<AF4>>,
        PF0<AlternateOD<AF4>>,
    ],
    scl: [
        PA9<AlternateOD<AF4>>,
        PC4<AlternateOD<AF4>>,
        #[cfg(any(
            feature = "stm32g471",
            feature = "stm32g473",
            feature = "stm32g474",
            feature = "stm32g483",
            feature = "stm32g484"
        ))]
        PF6<AlternateOD<AF4>>,
    ],
);

i2c!(
    I2C3,
    i2c3,
    sda: [
        PB5<AlternateOD<AF8>>,
        PC11<AlternateOD<AF8>>,
        PC9<AlternateOD<AF8>>,
        #[cfg(any(
            feature = "stm32g471",
            feature = "stm32g473",
            feature = "stm32g474",
            feature = "stm32g483",
            feature = "stm32g484"
        ))]
        PF4<AlternateOD<AF4>>,
        #[cfg(any(
            feature = "stm32g471",
            feature = "stm32g473",
            feature = "stm32g474",
            feature = "stm32g483",
            feature = "stm32g484"
        ))]
        PG8<AlternateOD<AF4>>,
    ],
    scl: [
        PA8<AlternateOD<AF2>>,
        PC8<AlternateOD<AF8>>,
        #[cfg(any(
            feature = "stm32g471",
            feature = "stm32g473",
            feature = "stm32g474",
            feature = "stm32g483",
            feature = "stm32g484"
        ))]
        PF3<AlternateOD<AF4>>,
        #[cfg(any(
            feature = "stm32g471",
            feature = "stm32g473",
            feature = "stm32g474",
            feature = "stm32g483",
            feature = "stm32g484"
        ))]
        PG7<AlternateOD<AF4>>,
    ],
);

#[cfg(any(
    feature = "stm32g471",
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484"
))]
i2c!(
    I2C4,
    i2c4,
    sda: [
        PB7<AlternateOD<AF3>>,
        PC7<AlternateOD<AF8>>,
        PF15<AlternateOD<AF4>>,
        PG4<AlternateOD<AF4>>,
    ],
    scl: [
        PA13<AlternateOD<AF3>>,
        PC6<AlternateOD<AF8>>,
        PF14<AlternateOD<AF4>>,
        PG3<AlternateOD<AF4>>,
    ],
);
