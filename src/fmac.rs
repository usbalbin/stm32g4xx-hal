use core::marker::PhantomData;

use crate::stm32::RCC;
use crate::{rcc::{Enable, Rcc, Reset}, stm32::FMAC};
pub use fixed::types::I1F15;

enum Func {
    /// Load X1 buffer
    ///
    /// Preload X1 buffer with N values, starting from the address in X1_BASE.
    /// Successive writes to the FMAC_WDATA register load the write data into the X1 buffer and
    /// increment the write address. The write pointer points to the address X1_BASE + N when the
    /// function completes.
    ///
    /// * P is the number of values to be loaded
    /// * Q is not used
    /// * R is not used
    LoadX1 = 1,

    /// Load X2 buffer
    ///
    /// Preload X2 buffer with N + M values, starting from the address in X2_BASE.
    /// Successive writes to the FMAC_WDATA register load the write data into the X1 buffer and
    /// increment the write address. In the case of an IIR, the N feed-forward and M feed-back coefficients
    /// are concatenated and loaded together into the X2 buffer. The total number of coefficients is
    /// equal to N + M. For an FIR, there are no feedback coefficients, so M = 0.
    ///
    /// * P is N
    /// * Q is M
    /// * R is not used
    LoadX2 = 2,

    /// Load Y buffer
    ///
    /// Preload Y buffer with N values, starting from the address in Y_BASE.
    /// Successive writes to the FMAC_WDATA register load the write data into the Y buffer and
    /// increment the write address. The write pointer points to the address Y_BASE + N when the
    /// function completes.
    ///
    /// * P is the number of values to be loaded
    /// * Q is not used
    /// * R is not used
    LoadY = 3,

    /// Dot product Y[n]=2^R * (X1 dot X2)
    ///
    /// * Y is the result
    /// * X1 is a circular buffer with sample data
    /// * X2 is a static buffer with n values values
    ///
    /// * P(2..=127) is the length of X1 and X2
    /// * Q is not used
    /// * R(0..=7) is the exponent of the gain=2^R
    Fir = 8,

    /// Y[n] = 2^R * ((B dot X) + (A dot Y[0..=n-1]))
    ///
    /// * Y is the result
    /// * X1 is a circular buffer with sample data
    /// * X2 is a static buffer with B and A concatenated (B[0], B[1]..., B[N-1], A[0], A[1], ..., A[M-1]
    ///
    /// * P(2..=64) is N which is the length of B
    /// * Q(1..=63) is M which is the length of A
    /// * R(0..=7) is the exponent of the gain=2^R
    Iir = 9,
}

fn write_x1(fmac: &mut FMAC, x1: &[I1F15], base_addr: u8) {
    assert!(x1.len() <= 255);
    assert!(fmac.param().read().start().bit_is_clear());

    fmac.x1bufcfg().write(|w| unsafe {
        w.x1_base()
            .bits(base_addr)
            .x1_buf_size()
            .bits(x1.len() as u8)
            .full_wm()
            .bits(0)
    });

    fmac.param().write(|w| unsafe {
        w.p()
            .bits(x1.len() as u8)
            .func()
            .bits(Func::LoadX1 as u8)
            .start()
            .bit(true)
    });

    for x in x1.into_iter() {
        fmac.wdata()
            .write(|w| unsafe { w.wdata().bits(x.to_bits() as u16) });
    }

    assert!(fmac.param().read().start().bit_is_clear());
}

/// Set `a` to the empty slice if not used
fn write_x2(fmac: &mut FMAC, b: &[I1F15], a: &[I1F15], base_addr: u8) {
    assert!(b.len() <= 255);
    assert!(a.len() <= 255);
    assert!(b.len() + a.len() <= 256);
    assert!(fmac.param().read().start().bit_is_clear());

    fmac.x2bufcfg().write(|w| unsafe {
        w.x2_base()
            .bits(base_addr)
            .x2_buf_size()
            .bits((b.len() + a.len()) as u8)
    });

    fmac.param().write(|w| unsafe {
        w.p()
            .bits(b.len() as u8)
            .q()
            .bits(a.len() as u8)
            .func()
            .bits(Func::LoadX2 as u8)
            .start()
            .bit(true)
    });

    for x in b.into_iter() {
        fmac.wdata()
            .write(|w| unsafe { w.wdata().bits(x.to_bits() as u16) });
    }
    for x in a.into_iter() {
        fmac.wdata()
            .write(|w| unsafe { w.wdata().bits(x.to_bits() as u16) });
    }

    assert!(fmac.param().read().start().bit_is_clear());
}

fn write_y(fmac: &mut FMAC, y: &[I1F15], base_addr: u8) {
    assert!(y.len() <= 255);
    assert!(fmac.param().read().start().bit_is_clear());

    fmac.ybufcfg().write(|w| unsafe {
        w.y_base()
            .bits(base_addr)
            .y_buf_size()
            .bits(y.len() as u8)
            .empty_wm()
            .bits(0)
    });

    fmac.param().write(|w| unsafe {
        w.p()
            .bits(y.len() as u8)
            .func()
            .bits(Func::LoadY as u8)
            .start()
            .bit(true)
    });

    for x in y.into_iter() {
        fmac.wdata()
            .write(|w| unsafe { w.wdata().bits(x.to_bits() as u16) });
    }

    assert!(fmac.param().read().start().bit_is_clear());
}

fn init_buffers(fmac: &mut FMAC, x1: &[I1F15], b: &[I1F15], a: &[I1F15], y: &[I1F15]) {
    let x1_base_addr = 0;
    let x2_base_addr = x1.len() as u8;
    let y_base_addr = x2_base_addr + (b.len() + a.len()) as u8;

    write_x1(fmac, x1, x1_base_addr);
    write_x2(fmac, b, a, x2_base_addr);
    write_y(fmac, y, y_base_addr);
}

#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FmacError {
    X1Full,
}

#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Gain {
    X1 = 0,
    X2 = 1,
    X4 = 2,
    X8 = 3,
    X16 = 4,
    X32 = 5,
    X64 = 6,
    X128 = 7,
}

pub struct Poll;
//struct Dma;
//struct Interrupt;

pub struct Fir<RMODE, WMODE> {
    _rmode: PhantomData<RMODE>,
    _wmode: PhantomData<WMODE>,
    fmac: FMAC,
}

pub struct Iir<RMODE, WMODE> {
    _rmode: PhantomData<RMODE>,
    _wmode: PhantomData<WMODE>,
    fmac: FMAC,
}

pub trait FmacExt {
    fn fir(
        self,
        initial_x1: &[I1F15],
        initial_x2: &[I1F15],
        initial_y: &[I1F15],
        r: Gain,
        rcc: &mut Rcc
    ) -> Fir<Poll, Poll>;
    fn iir(
        self,
        cfg: IirConfig,
        rcc: &mut Rcc
    ) -> Iir<Poll, Poll>;
}

impl FmacExt for FMAC {
    /// Dot product Y[n]=2^R * (X1 dot X2)
    ///
    /// * Y is the result
    /// * X1 is a circular buffer with sample data
    /// * X2 is a static buffer with n values values
    ///
    /// * P(2..=127) is the length of X1 and X2
    /// * Q is not used
    /// * R(0..=7) is the exponent of the gain=2^R
    fn fir(
        mut self,
        initial_x1: &[I1F15],
        initial_x2: &[I1F15],
        initial_y: &[I1F15],
        r: Gain,
        _rcc: &mut Rcc
    ) -> Fir<Poll, Poll> {
        unsafe {
            let rcc_ptr = &(*RCC::ptr());
            FMAC::enable(rcc_ptr);
            FMAC::reset(rcc_ptr);
        }

        assert_eq!(initial_x1.len(), initial_x2.len());
        let p = initial_x1.len();
        assert!((2..=127).contains(&p));

        init_buffers(&mut self, initial_x1, initial_x2, &[], initial_y);

        self.param().write(|w| unsafe {
            w.p()
                .bits(p as u8)
                .q()
                .bits(0)
                .r()
                .bits(r as u8)
                .func()
                .bits(Func::Fir as u8)
                .start()
                .bit(true)
        });

        Fir {
            _rmode: PhantomData,
            _wmode: PhantomData,
            fmac: self,
        }
    }

    /// Y[n] = 2^R * ((B dot X) + (A dot Y[0..=n-1]))
    ///
    /// * Y is the result
    /// * X1 is a circular buffer with sample data
    /// * X2 is a static buffer with B and A concatenated (B[0], B[1]..., B[N-1], A[0], A[1], ..., A[M-1]
    ///
    /// * P(2..=64) is N which is the length of B
    /// * Q(1..=63) is M which is the length of A
    /// * R(0..=7) is the exponent of the gain=2^R
    fn iir(mut self, cfg: IirConfig, _rcc: &mut Rcc) -> Iir<Poll, Poll> {
        unsafe {
            let rcc_ptr = &(*RCC::ptr());
            FMAC::enable(rcc_ptr);
            FMAC::reset(rcc_ptr);
        }

        let p = cfg.initial_b.len();
        let q = cfg.initial_a.len();

        init_buffers(&mut self, cfg.initial_x1, cfg.initial_b, cfg.initial_a, cfg.initial_y);

        self.param().write(|w| unsafe {
            w.p()
                .bits(p as u8)
                .q()
                .bits(q as u8)
                .r()
                .bits(cfg.r as u8)
                .func()
                .bits(Func::Iir as u8)
                .start()
                .bit(true)
        });

        Iir {
            _rmode: PhantomData,
            _wmode: PhantomData,
            fmac: self,
        }
    }
}

// TODO
fn set_config(fmac: &mut FMAC) {
    fmac.cr().write(|w| {
        w.rien()
            .bit(false)
            .wien()
            .bit(false)
            .ovflien()
            .bit(false)
            .unflien()
            .bit(false)
            .satien()
            .bit(false)
            .dmaren()
            .bit(false)
            .dmawen()
            .bit(false)
            .clipen()
            .bit(false)
            .reset()
            .bit(false)
    });
}

fn read(fmac: &mut FMAC) -> Option<I1F15> {
    if fmac.sr().read().yempty().bit_is_set() {
        return None;
    }

    Some(I1F15::from_bits(fmac.rdata().read().rdata().bits() as i16))
}

fn read_blocking(fmac: &mut FMAC) -> I1F15 {
    loop {
        if let Some(x) = read(fmac) {
            return x;
        }
    }
}

fn write(fmac: &mut FMAC, value: u16) -> Result<(), FmacError> {
    if fmac.sr().read().x1full().bit_is_set() {
        return Err(FmacError::X1Full);
    }
    fmac.wdata().write(|w| unsafe { w.wdata().bits(value) });

    Ok(())
}

fn write_blocking(fmac: &mut FMAC, value: I1F15) {
    let value = value.to_bits() as u16;
    loop {
        if let Ok(()) = write(fmac, value) {
            return;
        }
    }
}

impl<WM> Fir<Poll, WM> {
    pub fn read(&mut self) -> Option<I1F15> {
        read(&mut self.fmac)
    }

    /// NOTE: This will block forever if there is no new data written
    pub fn read_blocking(&mut self) -> I1F15 {
        read_blocking(&mut self.fmac)
    }
}

impl<RM> Fir<RM, Poll> {
    pub fn write(&mut self, value: I1F15) -> Result<(), FmacError> {
        write(&mut self.fmac, value.to_bits() as u16)
    }

    pub fn write_blocking(&mut self, value: I1F15) {
        write_blocking(&mut self.fmac, value);
    }
}

impl Fir<Poll, Poll> {
    pub fn compute_blocking(&mut self, value: I1F15) -> I1F15 {
        self.write_blocking(value);
        self.read_blocking()
    }
}

#[derive(Copy, Clone)]
pub struct IirConfig<'a, 'b, 'c, 'd> {
    initial_x1: &'a [I1F15],
    initial_b: &'b [I1F15],
    initial_a: &'c [I1F15],
    initial_y: &'d [I1F15],
    r: Gain,
}

impl<'a, 'b, 'c, 'd> IirConfig<'a, 'b, 'c, 'd> {
    pub const fn new(
        initial_x1: &'a [I1F15],
        initial_b: &'b [I1F15],
        initial_a: &'c [I1F15],
        initial_y: &'d [I1F15],
        r: Gain,
    ) -> Self {
        assert!(initial_x1.len() <= 255);
        assert!(initial_b.len() + initial_a.len() <= 255);
        assert!(initial_y.len() <= 255);

        assert!(2 <= initial_b.len() && initial_b.len() <= 64);
        assert!(1 <= initial_a.len() && initial_a.len() <= initial_b.len() - 1);

        let num_input_samples = initial_x1.len();
        let num_output_samples = initial_y.len();
        assert!(initial_b.len() == num_input_samples);
        assert!(initial_a.len() == num_output_samples);

        assert!(initial_x1.len() + initial_b.len() + initial_a.len() + initial_y.len() <= 256);

        Self {
            initial_x1,
            initial_b,
            initial_a,
            initial_y,
            r,
        }
    }
}

impl<WM> Iir<Poll, WM> {
    pub fn read(&mut self) -> Option<I1F15> {
        read(&mut self.fmac)
    }

    /// NOTE: This will block forever if there is no new data written
    pub fn read_blocking(&mut self) -> I1F15 {
        read_blocking(&mut self.fmac)
    }
}

impl<RM> Iir<RM, Poll> {
    pub fn write(&mut self, value: I1F15) -> Result<(), FmacError> {
        write(&mut self.fmac, value.to_bits() as u16)
    }

    pub fn write_blocking(&mut self, value: I1F15) {
        write_blocking(&mut self.fmac, value);
    }
}

impl Iir<Poll, Poll> {
    pub fn compute_blocking(&mut self, value: I1F15) -> I1F15 {
        self.write_blocking(value);
        self.read_blocking()
    }
}
