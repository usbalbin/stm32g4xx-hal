use stm32g4::stm32g474::FMAC;
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

fn foo(fmac: FMAC) {
    fmac.param().write(|w| unsafe {
        w
            .p().bits(0)
            .q().bits(0)
            .r().bits(0)
            .func().bits(0)
            .start().bit(false)
    });
    fmac.cr().write(|w| w
        .rien().bit(false)
        .wien().bit(false)
        .ovflien().bit(false)
        .unflien().bit(false)
        .satien().bit(false)
        .dmaren().bit(false)
        .dmawen().bit(false)
        .clipen().bit(false)
        .reset().bit(false)
    );

    fmac.x1bufcfg().write(|w| unsafe { 
        w
            .x1_base().bits(0)
            .x1_buf_size().bits(0)
            .full_wm().bits(0)
    });
    fmac.x2bufcfg().write(|w| unsafe {
        w
            .x2_base().bits(0)
            .x2_buf_size().bits(0)
    });
    fmac.ybufcfg().write(|w| unsafe {
        w
            .y_base().bits(0)
            .y_buf_size().bits(0)
            .empty_wm().bits(0)
    });
    fmac.wdata().write(|w| unsafe { w.wdata().bits(0) }); // Input
    fmac.rdata().read().bits(); // Output
}

fn write_x1(fmac: &mut FMAC, x1: &[I1F15], base_addr: u8) {
    assert!(x1.len() <= 255);
    assert!(fmac.param().read().start().bit_is_clear());
    
    fmac.x1bufcfg().write(|w| unsafe { 
        w
            .x1_base().bits(base_addr)
            .x1_buf_size().bits(x1.len() as u8)
            .full_wm().bits(0)
    });

    fmac.param().write(|w| unsafe {
        w
            .p().bits(x1.len() as u8)
            .func().bits(Func::LoadX1 as u8)
            .start().bit(true)
    });

    for x in x1.into_iter() {
        fmac.wdata().write(|w| unsafe { w.wdata().bits(x.to_bits() as u16) });
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
        w
            .x2_base().bits(base_addr)
            .x2_buf_size().bits((b.len() + a.len()) as u8)
    });

    fmac.param().write(|w| unsafe {
        w
            .p().bits(b.len() as u8)
            .q().bits(a.len() as u8)
            .func().bits(Func::LoadX2 as u8)
            .start().bit(true)
    });

    for x in b.into_iter() {
        fmac.wdata().write(|w| unsafe { w.wdata().bits(x.to_bits() as u16) });
    }   
    for x in a.into_iter() {
        fmac.wdata().write(|w| unsafe { w.wdata().bits(x.to_bits() as u16) });
    }

    assert!(fmac.param().read().start().bit_is_clear());
}

fn write_y(fmac: &mut FMAC, y: &[I1F15], base_addr: u8) {
    assert!(y.len() <= 255);
    assert!(fmac.param().read().start().bit_is_clear());

    fmac.ybufcfg().write(|w| unsafe {
        w
            .y_base().bits(base_addr)
            .y_buf_size().bits(y.len() as u8)
            .empty_wm().bits(0)
    });

    fmac.param().write(|w| unsafe {
        w
            .p().bits(y.len() as u8)
            .func().bits(Func::LoadY as u8)
            .start().bit(true)
    });

    for x in y.into_iter() {
        fmac.wdata().write(|w| unsafe { w.wdata().bits(x.to_bits() as u16) });
    }
    
    assert!(fmac.param().read().start().bit_is_clear());
}

fn init_buffers(fmac: &mut FMAC, x1: &[I1F15], b: &[I1F15], a: &[I1F15], y: &[I1F15]) {
    assert!(x1.len() + b.len() + a.len() + y.len() <= 256);
    
    let x1_base_addr = 0;
    let x2_base_addr = x1.len() as u8;
    let y_base_addr = x2_base_addr + (b.len() + a.len()) as u8;

    write_x1(fmac, x1, x1_base_addr);
    write_x2(fmac, b, a, x2_base_addr);
    write_y(fmac, y, y_base_addr);
}

enum Gain {
    X1 = 0,
    X2 = 1,
    X4 = 2,
    X8 = 3,
    X16 = 4,
    X32 = 5,
    X64 = 6,
    X128 = 7,
}

fn init_fir(fmac: &mut FMAC, initial_x1: &[I1F15], initial_x2: &[I1F15], initial_y: &[I1F15], r: Gain) {
    assert_eq!(initial_x1.len(), initial_x2.len());
    let p = initial_x1.len();
    assert!((2..=127).contains(&p));

    init_buffers(fmac, initial_x1, initial_x2, &[], initial_y);

    fmac.param().write(|w| unsafe {
        w
            .p().bits(p as u8)
            .q().bits(0)
            .r().bits(r as u8)
            .func().bits(Func::Iir as u8)
            .start().bit(true)
    });
}

fn init_iir(fmac: &mut FMAC, initial_x1: &[I1F15], initial_b: &[I1F15], initial_a: &[I1F15], initial_y: &[I1F15], r: Gain) {
    let p = initial_b.len();
    let q = initial_a.len();
    assert!((2..=64).contains(&p));
    assert!((1..=63).contains(&q));

    init_buffers(fmac, initial_x1, initial_b, initial_a, initial_y);

    fmac.param().write(|w| unsafe {
        w
            .p().bits(p as u8)
            .q().bits(q as u8)
            .r().bits(r as u8)
            .func().bits(Func::Iir as u8)
            .start().bit(true)
    });
}