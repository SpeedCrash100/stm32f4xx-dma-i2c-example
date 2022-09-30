#![no_std]
#![no_main]

extern crate display_interface;
extern crate ssd1306;

mod dma_display_connector;

/// HAL library for our device
extern crate stm32f4xx_hal as hal;

/// Peripheral Access Crate for our device
pub use hal::pac;

use panic_halt as _;

#[rtic::app(device = crate::pac, peripherals = true, dispatchers = [USART6, SPI5])]
mod app {

    // Standart library imports
    use core::cell::RefCell;
    use core::sync::atomic::AtomicI32;
    use core::sync::atomic::Ordering;

    // Cortex specific
    use cortex_m::asm::wfi;

    // HAL imports
    use hal::dma::*;
    use hal::gpio::*;
    use hal::i2c::dma::{I2CMasterDma, I2CMasterWriteDMA, I2CMasterWriteReadDMA};
    use hal::pac::{DMA1, I2C1};
    use hal::prelude::*;
    use hal::timer::MonoTimerUs;

    // External imports
    use crate::ssd1306::mode::DisplayConfig;
    use critical_section::Mutex;
    use ssd1306::rotation::DisplayRotation;
    use ssd1306::size::DisplaySize128x64;
    use ssd1306::Ssd1306;

    use crate::dma_display_connector;

    // Types
    pub type I2c1Handle = I2CMasterDma<
        I2C1,
        (PB8<AF4<OpenDrain>>, PB9<AF4<OpenDrain>>),
        Stream1<DMA1>,
        0,
        Stream0<DMA1>,
        1,
    >;
    pub type I2c1HandleProtected = Mutex<RefCell<I2c1Handle>>;
    pub type Display = Ssd1306<
        dma_display_connector::DMAI2cInterface<I2c1Handle>,
        DisplaySize128x64,
        ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>,
    >;

    #[shared]
    struct Shared {
        temperature: AtomicI32,
        i2c: &'static I2c1HandleProtected,
    }

    #[local]
    struct Local {
        temp_register_address: [u8; 1],
        raw_temperature_buf: [u8; 2],

        // Display need to init with interrupts enabled(not in init)
        display: Option<Display>,
    }

    #[monotonic(binds = TIM5, default = true)]
    type MicrosecMono = MonoTimerUs<crate::pac::TIM5>;

    /// Init function running on reset
    ///
    /// * Configures clocks to 100 MHz
    /// * Configures PA5(User LED) for tick indication
    /// * Creates I2C bus, display, temperature sensor, RTC
    /// * Configures joystick
    /// * Starts repeating tasks
    #[init(local = [
        _i2c_bus: Option<I2c1HandleProtected> = None
    ])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Init clocks
        let dp = ctx.device;

        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(100.MHz()).freeze();

        // Timers
        let mono = dp.TIM5.monotonic_us(&clocks);

        // I2C bus init
        let gpiob = dp.GPIOB.split();
        // First create a simple blocking I2C bus
        let i2c = dp.I2C1.i2c(
            (
                gpiob.pb8.into_alternate_open_drain(),
                gpiob.pb9.into_alternate_open_drain(),
            ),
            400.kHz(),
            &clocks,
        );

        // On my board it is required to manually toggle Reset Pin of display
        let gpioa = dp.GPIOA.split();
        gpioa.pa8.into_push_pull_output().set_high();

        // Then convert it to DMA
        let streams = StreamsTuple::new(dp.DMA1);
        let i2c_dma = i2c.use_dma(streams.1, streams.0);
        *ctx.local._i2c_bus = Some(Mutex::new(RefCell::new(i2c_dma)));

        let i2c_bus_ref = ctx.local._i2c_bus.as_ref().unwrap();

        // Start tasks
        get_temp::spawn().unwrap();
        draw::spawn_after(100.millis()).unwrap();

        (
            Shared {
                i2c: i2c_bus_ref,
                temperature: AtomicI32::new(0),
            },
            Local {
                temp_register_address: [0],
                raw_temperature_buf: [0; 2],
                display: None,
            },
            init::Monotonics(mono),
        )
    }

    /// Idle function runs when nothing to do
    /// Used for sleep
    #[idle(local = [], shared = [])]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            wfi();
        }
    }

    #[task(local = [temp_register_address, raw_temperature_buf], shared = [&i2c, &temperature], priority = 3)]
    fn get_temp(ctx: get_temp::Context) {
        // LM75B updates reading every 100 ms
        get_temp::spawn_after(100.millis()).unwrap();

        // Convert current readings
        let mut raw_temp_copy = [0_u8; 2];
        raw_temp_copy.copy_from_slice(ctx.local.raw_temperature_buf);
        let temp = i16::from_be_bytes(raw_temp_copy) >> 5; // According to docs 5 less significant bits should be ignored
        let real_temp = (temp as f32 * 0.125) as i32;
        ctx.shared.temperature.store(real_temp, Ordering::Relaxed);

        // LM75B address
        let addr = 0b1001000;

        nb::block!(
            // Critical section should be inside `block!` otherwise interrupt for DMA stream will never called to handle end of transmition
            critical_section::with(|cs| {
                let mut bus = ctx.shared.i2c.borrow(cs).borrow_mut();
                // Safe because temp_register_address and raw_temperature_buf will live whole program

                bus.write_read(
                    addr,
                    ctx.local.temp_register_address,
                    ctx.local.raw_temperature_buf,
                )
            })
        )
        .unwrap();
    }

    #[task(local = [display], shared = [&i2c, &temperature], priority = 1)]
    fn draw(ctx: draw::Context) {
        draw::spawn_after(100.millis()).unwrap();
        // init display
        let i2c_bus = ctx.shared.i2c;
        let display = ctx.local.display.get_or_insert_with(|| {
            let display_connector = dma_display_connector::DMAI2cInterface::new(i2c_bus);

            let mut out = Ssd1306::new(
                display_connector,
                DisplaySize128x64,
                DisplayRotation::Rotate0,
            )
            .into_buffered_graphics_mode();

            out.init().unwrap();

            out
        });

        use core::fmt::Write;
        use embedded_graphics::mono_font::ascii::FONT_10X20;
        use embedded_graphics::mono_font::{MonoTextStyle, MonoTextStyleBuilder};
        use embedded_graphics::pixelcolor::BinaryColor;
        use embedded_graphics::prelude::*;
        use embedded_graphics::primitives::{PrimitiveStyleBuilder, Sector};
        use embedded_graphics::text::{Alignment, Text};

        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(BinaryColor::On)
            .build();

        display.clear();

        let mut temp_string: heapless::String<10> = heapless::String::default();
        write!(
            &mut temp_string,
            "{} C",
            ctx.shared.temperature.load(Ordering::Relaxed)
        )
        .ok();

        Text::with_alignment(
            temp_string.as_str(),
            Point { x: 64, y: 48 },
            text_style,
            Alignment::Center,
        )
        .draw(display)
        .unwrap();

        display.flush().unwrap();
    }

    /// Handles DMA interrupt for Tx
    #[task(binds = DMA1_STREAM1, shared = [&i2c], priority = 5)]
    fn i2c_dma_tx_it(ctx: i2c_dma_tx_it::Context) {
        critical_section::with(|cs| {
            let mut c = ctx.shared.i2c.borrow(cs).borrow_mut();
            c.handle_dma_interrupt();
        })
    }

    /// Handles DMA interrupt for Rx
    #[task(binds = DMA1_STREAM0, shared = [&i2c], priority = 5)]
    fn i2c_dma_rx_it(ctx: i2c_dma_rx_it::Context) {
        critical_section::with(|cs| {
            let mut c = ctx.shared.i2c.borrow(cs).borrow_mut();
            c.handle_dma_interrupt();
        })
    }
}
