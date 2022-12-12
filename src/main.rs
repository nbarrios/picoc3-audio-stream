//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

pub mod bsp;

use panic_semihosting as _;

#[rtic::app(device = crate::bsp::hal::pac, peripherals = true, dispatchers = [TIMER_IRQ_2, TIMER_IRQ_3])]
mod app {
    use crate::bsp::hal;
    use core::convert::TryInto;
    use core::fmt::Write;
    use defmt_rtt as _;
    use display_interface_spi::SPIInterface;
    use embedded_graphics::{
        mono_font::{MonoTextStyle},
        pixelcolor::{self, Rgb565},
        prelude::*,
        primitives::Rectangle,
    };
    use embedded_hal::digital::v2::OutputPin;
    use embedded_hal::serial::{Read, Write as SerialWrite};
    use embedded_hal::adc::OneShot;
    use embedded_text::{
        alignment::HorizontalAlignment,
        style::{HeightMode, TextBoxStyleBuilder},
        TextBox,
    };
    use rp2040_monotonic::Rp2040Monotonic;
    use rp2040_monotonic::fugit::ExtU64;
    use rp2040_monotonic::fugit::RateExtU32;
    use hal::{
        clocks::ClocksManager,
        clocks::{init_clocks_and_plls, Clock},
        gpio,
        gpio::pin::bank0::{Gpio0, Gpio1, Gpio2, Gpio3, Gpio4, Gpio5, Gpio8, Gpio9, Gpio12, Gpio13, Gpio27},
        gpio::Output,
        gpio::Input,
        gpio::Pin,
        gpio::PullDown,
        gpio::PushPull,
        gpio::Floating,
        gpio::{Disabled, Function, Uart},
        pac::{self},
        pac::Interrupt,
        sio::Sio,
        spi,
        uart::{Enabled, UartPeripheral},
        watchdog::Watchdog,
    };
    use heapless::spsc::{Consumer, Producer, Queue};
    use profont::PROFONT_10_POINT;
    use st7789::ST7789;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Mono = Rp2040Monotonic;

    type SPIIntType = SPIInterface<
        spi::Spi<spi::Enabled, pac::SPI0, 8>,
        Pin<Gpio1, Output<PushPull>>,
        Pin<Gpio5, Output<PushPull>>,
    >;

    type UartPins = (Pin<Gpio8, Function<Uart>>, Pin<Gpio9, Function<Uart>>);

    #[shared]
    struct Shared {
        #[lock_free]
        uart0: UartPeripheral<Enabled, pac::UART0, (Pin<Gpio12, Function<Uart>>, Pin<Gpio13, Function<Uart>>)>,
    }

    #[local]
    struct Local {
        display: ST7789<SPIIntType, Pin<Gpio0, Output<PushPull>>, Pin<Gpio4, Output<PushPull>>>,
        uart1: UartPeripheral<Enabled, pac::UART1, UartPins>,
        uart_rx_read: Consumer<'static, u8, 256>,
        uart_rx_write: Producer<'static, u8, 256>,
        uart_tx_read: Consumer<'static, u8, 256>,
        uart_tx_write: Producer<'static, u8, 256>,
        adc: hal::Adc,
        adc_pin_1: Pin<Gpio27, Input<Floating>>,
    }

    #[init(local = [rx_q: Queue<u8, 256> = Queue::new(), tx_q: Queue<u8, 256> = Queue::new()])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        unsafe {
            crate::bsp::hal::sio::spinlock_reset();
        }

        let mut resets = cx.device.RESETS;
        let mut watchdog = Watchdog::new(cx.device.WATCHDOG);
        let sio = Sio::new(cx.device.SIO);

        // External high-speed crystal on the pico board is 12Mhz
        let clocks = init_clocks_and_plls(
            crate::bsp::XOSC_CRYSTAL_FREQ,
            cx.device.XOSC,
            cx.device.CLOCKS,
            cx.device.PLL_SYS,
            cx.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let mut delay =
            cortex_m::delay::Delay::new(cx.core.SYST, clocks.system_clock.freq().raw());

        let mono = Rp2040Monotonic::new(cx.device.TIMER);

        let pins = crate::bsp::Pins::new(
            cx.device.IO_BANK0,
            cx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        //let mut led_pin = pins.led.into_push_pull_output();

        //UART1 (ESP32-AT)
        let uart_pins: UartPins = (
            pins.gpio8.into_mode::<gpio::FunctionUart>(),
            pins.gpio9.into_mode::<gpio::FunctionUart>(),
        );
        let mut uart1 = uart_setup(uart_pins, cx.device.UART1, &clocks, &mut resets);

        let (mut uart_rx_write, uart_rx_read) = cx.local.rx_q.split();
        let (uart_tx_write, uart_tx_read) = cx.local.tx_q.split();

        //UART0 (PICOPROBE)
        let uart0_pins = (
            pins.gpio12.into_mode::<gpio::FunctionUart>(),
            pins.gpio13.into_mode::<gpio::FunctionUart>(),
        );
        let uart0 = 
            UartPeripheral::new(cx.device.UART0, uart0_pins, &mut resets)
            .enable(hal::uart::common_configs::_115200_8_N_1, clocks.peripheral_clock.freq()).unwrap();

        let adc = hal::Adc::new(cx.device.ADC, &mut resets); 
        let adc_pin_1 = pins.gpio27.into_floating_input();

        let mut display = tft_display_setup(
            pins.tft_bl,
            pins.tft_rst,
            pins.tft_dc,
            pins.tft_sclk,
            pins.tft_mosi,
            pins.tft_csn,
            cx.device.SPI0,
            &mut resets,
            &clocks,
            &mut delay,
        );
        display.clear(pixelcolor::Rgb565::RED).unwrap();

        let init_msg = "Starting...\n".as_bytes();
        for byte in init_msg.iter() {
            uart_rx_write.enqueue(*byte).unwrap();
        } 

        write_serial::spawn_after(1_u64.secs()).unwrap();
        read_serial::spawn_after(1_u64.secs()).unwrap();
        //update_display::spawn_after(250_u64.millis()).unwrap();
        read_adc::spawn_after(1_u64.secs()).unwrap();
        //led_pin.set_high().unwrap();
        //led_pin.set_low().unwrap();

        (
            Shared {
                uart0,
            },
            Local {
                display,
                uart1,
                uart_rx_read,
                uart_rx_write,
                uart_tx_read,
                uart_tx_write,
                adc,
                adc_pin_1,
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        // unsafe {
        //     hal::pac::NVIC::unmask(Interrupt::UART1_IRQ);
        // }

        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(shared = [uart0], local = [adc, adc_pin_1])]
    fn read_adc(cx: read_adc::Context) {
        let adc_count: u16 = cx.local.adc.read(cx.local.adc_pin_1).unwrap();

        //let _res = write!(cx.shared.uart0, "ADC: {}\n", adc_count);
        read_adc::spawn_after(1_u64.secs()).unwrap();
    }

    #[task(local = [uart_tx_write, index: usize = 0], shared = [uart0])]
    fn write_serial(cx: write_serial::Context) {
        let commands = [
            "AT+GMR\r\n",
            "AT+SYSSTORE=0\r\n",
            "AT+CWMODE=1,0\r\n",
            "AT+CWSTATE?\r\n",
            "AT+CWJAP=\"Home 1\",\"1234554321\"\r\n",
            "AT+CIPSTA?\r\n"
        ];

        if *cx.local.index < commands.len() {
            write!(cx.shared.uart0, "{}", commands[*cx.local.index]).unwrap();

            let at_gmr = commands[*cx.local.index].as_bytes();
            for byte in at_gmr.iter() {
                cx.local.uart_tx_write.enqueue(*byte).unwrap();
            }
            *cx.local.index += 1;
            write_serial::spawn_after(3.secs()).unwrap();

            unsafe {
                hal::pac::NVIC::unmask(Interrupt::UART1_IRQ);
            }
            rtic::pend(Interrupt::UART1_IRQ);
        }
    }

    #[task(shared = [uart0], local = [uart_rx_read])]
    fn read_serial(cx: read_serial::Context) {
        let uart0 = cx.shared.uart0;
        let rx_q = cx.local.uart_rx_read;

        while let Some(byte) = rx_q.peek().cloned() {
            if uart0.write(byte).is_ok() {
                rx_q.dequeue();
            } else {
                break;
            }
        }
        read_serial::spawn_after(1.millis()).unwrap();
    }

    #[task(local = [display,
                    index: usize = 0, 
                    buffer: [u8; 256] = [0; 256],
                   ],
            shared = [uart0]
          )]
    fn update_display(cx: update_display::Context) {
        let mut buffer_updated = false;

        // while let Some(byte) = cx.local.uart_rx_read.dequeue() {
        //     cx.local.buffer[*cx.local.index] = byte;
        //     *cx.local.index += 1;

        //     buffer_updated = true;
        //     if *cx.local.index == cx.local.buffer.len() {
        //         *cx.local.index = 0;
        //         cx.local.display.clear(pixelcolor::Rgb565::RED).unwrap();
        //     }
        // }

        if buffer_updated && *cx.local.index > 0 {
            cx.local.display.clear(pixelcolor::Rgb565::BLACK).unwrap();

            let character_style: MonoTextStyle<Rgb565> = MonoTextStyle::new(&PROFONT_10_POINT, Rgb565::WHITE);
            let textbox_style = TextBoxStyleBuilder::new()
                .height_mode(HeightMode::FitToText)
                .alignment(HorizontalAlignment::Left)
                .paragraph_spacing(6)
                .build();
            let bounds = Rectangle::new(Point::new(40, 53), Size::new(240, 0));

            if let Ok(str_rd) = core::str::from_utf8(&cx.local.buffer[..*cx.local.index]) {
                let mut text_box =
                    TextBox::with_textbox_style(str_rd, bounds, character_style, textbox_style);

                let height: i32 = text_box.bounding_box().size.height.try_into().unwrap();
                let offset = 135 - height;
                if offset < 0 {
                    text_box.set_vertical_offset(offset);
                }

                text_box.draw(cx.local.display).unwrap();
            } else {
                *cx.local.index = 0;
            }
        }

        update_display::spawn_after(250_u64.millis()).unwrap();
    }

    #[task(binds = UART1_IRQ, local = [uart1, uart_rx_write, uart_tx_read])]
    fn uart1(cx: uart1::Context) {
        let uart = cx.local.uart1;

        while uart.uart_is_readable() {
            if cx.local.uart_rx_write.ready() {
                if let Ok(byte) = uart.read() {
                    cx.local.uart_rx_write.enqueue(byte).unwrap();
                }
            } else {
                break;
            }
        }

        if uart.uart_is_writable() {
            while let Some(byte) = cx.local.uart_tx_read.peek().cloned() {
                if uart.write(byte).is_ok() {
                    cx.local.uart_tx_read.dequeue();
                } else {
                    break;
                }
            }
        }
    }

    fn tft_display_setup(
        tft_bl: Pin<Gpio4, Disabled<PullDown>>,
        tft_rst: Pin<Gpio0, Disabled<PullDown>>,
        tft_dc: Pin<Gpio1, Disabled<PullDown>>,
        tft_sclk: Pin<Gpio2, Disabled<PullDown>>,
        tft_mosi: Pin<Gpio3, Disabled<PullDown>>,
        tft_csn: Pin<Gpio5, Disabled<PullDown>>,
        spi0: pac::SPI0,
        resets: &mut pac::RESETS,
        clocks: &ClocksManager,
        delay: &mut cortex_m::delay::Delay,
    ) -> ST7789<SPIIntType, Pin<Gpio0, Output<PushPull>>, Pin<Gpio4, Output<PushPull>>> {
        //TFT
        let mut backlight = tft_bl.into_push_pull_output();
        backlight.set_high().unwrap();
        let rst = tft_rst.into_push_pull_output();
        let dc = tft_dc.into_push_pull_output();

        let _spi_sclk = tft_sclk.into_mode::<gpio::FunctionSpi>();
        let _spi_mosi = tft_mosi.into_mode::<gpio::FunctionSpi>();
        let spi_cs = tft_csn.into_push_pull_output();

        let spi = spi::Spi::<_, _, 8>::new(spi0);
        let spi = spi.init(
            resets,
            clocks.peripheral_clock.freq(),
            (30u32 << 20u32).Hz(),
            &embedded_hal::spi::MODE_3,
        );

        let di = SPIInterface::new(spi, dc, spi_cs);

        let mut display = ST7789::new(di, Some(rst), Some(backlight), 280, 188);
        display.init(delay).unwrap();
        display
            .set_orientation(st7789::Orientation::Landscape)
            .unwrap();
        display
    }

    fn uart_setup(
        uart_pins: UartPins,
        device: pac::UART1,
        clocks: &ClocksManager,
        resets: &mut pac::RESETS,
    ) -> UartPeripheral<Enabled, hal::pac::UART1, UartPins> {
        let mut uart = UartPeripheral::new(device, uart_pins, resets)
            .enable(
                hal::uart::common_configs::_115200_8_N_1,
                clocks.peripheral_clock.freq(),
            )
            .unwrap();

        uart.enable_rx_interrupt();
        uart.enable_tx_interrupt();

        uart
    }
}

// End of file
