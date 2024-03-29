//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

pub mod bsp;
mod adc;

#[rtic::app(device = crate::bsp::hal::pac, peripherals = true, dispatchers = [PIO0_IRQ_0, PIO0_IRQ_1])]
mod app {
    use crate::bsp::hal;
    use crate::adc;
    use picoc3_audio_stream::audio_packet::AudioPacket;
    use esp_at_nal::wifi::JoinError;
    use rp2040_pwm_timer::{ExtDrivenTimer, PwmTimerDriver};
    use core::convert::TryInto;
    use core::sync::atomic::{AtomicU32};
    use core::str::FromStr;
    use defmt::{info, error};
    use defmt_rtt as _;
    use display_interface_spi::SPIInterface;
    use embedded_graphics::{
        mono_font::{MonoTextStyle},
        pixelcolor::{self, Rgb565},
        prelude::*,
        primitives::Rectangle,
    };
    use embedded_hal::digital::v2::OutputPin;
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
        gpio::pin::bank0::{Gpio0, Gpio1, Gpio2, Gpio3, Gpio4, Gpio5, Gpio8, Gpio9, Gpio10, Gpio11, Gpio12, Gpio13},
        gpio::{Output, Pin, PullDown, PushPull},
        gpio::{Disabled, Function, Uart},
        pac::{self, UART1},
        pwm::{Slices, Pwm6, Pwm7},
        sio::Sio,
        spi,
        uart::{Enabled, UartPeripheral, UartConfig, DataBits, StopBits, Reader, Writer},
        watchdog::Watchdog,
    };
    use profont::PROFONT_10_POINT;
    use st7789::ST7789;
    use atat::{Client, ClientBuilder, Queues, bbqueue::BBBuffer, IngressManager, AtDigester};
    use embedded_nal::{SocketAddr, UdpClientStack};
    use esp_at_nal::{
        urc::URCMessages,
        wifi::{Adapter, WifiAdapter},
        stack::Socket,    
    };

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Mono = Rp2040Monotonic;

    type SPIIntType = SPIInterface<
        spi::Spi<spi::Enabled, pac::SPI0, 8>,
        Pin<Gpio1, Output<PushPull>>,
        Pin<Gpio5, Output<PushPull>>,
    >;

    type Uart0Pins = (Pin<Gpio12, Function<Uart>>, Pin<Gpio13, Function<Uart>>);
    type Uart1Pins = (Pin<Gpio8, Function<Uart>>, Pin<Gpio9, Function<Uart>>, Pin<Gpio10, Function<Uart>>, Pin<Gpio11, Function<Uart>>);

    const TX_BUFFER_BYTES: usize = 1536;
    const RX_BUFFER_BYTES: usize = 2048;
    const RES_CAPACITY_BYTES: usize = RX_BUFFER_BYTES;
    const URC_CAPACITY_BYTES: usize = RX_BUFFER_BYTES * 3;

    #[shared]
    struct Shared {
        ingress: IngressManager<
            AtDigester<URCMessages<RX_BUFFER_BYTES>>,
            RX_BUFFER_BYTES,
            RES_CAPACITY_BYTES,
            URC_CAPACITY_BYTES
        >,
        #[lock_free]
        uart0: UartPeripheral<Enabled, pac::UART0, Uart0Pins>,
        #[lock_free]
        adapter: Adapter<
            Client<Writer<UART1, Uart1Pins>, ExtDrivenTimer<'static, 1000>, 1000, RES_CAPACITY_BYTES, URC_CAPACITY_BYTES>,
            ExtDrivenTimer<'static, 1000>, 1000, TX_BUFFER_BYTES, RX_BUFFER_BYTES
        >,
        #[lock_free]
        socket: Option<Socket>,
        adc: adc::DmaAdc,
    }

    #[local]
    struct Local {
        display: ST7789<SPIIntType, Pin<Gpio0, Output<PushPull>>, Pin<Gpio4, Output<PushPull>>>,
        esp32_uart_rx: Reader<UART1, Uart1Pins>,
        pwm1_timer_driver: PwmTimerDriver<'static, Pwm6>,
        pwm2_timer_driver: PwmTimerDriver<'static, Pwm7>,
    }

    #[init(local = [])]
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

        //Timers
        let mut delay =
            cortex_m::delay::Delay::new(cx.core.SYST, clocks.system_clock.freq().raw());
        let mono = Rp2040Monotonic::new(cx.device.TIMER);

        let pwm_slices = Slices::new(cx.device.PWM, &mut resets);

        static PWM1_OVERFLOWS: AtomicU32 = AtomicU32::new(0);
        let pwm1_timer_driver =
            PwmTimerDriver::new(pwm_slices.pwm6, &PWM1_OVERFLOWS, clocks.peripheral_clock.freq().raw());
        let pwm_timer1 = ExtDrivenTimer::new(&PWM1_OVERFLOWS);

        static PWM2_OVERFLOWS: AtomicU32 = AtomicU32::new(0);
        let pwm2_timer_driver =
            PwmTimerDriver::new(pwm_slices.pwm7, &PWM2_OVERFLOWS, clocks.peripheral_clock.freq().raw());
        let pwm_timer2 = ExtDrivenTimer::new(&PWM2_OVERFLOWS);

        let pins = crate::bsp::Pins::new(
            cx.device.IO_BANK0,
            cx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        //UART1 (ESP32-AT)
        let uart_pins = (
            pins.gpio8.into_mode::<gpio::FunctionUart>(),
            pins.gpio9.into_mode::<gpio::FunctionUart>(),
            pins.gpio10.into_mode::<gpio::FunctionUart>(),
            pins.gpio11.into_mode::<gpio::FunctionUart>(),
        );
        let mut uart1 = UartPeripheral::new(cx.device.UART1, uart_pins, &mut resets)
            .enable(
                UartConfig::new(2_000_000.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq()
            ).unwrap();
        uart1.set_fifos(true);
        uart1.enable_rx_interrupt();
        let (rx, tx) = uart1.split();

        //UART0 (PICOPROBE)
        let uart0_pins = (
            pins.gpio12.into_mode::<gpio::FunctionUart>(),
            pins.gpio13.into_mode::<gpio::FunctionUart>(),
        );
        let uart0 = 
            UartPeripheral::new(cx.device.UART0, uart0_pins, &mut resets)
            .enable(
                UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq()
            ).unwrap();

        //ADC
        pins.gpio27.into_floating_input();
        let adc = adc::DmaAdc::new(cx.device.ADC, cx.device.DMA, &mut resets);

        //Display
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
        display.clear(pixelcolor::Rgb565::BLACK).unwrap();

        //ATAT
        static mut RES_QUEUE: BBBuffer<RES_CAPACITY_BYTES> = BBBuffer::new();
        static mut URC_QUEUE: BBBuffer<URC_CAPACITY_BYTES> = BBBuffer::new();
        let queues = Queues {
            res_queue: unsafe { RES_QUEUE.try_split_framed().unwrap() },
            urc_queue: unsafe { URC_QUEUE.try_split_framed().unwrap() }
        };

        let (client, ingress) = ClientBuilder::new(
            tx,
            pwm_timer1,
            AtDigester::<URCMessages<RX_BUFFER_BYTES>>::new(),
            atat::Config::new(atat::Mode::Timeout),
        )
        .build(queues);

        let adapter: Adapter<_, _, 1_000, TX_BUFFER_BYTES, RX_BUFFER_BYTES> = Adapter::new(client, pwm_timer2);

        at_digest::spawn_after(500.millis()).unwrap();
        restart_esp32::spawn_after(500.millis()).unwrap();

        (
            Shared {
                ingress,
                uart0,
                adapter,
                socket: None,
                adc,
            },
            Local {
                display,
                esp32_uart_rx: rx,
                pwm1_timer_driver,
                pwm2_timer_driver,
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(shared = [adapter])]
    fn restart_esp32(ctx: restart_esp32::Context) {
        let adapter = ctx.shared.adapter;
        info!("Restarting ESP32 Module...");
        if let Err(_) = adapter.restart() {
            info!("Restart Failed, trying again...");
            restart_esp32::spawn_after(1.secs()).unwrap();
            return;
        }
        info!("Done");
        wifi_connect::spawn_after(3.secs()).unwrap();
    }

    #[task(shared = [adapter, socket])]
    fn wifi_connect(ctx: wifi_connect::Context) {
        let adapter = ctx.shared.adapter;

        let response = adapter.get_current_uart_config().unwrap();
        info!("UART Config: {},{},{},{},{}", 
            response.baudrate, response.databits, response.parity, response.stopbits, response.flow_control);

        let mut state = adapter.get_join_status();
        if !state.connected {
            match adapter.join("Home 1", "1234554321") {
                Ok(s) => {
                    state = s;
                },
                Err(e) => {
                    match e {
                        JoinError::ConnectError(_) => info!("Connect Error"),
                        JoinError::ModeError(_) => info!("Mode Error"),
                        _ => info!("Unknown Join Error"),
                    }
                    info!("WiFI failed to join, retrying...");
                    wifi_connect::spawn_after(3.secs()).unwrap();
                    return;
                }
            }
        }
        info!("WiFi Connection State: {}", state.connected);

        let address = adapter.get_address().unwrap();
        if let Some(ipv4) = address.ipv4 {
            info!("IP Address: {}", ipv4.octets());
        }
        if let Some(mac) = address.mac {
            info!("MAC Address: {}", mac.as_str());
        }

        match adapter.socket() {
            Ok(mut socket) => {
                match adapter.connect(&mut socket, SocketAddr::from_str("192.168.1.184:5555").unwrap()) {
                    Ok(_) => {
                        info!("UDP connection successful (192.168.1.184:5555)");
                        ctx.shared.socket.replace(socket);
                        read_adc::spawn().unwrap();
                    },
                    Err(_e) => {
                        error!("Connection error");
                    }
                };
            },
            Err(_) => error!("Socket error")
        }
    }

    #[task(shared = [uart0, adapter, socket, adc])]
    fn read_adc(cx: read_adc::Context) {
        let len = minicbor::len(unsafe { &adc::AUDIO_PACKET_ONE });
        let mut buffer = [0u8; 1024];

        minicbor::encode(unsafe { &adc::AUDIO_PACKET_ONE }, buffer[..len].as_mut()).unwrap();

        if let Err(_e) = cx.shared.adapter.send(cx.shared.socket.as_mut().unwrap(), &buffer[..len]) {
            info!("Send failed");
        }

        read_adc::spawn_at(monotonics::now() + 10_u64.millis()).unwrap();
    }

    #[task(local = [display,
                    index: usize = 0, 
                    buffer: [u8; 256] = [0; 256],
                   ],
            shared = [uart0]
          )]
    fn update_display(cx: update_display::Context) {
        let buffer_updated = false;

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

    #[task(priority = 3, binds = UART1_IRQ, shared = [ingress], local = [esp32_uart_rx])]
    fn uart1(mut cx: uart1::Context) {
        static mut BUFFER: [u8; 64] = [0; 64];

        let reader = cx.local.esp32_uart_rx;

        cx.shared.ingress.lock(|ingress| {
            unsafe {
                while let Ok(bytes_read) = reader.read_raw(&mut BUFFER) {
                    ingress.write(&BUFFER[..bytes_read]);
                }
            }
        });
    }

    #[task(priority = 2, shared = [ingress])]
    fn at_digest(mut cx: at_digest::Context) {
        cx.shared.ingress.lock(|ingress| {
            ingress.digest();
        });

        at_digest::spawn_after(10.millis()).unwrap();
    }

    #[task(priority = 3, binds = PWM_IRQ_WRAP, local = [pwm1_timer_driver, pwm2_timer_driver])]
    fn pwm_wrap(ctx: pwm_wrap::Context) {
        ctx.local.pwm1_timer_driver.on_wrap();
        ctx.local.pwm2_timer_driver.on_wrap();
    }

    #[task(priority = 3, binds = DMA_IRQ_0, shared = [adc])]
    fn dma_irq_0(ctx: dma_irq_0::Context) {
        let mut adc = ctx.shared.adc;
        info!("DMA Transfer complete!");
        adc.lock(|adc| {
            adc.clear_interrupt();
        });
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
}

// End of file
