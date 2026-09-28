//! A76xx modem over UART with an asynchronous ESP-NETIF PPP bridge.
//!
//! Set `CELLULAR_APN` at build time. Adjust the UART and power pins for your
//! board, and implement its modem power-key sequence in `ModemPower` below.
//! The A76xx dependency tracks DaneSlattery's fork while its PPP changes are
//! being tested before upstreaming.

#![allow(unexpected_cfgs)]

#[cfg(esp_idf_lwip_ppp_support)]
mod example {
    use core::future::pending;
    use std::sync::mpsc;
    use std::time::Duration;

    use a76xx::{Error as ModemError, ModemPower, ModemResources};
    use edge_executor::LocalExecutor;
    use embassy_time::{Duration as EmbassyDuration, Timer};
    use esp_idf_svc::eventloop::EspSystemEventLoop;
    use esp_idf_svc::hal::gpio::{self, Output, PinDriver};
    use esp_idf_svc::hal::peripherals::Peripherals;
    use esp_idf_svc::hal::uart::{AsyncUartDriver, UartConfig};
    use esp_idf_svc::hal::units::Hertz;
    use esp_idf_svc::handle::RawHandle;
    use esp_idf_svc::netif::{
        AsyncEspNetifChannel, EspNetif, IpEvent, NetifStack, PppConfiguration,
    };
    use static_cell::ConstStaticCell;

    const APN: &str = env!(
        "CELLULAR_APN",
        "set CELLULAR_APN to your SIM provider's APN"
    );
    const DIAL_NUMBER: &str = match option_env!("CELLULAR_DIAL") {
        Some(number) => number,
        None => "*99#",
    };

    type Resources = ModemResources<2048, 2048, 512, 512>;
    static MODEM_RESOURCES: ConstStaticCell<Resources> =
        ConstStaticCell::new(ModemResources::new());

    struct BoardPower {
        enable: PinDriver<'static, Output>,
        power_key: PinDriver<'static, Output>,
    }

    impl ModemPower for BoardPower {
        async fn power_on(&mut self) -> Result<(), ModemError> {
            self.enable
                .set_high()
                .map_err(|_| ModemError::PowerOnError)?;
            Timer::after(EmbassyDuration::from_millis(100)).await;
            self.power_key
                .set_high()
                .map_err(|_| ModemError::PowerOnError)?;
            Timer::after(EmbassyDuration::from_millis(100)).await;
            self.power_key
                .set_low()
                .map_err(|_| ModemError::PowerOnError)?;
            Timer::after(EmbassyDuration::from_secs(10)).await;
            Ok(())
        }
    }

    pub fn run() -> anyhow::Result<()> {
        esp_idf_svc::sys::link_patches();
        esp_idf_svc::log::EspLogger::initialize_default();

        let peripherals = Peripherals::take()?;
        let system_loop = EspSystemEventLoop::take()?;
        let power = BoardPower {
            enable: PinDriver::output(peripherals.pins.gpio2)?,
            power_key: PinDriver::output(peripherals.pins.gpio4)?,
        };
        let mut uart = AsyncUartDriver::new(
            peripherals.uart1,
            peripherals.pins.gpio17,
            peripherals.pins.gpio18,
            Option::<gpio::Gpio0>::None,
            Option::<gpio::Gpio0>::None,
            &UartConfig::default().baudrate(Hertz(115_200)),
        )?;
        let (mut modem, rx_pump, tx_pump) = a76xx::Modem::new(MODEM_RESOURCES.take(), power);

        let mut bridge =
            AsyncEspNetifChannel::<_, 8>::new(EspNetif::new(NetifStack::Ppp)?, |netif| {
                netif.set_ppp_conf(&PppConfiguration::default())
            })?;
        let handle = bridge.driver().netif().handle() as usize;
        let (ip_sender, ip_receiver) = mpsc::sync_channel(1);
        let _subscription = system_loop.subscribe::<IpEvent, _>(move |event| {
            if event.is_for_handle(handle as *mut _) && matches!(event, IpEvent::DhcpIpAssigned(_))
            {
                let _ = ip_sender.try_send(());
            }
        })?;
        bridge.driver_mut().start()?;

        std::thread::Builder::new()
            .name("ppp-ip-monitor".into())
            .stack_size(8 * 1024)
            .spawn(
                move || match ip_receiver.recv_timeout(Duration::from_secs(120)) {
                    Ok(()) => log::info!("PPP has an IPv4 address"),
                    Err(error) => log::error!("PPP address wait failed: {error}"),
                },
            )?;

        let executor = LocalExecutor::new();
        esp_idf_svc::hal::task::block_on(executor.run(async {
            let (uart_tx, uart_rx) = uart.split();
            executor.spawn(rx_pump.run(uart_rx)).detach();
            executor.spawn(tx_pump.run(uart_tx)).detach();

            if let Err(error) = modem.power_on().await {
                log::error!("modem power-on failed: {error:?}");
                pending::<()>().await;
            }
            modem.wait_for_connection().await;

            let mut ppp_io = match modem.connect_ppp_without_pin(APN, DIAL_NUMBER).await {
                Ok(io) => io,
                Err(error) => {
                    log::error!("modem PPP negotiation failed: {error:?}");
                    pending::<()>().await;
                    unreachable!()
                }
            };
            executor
                .spawn(async move {
                    let mut rx_buffer = [0_u8; 1600];
                    if let Err(error) = bridge.run(&mut ppp_io, &mut rx_buffer).await {
                        log::error!("PPP bridge stopped: {error}");
                    }
                })
                .detach();

            pending::<()>().await;
        }));
        unreachable!("the modem pumps run forever")
    }
}

#[cfg(esp_idf_lwip_ppp_support)]
fn main() -> anyhow::Result<()> {
    example::run()
}

#[cfg(not(esp_idf_lwip_ppp_support))]
fn main() {
    panic!("Enable CONFIG_LWIP_PPP_SUPPORT in sdkconfig.defaults");
}
