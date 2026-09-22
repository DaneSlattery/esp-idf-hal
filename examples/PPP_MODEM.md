# PPP modem integration

Modem-specific initialization belongs in a modem driver. `esp-idf-svc` only
bridges the resulting PPP byte stream to ESP-NETIF.

The [`a76xx`](https://github.com/jarkonik/a76xx) driver provides an example of
this separation. It performs the AT-command negotiation and returns a
bidirectional `PppIo` implementing `embedded-io-async` after the modem enters
PPP data mode.

The relevant handoff is:

```rust,ignore
use esp_idf_svc::netif::{
    AsyncEspNetifDriver, EspNetif, NetifStack, PppConfiguration,
};

// Initialize the modem and its UART pumps as described by a76xx, then enter
// PPP data mode. The modem driver remains responsible for SIM, APN, dialing,
// and power sequencing.
let mut ppp_io = modem.connect_ppp(pin, apn, "*99#").await?;

let mut bridge = AsyncEspNetifDriver::<_, 8>::new(
    EspNetif::new(NetifStack::Ppp)?,
    |netif| netif.set_ppp_conf(&PppConfiguration::default()),
)?;

bridge.driver_mut().start()?;

// `run` forwards both directions until the transport closes or an error is
// returned. It can be spawned while the application waits for
// IP_EVENT_PPP_GOT_IP and then opens sockets through lwIP.
let mut rx_buffer = [0; 1536];
bridge.run(&mut ppp_io, &mut rx_buffer).await?;
```

## Hardware test

1. Connect the modem's UART and power-control pins and configure them for the
   board being tested.
2. Start the modem driver's UART RX and TX pumps.
3. Ask the modem driver to enter PPP mode with the SIM PIN, APN, and dial
   string required by the network.
4. Construct and start `AsyncEspNetifDriver`, then run the bridge as above.
5. Confirm that `IP_EVENT_PPP_GOT_IP` is received and contains a non-zero IPv4
   address.
6. Make a DNS lookup and an HTTP request through the normal ESP-IDF/lwIP socket
   APIs.
7. Disconnect the cellular network and confirm that `IP_EVENT_PPP_LOST_IP` is
   received and the bridge can be stopped without an invalid-state error.

The current `a76xx` release documents A7670-family support. SIM7600
compatibility must be verified in that driver (and any required support added
there) rather than encoded in `esp-idf-svc`.
