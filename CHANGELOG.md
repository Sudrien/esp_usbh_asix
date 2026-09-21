# Changelog

## v0.1.0 - unreleased

- **First hardware run** (AX88772A, `0b95:772a`, on an M5Stack Tab5 /
  ESP32-P4, IDF v5.5.5): enumerates, reads the MAC, links, gets a DHCP
  lease and carried a 512 kbit/s internet radio stream for minutes.
  Unplugging and replugging the cable works.
- Fix: a failed IN transfer killed receive for good. The endpoint is
  halted by the host stack on any transfer error; the callbacks
  resubmitted into it anyway, ignored the ESP_ERR_INVALID_STATE, and
  after one failure per RX transfer nothing was queued on the IN
  endpoint. Seen on the second board run as two "Enqueue URB error"
  lines and a link that never got an IPv4 address. Failures now post a
  recovery to the worker: halt, flush, clear, restart.
- Fix: two "EP command error" lines at every bring-up, from clearing
  endpoints that were never halted. Cleared only after a stop or a
  recovery now.
- Bulk IN is queued only while the link is up. Left queued with the
  cable out, or idle, RX transfers failed with a transfer error every
  one to five seconds (17 recoveries in a minute on the board, none in
  90 s of streaming). The interrupt endpoint stays queued throughout,
  since it is how link-up is learnt. A start straight after a stop
  waits for the stop's cancellations to return.

- Initial driver for AX88772 / AX88772A / AX88772B, implementing
  `iot_eth_driver_t`.
- Ported chip sequences from CherryUSB `usbh_asix.c`, with three fixes:
  AX88772A match-table entry, TX zero-length-packet padding test, RX
  odd-length frame padding.
- `skip_init_usb_host_driver` for applications that own the USB host
  stack.
- `examples/basic`.
- Compiles for ESP32-P4 on ESP-IDF v5.5.1; not yet run on hardware.
