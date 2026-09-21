# Changelog

## v0.1.0 - unreleased

- Initial driver for AX88772 / AX88772A / AX88772B, implementing
  `iot_eth_driver_t`.
- Ported chip sequences from CherryUSB `usbh_asix.c`, with three fixes:
  AX88772A match-table entry, TX zero-length-packet padding test, RX
  odd-length frame padding.
- `skip_init_usb_host_driver` for applications that own the USB host
  stack.
- `examples/basic`.
- Compiles for ESP32-P4 on ESP-IDF v5.5.1; not yet run on hardware.
