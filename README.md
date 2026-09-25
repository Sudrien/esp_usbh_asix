# esp_usbh_asix

USB host Ethernet driver for ASIX AX88772-family adapters on ESP-IDF.

The ASIX chips speak a vendor protocol, not CDC-ECM, so
`espressif/iot_usbh_ecm` does not pick them up. This component fills
that gap. It implements the `iot_eth_driver_t` interface from
[`espressif/iot_eth`](https://components.espressif.com/components/espressif/iot_eth),
so the netif glue, DHCP and everything above work unchanged.

## Status

**Runs on one chip.** An AX88772A has carried real traffic on an
ESP32-P4 (M5Stack Tab5, ESP-IDF v5.5.5): DHCP, a TLS audio stream for
minutes, cable unplug and replug. The other two chips have still not
been tried.

Tested at `5843b0c`, in [m5tab5_defeatist_music_player](https://github.com/Sudrien/m5tab5_defeatist_music_player)
on ESP-IDF v5.5.5, chip code `0x10`, internal PHY:

- DHCP and the default route; a TLS internet radio stream over the
  cable for about three and a half minutes, 35 ms of stall in total.
- Sharing the one USB host with `espressif/iot_usbh_ecm`, the USB mass
  storage and audio class drivers and a HID remote. A flash drive
  plugged in beside the adapter mounted, and the stream carried on.
- Pulling the adapter out mid-stream: link down, then adapter removed.
- Two things in the log that are noise, not faults (see the changelog):
  "bring-up failed: ESP_ERR_INVALID_STATE" when any other device
  enumerates while an adapter is attached, and "asix_start_rx ... int
  submit" when the adapter is pulled out.

| Chip     | VID:PID     | Chip code | Status |
|----------|-------------|-----------|--------|
| AX88772  | `0b95:7720` | `0x00`    | untested |
| AX88772A | `0b95:772a` | `0x10`    | works (ESP32-P4) |
| AX88772B | `0b95:772b` | `0x20`    | untested |

The AX88178/179 (gigabit) are a different command set and are not
supported. Rebadged adapters with other IDs can be added at runtime via
`extra_id_list`.

Targets: ESP32-S2, ESP32-S3, ESP32-P4 (anything with a USB OTG host).
Only the P4 has been built.

## Usage

```c
const esp_usbh_asix_config_t asix_cfg = {
    .skip_init_usb_host_driver = false,   /* true if your app owns usb_host */
    .task_coreid = -1,
};
iot_eth_driver_t *driver;
ESP_ERROR_CHECK(esp_usbh_asix_new_eth(&asix_cfg, &driver));

const iot_eth_config_t eth_cfg = { .driver = driver };
iot_eth_handle_t eth;
ESP_ERROR_CHECK(iot_eth_install(&eth_cfg, &eth));

esp_netif_inherent_config_t inherent = ESP_NETIF_INHERENT_DEFAULT_ETH();
const esp_netif_config_t netif_cfg = {
    .base = &inherent,
    .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH,
};
esp_netif_t *netif = esp_netif_new(&netif_cfg);
ESP_ERROR_CHECK(esp_netif_attach(netif, iot_eth_new_netif_glue(eth)));

ESP_ERROR_CHECK(iot_eth_start(eth));
```

A complete program is in [`examples/basic`](examples/basic). The adapter
can be plugged in before or after `iot_eth_start()`, and unplugged and
replugged.

### Sharing the bus

If your application already calls `usb_host_install()` -- because it also
uses mass storage, audio or HID on the same port -- set
`skip_init_usb_host_driver`. The driver then registers as one more
`usb_host` client and never installs or uninstalls the stack.

### Hubs

Adapters built into a hub work if hub support is on:

```
CONFIG_USB_HOST_HUBS_SUPPORTED=y
CONFIG_USB_HOST_HUB_MULTI_LEVEL=y
```

The AX88772 family is high-speed, so it is unaffected by ESP-IDF's
missing transaction translator, which stops full- and low-speed devices
behind a hub from enumerating.

## Configuration

| Kconfig | Default | |
|---------|---------|---|
| `ASIX_RX_BUFFER_SIZE` | 2048 | Bytes per bulk IN transfer. The chip is told the matching maximum burst. |
| `ASIX_RX_TRANSFER_COUNT` | 2 | Bulk IN transfers kept in flight. |

## Design notes

- **Two tasks.** In ESP-IDF, transfer callbacks run inside
  `usb_host_client_handle_events()`. Setting up the chip needs many
  blocking control transfers, and acting on a link change writes a
  register. Doing either in a callback would deadlock the task that has
  to deliver the completion. So one task only pumps events, and a worker
  task does anything that blocks.
- **RX copies each frame.** `iot_eth` hands frames to esp_netif, which
  later `free()`s them. Each frame is therefore copied out of the USB
  buffer into its own allocation, as `iot_usbh_ecm` does.
- **Unplug waits 50 ms** for cancelled transfers to be returned before
  freeing them. A counted drain would be tighter.
- **Link is assumed 100 Mbit full duplex** when the chip reports link up,
  as in the donor driver. There is no autonegotiation readback yet.

## Provenance

The chip-level sequences are ported from CherryUSB's
[`class/vendor/net/usbh_asix.c`](https://github.com/cherry-embedded/CherryUSB/blob/master/class/vendor/net/usbh_asix.c)
by sakumisu, Apache-2.0. That covers the reset order, the PHY selection,
the AX88772A PHY register restore, and the medium-mode and RX_CTL values.
The transport layer (transfers, tasks, enumeration, lifetime) is
rewritten against ESP-IDF's `usb_host` API.

Three fixes relative to the donor, each commented where it occurs:

1. **`0b95:772a` added to the match table.** CherryUSB lists only `7720`
   and `772b`, so a plain AX88772A never binds there, even though the
   reset code handles it.
2. **TX zero-length-packet padding.** The donor's test,
   `!(buflen + 4) % mps`, parses as `(!(buflen + 4)) % mps`. That is 0
   for every non-zero length, so the padding is never appended.
3. **RX odd-length padding.** The donor advances by `len + 4`, which
   misreads every frame after an odd-length one in the same transfer.
   The pad byte is confirmed by FreeBSD's `if_axe.c`
   (`pos += len + (len % 2)`, BSD licence).

The Linux `asix` driver (GPL) was not used as a reference.

This code was written by Claude (Anthropic), an AI model, and the commits
are authored that way. Nobody has warranted it as their own work.

## Licence

Apache-2.0. See [LICENSE](LICENSE).
