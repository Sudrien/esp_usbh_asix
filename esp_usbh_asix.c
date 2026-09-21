/*
 * SPDX-FileCopyrightText: 2026 Sudrien
 * SPDX-FileCopyrightText: 2024 sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * ASIX AX88772/AX88772A/AX88772B USB Ethernet host driver for ESP-IDF.
 *
 * The chip sequences -- reset order, PHY selection, the AX88772A PHY
 * register restore, the medium-mode and RX_CTL writes, the 4 byte frame
 * headers -- are ported from CherryUSB's class/vendor/net/usbh_asix.c
 * (https://github.com/cherry-embedded/CherryUSB), Apache-2.0, by
 * sakumisu. That driver sits on CherryUSB's own host stack; everything
 * here below the chip sequences -- transfers, tasks, enumeration,
 * lifetime -- is rewritten against ESP-IDF's usb_host API and the
 * espressif/iot_eth driver interface.
 *
 * Deliberate differences from the donor, all of them noted again at the
 * point they occur:
 *
 *   - 0x0b95:0x772a (AX88772A) is in the match table. CherryUSB lists
 *     only 0x7720 and 0x772b, so a plain AX88772A never binds there,
 *     even though its reset path is implemented.
 *   - The TX zero-length-packet test is `((len + 4) % mps) == 0`.
 *     CherryUSB has `!(buflen + 4) % mps`, which parses as
 *     `(!(buflen + 4)) % mps` and is therefore 0 for every non-zero
 *     length -- the padding never gets appended. See asix_transmit().
 *   - RX advances past a pad byte after odd-length frames. CherryUSB
 *     advances by `len + 4`, which misaligns every later frame in the
 *     same bulk transfer; FreeBSD's if_axe.c (BSD licence) confirms the
 *     padding. See rx_cb().
 *   - Buffers are heap allocated per device rather than static, because
 *     a component cannot assume it is the only user of the bus.
 */

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_check.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "usb/usb_host.h"

#include "esp_usbh_asix.h"
#include "esp_usbh_asix_reg.h"
#include "iot_eth_interface.h"
#include "iot_eth_types.h"

static const char *TAG = "asix";

#define ASIX_VID                0x0b95

#define ETH_ALEN                6
#define ETH_MAX_FRAME           1518

#define ASIX_RX_BUF_SIZE        CONFIG_ASIX_RX_BUFFER_SIZE
#define ASIX_RX_XFER_COUNT      CONFIG_ASIX_RX_TRANSFER_COUNT
#define ASIX_TX_BUF_SIZE        (ETH_MAX_FRAME + 8)
#define ASIX_INT_BUF_SIZE       8

#define ASIX_CTRL_BUF_SIZE      (USB_SETUP_PACKET_SIZE + 64)
#define ASIX_CTRL_TIMEOUT_MS    1000
#define ASIX_TX_TIMEOUT_MS      1000

#define ASIX_DEFAULT_STACK      4096
#define ASIX_DEFAULT_PRIO       5

/* Worker commands. The client event callback must not block, and the
 * event task must stay free to pump completions, so device setup and
 * teardown are handed to a second task. */
typedef enum {
    ASIX_CMD_NEW_DEV,
    ASIX_CMD_DEV_GONE,
    ASIX_CMD_LINK,      /* Link change seen on the interrupt endpoint */
} asix_cmd_type_t;

typedef struct {
    asix_cmd_type_t type;
    uint8_t addr;
    usb_device_handle_t dev;
    bool link_up;
} asix_cmd_t;

typedef struct {
    iot_eth_driver_t base;          /* Must be first: the handle is cast to it */
    esp_usbh_asix_config_t cfg;

    iot_eth_mediator_t *mediator;

    usb_host_client_handle_t client;
    TaskHandle_t event_task;
    TaskHandle_t worker_task;
    QueueHandle_t cmd_queue;
    volatile bool running;

    /* Attached device state. Valid only while `attached`. */
    usb_device_handle_t dev;
    uint8_t dev_addr;
    uint8_t itf_num;
    uint8_t ep_in;
    uint8_t ep_out;
    uint8_t ep_int;
    uint16_t mps_out;

    usb_transfer_t *ctrl_xfer;
    usb_transfer_t *tx_xfer;
    usb_transfer_t *int_xfer;
    usb_transfer_t *rx_xfer[ASIX_RX_XFER_COUNT];

    SemaphoreHandle_t ctrl_done;
    SemaphoreHandle_t tx_done;
    SemaphoreHandle_t tx_lock;

    uint8_t mac[ETH_ALEN];
    uint8_t phy_addr;
    uint8_t chipcode;
    bool embd_phy;

    bool attached;      /* Device open, endpoints known */
    bool started;       /* iot_eth has called start() */
    bool link_up;
} asix_t;

/* ------------------------------------------------------------------ */
/* Control transfers                                                   */
/* ------------------------------------------------------------------ */

static void ctrl_cb(usb_transfer_t *xfer)
{
    asix_t *asix = (asix_t *)xfer->context;
    xSemaphoreGive(asix->ctrl_done);
}

/* One vendor request, synchronous.
 *
 * Only the worker task calls this. It blocks on a semaphore that the
 * event task gives from the completion callback, so calling it from the
 * event task itself would deadlock. */
static esp_err_t asix_ctrl(asix_t *asix, bool dir_in, uint8_t cmd,
                           uint16_t value, uint16_t index,
                           void *data, uint16_t size)
{
    if (!asix->attached) {
        return ESP_ERR_INVALID_STATE;
    }
    if (size > ASIX_CTRL_BUF_SIZE - USB_SETUP_PACKET_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    usb_transfer_t *xfer = asix->ctrl_xfer;
    usb_setup_packet_t *setup = (usb_setup_packet_t *)xfer->data_buffer;

    setup->bmRequestType = (dir_in ? USB_BM_REQUEST_TYPE_DIR_IN : USB_BM_REQUEST_TYPE_DIR_OUT) |
                           USB_BM_REQUEST_TYPE_TYPE_VENDOR |
                           USB_BM_REQUEST_TYPE_RECIP_DEVICE;
    setup->bRequest = cmd;
    setup->wValue = value;
    setup->wIndex = index;
    setup->wLength = size;

    if (!dir_in && data && size) {
        memcpy(xfer->data_buffer + USB_SETUP_PACKET_SIZE, data, size);
    }

    xfer->device_handle = asix->dev;
    xfer->bEndpointAddress = 0;
    xfer->num_bytes = USB_SETUP_PACKET_SIZE + size;
    xfer->timeout_ms = ASIX_CTRL_TIMEOUT_MS;
    xfer->callback = ctrl_cb;
    xfer->context = asix;

    esp_err_t err = usb_host_transfer_submit_control(asix->client, xfer);
    if (err != ESP_OK) {
        return err;
    }

    if (xSemaphoreTake(asix->ctrl_done, pdMS_TO_TICKS(ASIX_CTRL_TIMEOUT_MS * 2)) != pdTRUE) {
        ESP_LOGE(TAG, "control request 0x%02x timed out", cmd);
        return ESP_ERR_TIMEOUT;
    }

    if (xfer->status != USB_TRANSFER_STATUS_COMPLETED) {
        ESP_LOGE(TAG, "control request 0x%02x failed, status %d", cmd, xfer->status);
        return ESP_FAIL;
    }

    if (dir_in && data && size) {
        const int got = xfer->actual_num_bytes - USB_SETUP_PACKET_SIZE;
        if (got < size) {
            ESP_LOGE(TAG, "control request 0x%02x short: %d of %u", cmd, got, size);
            return ESP_ERR_INVALID_RESPONSE;
        }
        memcpy(data, xfer->data_buffer + USB_SETUP_PACKET_SIZE, size);
    }

    return ESP_OK;
}

static esp_err_t asix_read_cmd(asix_t *asix, uint8_t cmd, uint16_t value,
                               uint16_t index, void *data, uint16_t size)
{
    return asix_ctrl(asix, true, cmd, value, index, data, size);
}

static esp_err_t asix_write_cmd(asix_t *asix, uint8_t cmd, uint16_t value,
                                uint16_t index, void *data, uint16_t size)
{
    return asix_ctrl(asix, false, cmd, value, index, data, size);
}

/* ------------------------------------------------------------------ */
/* MII, reset and mode helpers -- the chip sequences                   */
/* ------------------------------------------------------------------ */

/* Take the MII bus away from the hardware state machine.
 *
 * The retry loop is the donor's: the chip does not always hand the bus
 * over on the first ask, and AX_HOST_EN in the status register is how it
 * says it has. */
static esp_err_t asix_mii_claim(asix_t *asix)
{
    for (int i = 0; i < 10; i++) {
        esp_err_t err = asix_write_cmd(asix, AX_CMD_SET_SW_MII, 0, 0, NULL, 0);
        if (err != ESP_OK) {
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(1));

        uint8_t smsr = 0;
        err = asix_read_cmd(asix, AX_CMD_STATMNGSTS_REG, 0, 0, &smsr, 1);
        if (err != ESP_OK) {
            return err;
        }
        if (smsr & AX_HOST_EN) {
            return ESP_OK;
        }
    }
    ESP_LOGW(TAG, "MII bus did not hand over; continuing anyway");
    return ESP_OK;
}

static esp_err_t asix_mdio_write(asix_t *asix, uint8_t phy_id, uint8_t loc, uint16_t val)
{
    ESP_RETURN_ON_ERROR(asix_mii_claim(asix), TAG, "mii claim");
    ESP_RETURN_ON_ERROR(asix_write_cmd(asix, AX_CMD_WRITE_MII_REG, phy_id, loc, &val, 2),
                        TAG, "mii write");
    return asix_write_cmd(asix, AX_CMD_SET_HW_MII, 0, 0, NULL, 0);
}

static esp_err_t asix_mdio_read(asix_t *asix, uint8_t phy_id, uint8_t loc, uint16_t *out)
{
    ESP_RETURN_ON_ERROR(asix_mii_claim(asix), TAG, "mii claim");
    ESP_RETURN_ON_ERROR(asix_read_cmd(asix, AX_CMD_READ_MII_REG, phy_id, loc, out, 2),
                        TAG, "mii read");
    return asix_write_cmd(asix, AX_CMD_SET_HW_MII, 0, 0, NULL, 0);
}

static esp_err_t asix_sw_reset(asix_t *asix, uint8_t flags)
{
    return asix_write_cmd(asix, AX_CMD_SW_RESET, flags, 0, NULL, 0);
}

static esp_err_t asix_write_rx_ctl(asix_t *asix, uint16_t mode)
{
    return asix_write_cmd(asix, AX_CMD_WRITE_RX_CTL, mode, 0, NULL, 0);
}

static esp_err_t asix_write_medium_mode(asix_t *asix, uint16_t mode)
{
    return asix_write_cmd(asix, AX_CMD_WRITE_MEDIUM_MODE, mode, 0, NULL, 0);
}

static esp_err_t asix_write_gpio(asix_t *asix, uint16_t value, uint32_t settle_ms)
{
    esp_err_t err = asix_write_cmd(asix, AX_CMD_WRITE_GPIOS, value, 0, NULL, 0);
    if (settle_ms) {
        vTaskDelay(pdMS_TO_TICKS(settle_ms));
    }
    return err;
}

/* The RX_CTL value that matches the RX buffer size this component was
 * built with. The chip must not be allowed to burst more into a transfer
 * than the buffer holds. */
static uint16_t asix_rx_ctl_mfb(void)
{
#if ASIX_RX_BUF_SIZE >= 16384
    return AX_RX_CTL_MFB_16384;
#elif ASIX_RX_BUF_SIZE >= 8192
    return AX_RX_CTL_MFB_8192;
#elif ASIX_RX_BUF_SIZE >= 4096
    return AX_RX_CTL_MFB_4096;
#else
    return AX_RX_CTL_MFB_2048;
#endif
}

/* Accept broadcast and the standard multicast set. The filter value is
 * the donor's, which is the Linux driver's all-hosts set. */
static void asix_set_multicast(asix_t *asix)
{
    static const uint8_t multi_filter[AX_MCAST_FILTER_SIZE] = {
        0x00, 0x00, 0x20, 0x80, 0x00, 0x00, 0x00, 0x40
    };
    uint8_t filter[AX_MCAST_FILTER_SIZE];
    memcpy(filter, multi_filter, sizeof(filter));

    asix_write_cmd(asix, AX_CMD_WRITE_MULTI_FILTER, 0, 0, filter, sizeof(filter));
    asix_write_rx_ctl(asix, AX_DEFAULT_RX_CTL | AX_RX_CTL_AM | asix_rx_ctl_mfb());
}

static void asix_mac_link_down(asix_t *asix)
{
    asix_write_medium_mode(asix, 0);
}

static void asix_mac_link_up(asix_t *asix, int speed, bool duplex, bool tx_pause, bool rx_pause)
{
    uint16_t m = AX_MEDIUM_AC | AX_MEDIUM_RE;

    if (duplex) {
        m |= AX_MEDIUM_FD;
    }

    switch (speed) {
    case AX_SPEED_100:
        m |= AX_MEDIUM_PS;
        break;
    case AX_SPEED_10:
        break;
    default:
        return;
    }

    if (tx_pause) {
        m |= AX_MEDIUM_TFC;
    }
    if (rx_pause) {
        m |= AX_MEDIUM_RFC;
    }

    asix_write_medium_mode(asix, m);
}

/* Reset for the original AX88772 (chipcode 0x00). */
static esp_err_t ax88772_hw_reset(asix_t *asix)
{
    ESP_RETURN_ON_ERROR(asix_write_gpio(asix, AX_GPIO_RSE | AX_GPIO_GPO_2 | AX_GPIO_GPO2EN, 5),
                        TAG, "gpio");
    ESP_RETURN_ON_ERROR(asix_write_cmd(asix, AX_CMD_SW_PHY_SELECT, asix->embd_phy ? 1 : 0, 0, NULL, 0),
                        TAG, "phy select");

    if (asix->embd_phy) {
        ESP_RETURN_ON_ERROR(asix_sw_reset(asix, AX_SWRESET_IPPD), TAG, "reset ippd");
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_RETURN_ON_ERROR(asix_sw_reset(asix, AX_SWRESET_CLEAR), TAG, "reset clear");
        vTaskDelay(pdMS_TO_TICKS(60));
        ESP_RETURN_ON_ERROR(asix_sw_reset(asix, AX_SWRESET_IPRL | AX_SWRESET_PRL), TAG, "reset iprl");
    } else {
        ESP_RETURN_ON_ERROR(asix_sw_reset(asix, AX_SWRESET_IPPD | AX_SWRESET_PRL), TAG, "reset ext");
    }
    vTaskDelay(pdMS_TO_TICKS(150));

    ESP_RETURN_ON_ERROR(asix_write_rx_ctl(asix, AX_DEFAULT_RX_CTL), TAG, "rx ctl");
    ESP_RETURN_ON_ERROR(asix_write_medium_mode(asix, AX88772_MEDIUM_DEFAULT), TAG, "medium");
    ESP_RETURN_ON_ERROR(asix_write_cmd(asix, AX_CMD_WRITE_IPG0,
                                       AX88772_IPG0_DEFAULT | AX88772_IPG1_DEFAULT,
                                       AX88772_IPG2_DEFAULT, NULL, 0),
                        TAG, "ipg");
    ESP_RETURN_ON_ERROR(asix_write_cmd(asix, AX_CMD_WRITE_NODE_ID, 0, 0, asix->mac, ETH_ALEN),
                        TAG, "node id");

    return asix_write_rx_ctl(asix, AX_DEFAULT_RX_CTL | asix_rx_ctl_mfb());
}

/* Reset for the AX88772A (0x10) and AX88772B (0x20).
 *
 * The delays are the donor's and are not generous: a shorter wait after
 * the second IPRL leaves the PHY unresponsive to the MDIO reads that
 * follow. */
static esp_err_t ax88772a_hw_reset(asix_t *asix)
{
    ESP_RETURN_ON_ERROR(asix_write_gpio(asix, AX_GPIO_RSE, 5), TAG, "gpio");
    ESP_RETURN_ON_ERROR(asix_write_cmd(asix, AX_CMD_SW_PHY_SELECT,
                                       (asix->embd_phy ? 1 : 0) | AX_PHYSEL_SSEN, 0, NULL, 0),
                        TAG, "phy select");
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_RETURN_ON_ERROR(asix_sw_reset(asix, AX_SWRESET_IPPD | AX_SWRESET_IPRL), TAG, "reset 1");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(asix_sw_reset(asix, AX_SWRESET_IPRL), TAG, "reset 2");
    vTaskDelay(pdMS_TO_TICKS(160));
    ESP_RETURN_ON_ERROR(asix_sw_reset(asix, AX_SWRESET_CLEAR), TAG, "reset 3");
    ESP_RETURN_ON_ERROR(asix_sw_reset(asix, AX_SWRESET_IPRL), TAG, "reset 4");
    vTaskDelay(pdMS_TO_TICKS(200));

    if (asix->chipcode == AX_AX88772B_CHIPCODE) {
        ESP_RETURN_ON_ERROR(asix_write_cmd(asix, AX_QCTCTRL, 0x8000, 0x8001, NULL, 0),
                            TAG, "bq setting");
    } else if (asix->chipcode == AX_AX88772A_CHIPCODE) {
        /* Restore the three PHY registers if the reset left them off
         * their defaults. Reads that fail are skipped rather than fatal:
         * the adapter still works, it just may not link reliably. */
        static const struct {
            uint8_t reg;
            uint16_t def;
        } phy_defaults[] = {
            { AX88772A_PHY14H, AX88772A_PHY14H_DEFAULT },
            { AX88772A_PHY15H, AX88772A_PHY15H_DEFAULT },
            { AX88772A_PHY16H, AX88772A_PHY16H_DEFAULT },
        };

        for (size_t i = 0; i < sizeof(phy_defaults) / sizeof(phy_defaults[0]); i++) {
            uint16_t val = 0;
            if (asix_mdio_read(asix, asix->phy_addr, phy_defaults[i].reg, &val) != ESP_OK) {
                ESP_LOGW(TAG, "PHY reg 0x%02x read failed", phy_defaults[i].reg);
                continue;
            }
            if (val != phy_defaults[i].def) {
                ESP_LOGD(TAG, "PHY reg 0x%02x is 0x%04x, restoring 0x%04x",
                         phy_defaults[i].reg, val, phy_defaults[i].def);
                asix_mdio_write(asix, asix->phy_addr, phy_defaults[i].reg, phy_defaults[i].def);
            }
        }
    }

    ESP_RETURN_ON_ERROR(asix_write_cmd(asix, AX_CMD_WRITE_IPG0,
                                       AX88772_IPG0_DEFAULT | AX88772_IPG1_DEFAULT,
                                       AX88772_IPG2_DEFAULT, NULL, 0),
                        TAG, "ipg");
    ESP_RETURN_ON_ERROR(asix_write_cmd(asix, AX_CMD_WRITE_NODE_ID, 0, 0, asix->mac, ETH_ALEN),
                        TAG, "node id");
    ESP_RETURN_ON_ERROR(asix_write_rx_ctl(asix, AX_DEFAULT_RX_CTL), TAG, "rx ctl");
    ESP_RETURN_ON_ERROR(asix_write_medium_mode(asix, AX88772_MEDIUM_DEFAULT), TAG, "medium");

    return asix_write_rx_ctl(asix, AX_DEFAULT_RX_CTL | asix_rx_ctl_mfb());
}

/* ------------------------------------------------------------------ */
/* Data path                                                           */
/* ------------------------------------------------------------------ */

static void asix_report_link(asix_t *asix, bool up)
{
    if (asix->link_up == up) {
        return;
    }
    asix->link_up = up;

    ESP_LOGI(TAG, "link %s", up ? "up" : "down");

    if (up) {
        asix_mac_link_up(asix, AX_SPEED_100, true, true, true);
        asix_set_multicast(asix);
    } else {
        asix_mac_link_down(asix);
    }

    if (asix->mediator) {
        iot_eth_link_t link = up ? IOT_ETH_LINK_UP : IOT_ETH_LINK_DOWN;
        asix->mediator->on_stage_changed(asix->mediator, IOT_ETH_STAGE_LINK, &link);
    }
}

/* Interrupt endpoint: the chip reports PHY link state here. Byte 2
 * bit 0 is the link bit for the primary PHY. The transfer is resubmitted
 * from its own callback; the device NAKs until its polling interval has
 * elapsed, so this does not spin.
 *
 * The link change itself is NOT handled here. This callback runs inside
 * usb_host_client_handle_events() on the event task, and acting on a
 * link change means writing the medium-mode register -- a blocking
 * control transfer whose completion only the event task can deliver.
 * Doing it here would deadlock on the first cable plug. So the state is
 * posted to the worker, and only when it differs from what the worker
 * last acted on, so a chip that reports on every poll does not flood
 * the queue. */
static void int_cb(usb_transfer_t *xfer)
{
    asix_t *asix = (asix_t *)xfer->context;

    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED && xfer->actual_num_bytes >= 3 &&
            xfer->data_buffer[1] == 0x00) {
        const bool up = (xfer->data_buffer[2] & 0x01) != 0;
        if (up != asix->link_up) {
            const asix_cmd_t cmd = { .type = ASIX_CMD_LINK, .link_up = up };
            xQueueSend(asix->cmd_queue, &cmd, 0);
        }
    }

    if (asix->running && asix->attached && asix->started) {
        usb_host_transfer_submit(xfer);
    }
}

/* Hand one received frame to the stack.
 *
 * The stack takes ownership of what it is given: esp_netif later calls
 * free() on the pointer (iot_eth_netif_glue.c, eth_l2_free). So the
 * frame is copied out of the transfer buffer into its own allocation --
 * passing a pointer into the middle of a USB buffer would corrupt the
 * heap on the first free. iot_usbh_ecm does the same. */
static void asix_deliver(asix_t *asix, const uint8_t *frame, uint16_t len)
{
    iot_eth_mediator_t *m = asix->mediator;
    if (!m) {
        return;
    }

    uint8_t *buf = malloc(len);
    if (!buf) {
        ESP_LOGW(TAG, "rx: no memory for %u byte frame, dropped", len);
        return;
    }
    memcpy(buf, frame, len);

    if (m->stack_input_info) {
        m->stack_input_info(m, buf, len, NULL);
    } else if (m->stack_input) {
        m->stack_input(m, buf, len);
    } else {
        free(buf);
    }
}

/* Bulk IN: one transfer may carry several frames, each behind a 4 byte
 * header of length and its complement. A header that fails the
 * complement check means the stream is out of step, so the rest of the
 * buffer is dropped rather than guessed at. */
static void rx_cb(usb_transfer_t *xfer)
{
    asix_t *asix = (asix_t *)xfer->context;

    if (xfer->status != USB_TRANSFER_STATUS_COMPLETED) {
        if (xfer->status != USB_TRANSFER_STATUS_CANCELED) {
            ESP_LOGD(TAG, "rx transfer status %d", xfer->status);
        }
        goto resubmit;
    }

    int remaining = xfer->actual_num_bytes;
    uint8_t *p = xfer->data_buffer;

    while (remaining >= AX_FRAME_HDR_LEN) {
        const uint16_t len = ((uint16_t)p[0] | ((uint16_t)p[1] << 8)) & AX_FRAME_LEN_MASK;
        const uint16_t len_c = ((uint16_t)p[2] | ((uint16_t)p[3] << 8)) & AX_FRAME_LEN_MASK;

        if (len != ((uint16_t)~len_c & AX_FRAME_LEN_MASK)) {
            ESP_LOGW(TAG, "rx header mismatch, dropping %d bytes", remaining);
            break;
        }
        if (len == 0 || len > remaining - AX_FRAME_HDR_LEN) {
            break;
        }

        asix_deliver(asix, p + AX_FRAME_HDR_LEN, len);

        /* An odd-length frame is followed by one pad byte, so the next
         * header starts on an even offset. CherryUSB advances by
         * `len + 4` and so misreads every frame after an odd-length one
         * in the same transfer; FreeBSD's if_axe.c has
         * `pos += len + (len % 2)`, which is what this does. */
        const int consumed = AX_FRAME_HDR_LEN + len + (len & 1);
        remaining -= consumed;
        p += consumed;
    }

resubmit:
    if (asix->running && asix->attached && asix->started) {
        usb_host_transfer_submit(xfer);
    }
}

static void tx_cb(usb_transfer_t *xfer)
{
    asix_t *asix = (asix_t *)xfer->context;
    xSemaphoreGive(asix->tx_done);
}

static esp_err_t asix_transmit(iot_eth_driver_t *driver, uint8_t *data, size_t len)
{
    asix_t *asix = (asix_t *)driver;

    if (!asix->attached || !asix->started) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!asix->link_up) {
        return ESP_ERR_INVALID_STATE;
    }
    if (len == 0 || len > ETH_MAX_FRAME) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (xSemaphoreTake(asix->tx_lock, pdMS_TO_TICKS(ASIX_TX_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = ESP_OK;
    usb_transfer_t *xfer = asix->tx_xfer;
    uint8_t *buf = xfer->data_buffer;

    buf[0] = len & 0xff;
    buf[1] = (len >> 8) & 0xff;
    buf[2] = (uint8_t)~buf[0];
    buf[3] = (uint8_t)~buf[1];
    memcpy(buf + AX_FRAME_HDR_LEN, data, len);

    size_t total = len + AX_FRAME_HDR_LEN;

    /* A transfer whose length is an exact multiple of the endpoint's
     * max packet size needs something after it, or the chip waits for a
     * short packet that never comes. The donor appends this four byte
     * null header, but its test -- `!(buflen + 4) % mps` -- parses as
     * `(!(buflen + 4)) % mps`, which is 0 for every non-zero length, so
     * the padding is never actually appended there. Written out
     * properly here. */
    if (asix->mps_out && (total % asix->mps_out) == 0) {
        buf[total + 0] = 0x00;
        buf[total + 1] = 0x00;
        buf[total + 2] = 0xff;
        buf[total + 3] = 0xff;
        total += AX_FRAME_HDR_LEN;
    }

    xfer->device_handle = asix->dev;
    xfer->bEndpointAddress = asix->ep_out;
    xfer->num_bytes = total;
    xfer->timeout_ms = ASIX_TX_TIMEOUT_MS;
    xfer->callback = tx_cb;
    xfer->context = asix;

    err = usb_host_transfer_submit(xfer);
    if (err != ESP_OK) {
        goto out;
    }

    if (xSemaphoreTake(asix->tx_done, pdMS_TO_TICKS(ASIX_TX_TIMEOUT_MS * 2)) != pdTRUE) {
        ESP_LOGW(TAG, "tx timed out");
        err = ESP_ERR_TIMEOUT;
        goto out;
    }

    if (xfer->status != USB_TRANSFER_STATUS_COMPLETED) {
        ESP_LOGD(TAG, "tx status %d", xfer->status);
        err = ESP_FAIL;
    }

out:
    xSemaphoreGive(asix->tx_lock);
    return err;
}

/* ------------------------------------------------------------------ */
/* Device bring-up and teardown                                        */
/* ------------------------------------------------------------------ */

static bool asix_id_matches(asix_t *asix, uint16_t vid, uint16_t pid)
{
    /* The built-in table. CherryUSB's equivalent lists only 0x7720 and
     * 0x772b, which leaves a bare AX88772A unbound even though its reset
     * path is implemented there. */
    static const esp_usbh_asix_id_t builtin[] = {
        { ASIX_VID, 0x7720 },   /* AX88772  */
        { ASIX_VID, 0x772a },   /* AX88772A */
        { ASIX_VID, 0x772b },   /* AX88772B */
        { 0, 0 },
    };

    for (const esp_usbh_asix_id_t *id = builtin; id->vid; id++) {
        if (id->vid == vid && id->pid == pid) {
            return true;
        }
    }
    for (const esp_usbh_asix_id_t *id = asix->cfg.extra_id_list; id && id->vid; id++) {
        if (id->vid == vid && id->pid == pid) {
            return true;
        }
    }
    return false;
}

static void asix_free_transfers(asix_t *asix)
{
    for (int i = 0; i < ASIX_RX_XFER_COUNT; i++) {
        if (asix->rx_xfer[i]) {
            usb_host_transfer_free(asix->rx_xfer[i]);
            asix->rx_xfer[i] = NULL;
        }
    }
    if (asix->tx_xfer) {
        usb_host_transfer_free(asix->tx_xfer);
        asix->tx_xfer = NULL;
    }
    if (asix->int_xfer) {
        usb_host_transfer_free(asix->int_xfer);
        asix->int_xfer = NULL;
    }
    if (asix->ctrl_xfer) {
        usb_host_transfer_free(asix->ctrl_xfer);
        asix->ctrl_xfer = NULL;
    }
}

static esp_err_t asix_alloc_transfers(asix_t *asix)
{
    ESP_RETURN_ON_ERROR(usb_host_transfer_alloc(ASIX_CTRL_BUF_SIZE, 0, &asix->ctrl_xfer),
                        TAG, "ctrl transfer");
    ESP_RETURN_ON_ERROR(usb_host_transfer_alloc(ASIX_TX_BUF_SIZE, 0, &asix->tx_xfer),
                        TAG, "tx transfer");
    if (asix->ep_int) {
        ESP_RETURN_ON_ERROR(usb_host_transfer_alloc(ASIX_INT_BUF_SIZE, 0, &asix->int_xfer),
                            TAG, "int transfer");
    }
    for (int i = 0; i < ASIX_RX_XFER_COUNT; i++) {
        ESP_RETURN_ON_ERROR(usb_host_transfer_alloc(ASIX_RX_BUF_SIZE, 0, &asix->rx_xfer[i]),
                            TAG, "rx transfer");
    }
    return ESP_OK;
}

/* Walk the active configuration for a vendor-class interface with the
 * bulk in, bulk out and interrupt in endpoints the chip needs. */
static esp_err_t asix_find_endpoints(asix_t *asix)
{
    const usb_config_desc_t *cfg_desc = NULL;
    ESP_RETURN_ON_ERROR(usb_host_get_active_config_descriptor(asix->dev, &cfg_desc),
                        TAG, "config descriptor");

    int offset = 0;
    const usb_intf_desc_t *intf = usb_parse_interface_descriptor(cfg_desc, 0, 0, &offset);
    if (!intf) {
        return ESP_ERR_NOT_FOUND;
    }

    asix->itf_num = intf->bInterfaceNumber;
    asix->ep_in = asix->ep_out = asix->ep_int = 0;
    asix->mps_out = 0;

    for (int i = 0; i < intf->bNumEndpoints; i++) {
        int ep_offset = offset;
        const usb_ep_desc_t *ep = usb_parse_endpoint_descriptor_by_index(
            intf, i, cfg_desc->wTotalLength, &ep_offset);
        if (!ep) {
            continue;
        }

        const bool dir_in = USB_EP_DESC_GET_EP_DIR(ep);
        const uint8_t type = USB_EP_DESC_GET_XFERTYPE(ep);

        if (type == USB_TRANSFER_TYPE_INTR && dir_in) {
            asix->ep_int = ep->bEndpointAddress;
        } else if (type == USB_TRANSFER_TYPE_BULK) {
            if (dir_in) {
                asix->ep_in = ep->bEndpointAddress;
            } else {
                asix->ep_out = ep->bEndpointAddress;
                asix->mps_out = USB_EP_DESC_GET_MPS(ep);
            }
        }
    }

    if (!asix->ep_in || !asix->ep_out) {
        ESP_LOGE(TAG, "adapter has no bulk endpoint pair");
        return ESP_ERR_NOT_FOUND;
    }
    if (!asix->ep_int) {
        ESP_LOGW(TAG, "no interrupt endpoint; link state will be assumed up");
    }

    return ESP_OK;
}

static esp_err_t asix_start_rx(asix_t *asix)
{
    /* stop() halts these endpoints; a halted endpoint refuses new
     * transfers until cleared. Harmless on a fresh device. */
    usb_host_endpoint_clear(asix->dev, asix->ep_in);
    if (asix->ep_int) {
        usb_host_endpoint_clear(asix->dev, asix->ep_int);
    }

    for (int i = 0; i < ASIX_RX_XFER_COUNT; i++) {
        usb_transfer_t *xfer = asix->rx_xfer[i];
        xfer->device_handle = asix->dev;
        xfer->bEndpointAddress = asix->ep_in;
        xfer->num_bytes = ASIX_RX_BUF_SIZE;
        xfer->timeout_ms = 0;
        xfer->callback = rx_cb;
        xfer->context = asix;
        ESP_RETURN_ON_ERROR(usb_host_transfer_submit(xfer), TAG, "rx submit");
    }

    if (asix->ep_int) {
        usb_transfer_t *xfer = asix->int_xfer;
        xfer->device_handle = asix->dev;
        xfer->bEndpointAddress = asix->ep_int;
        xfer->num_bytes = ASIX_INT_BUF_SIZE;
        xfer->timeout_ms = 0;
        xfer->callback = int_cb;
        xfer->context = asix;
        ESP_RETURN_ON_ERROR(usb_host_transfer_submit(xfer), TAG, "int submit");
    } else {
        /* No link notifications to wait for. */
        asix_report_link(asix, true);
    }

    return ESP_OK;
}

static esp_err_t asix_open_device(asix_t *asix, uint8_t addr)
{
    if (asix->attached) {
        return ESP_ERR_INVALID_STATE;   /* One adapter at a time. */
    }

    usb_device_handle_t dev = NULL;
    ESP_RETURN_ON_ERROR(usb_host_device_open(asix->client, addr, &dev), TAG, "device open");

    esp_err_t ret = ESP_OK;
    const usb_device_desc_t *dev_desc = NULL;
    ESP_GOTO_ON_ERROR(usb_host_get_device_descriptor(dev, &dev_desc), close, TAG, "device desc");

    if (!asix_id_matches(asix, dev_desc->idVendor, dev_desc->idProduct)) {
        ret = ESP_ERR_NOT_FOUND;    /* Not ours; some other client's device. */
        goto close;
    }

    ESP_LOGI(TAG, "ASIX adapter %04x:%04x at address %d",
             dev_desc->idVendor, dev_desc->idProduct, addr);

    asix->dev = dev;
    asix->dev_addr = addr;
    asix->attached = true;      /* asix_ctrl() needs this set */

    ESP_GOTO_ON_ERROR(asix_find_endpoints(asix), detach, TAG, "endpoints");
    ESP_GOTO_ON_ERROR(usb_host_interface_claim(asix->client, dev, asix->itf_num, 0),
                      detach, TAG, "interface claim");
    ESP_GOTO_ON_ERROR(asix_alloc_transfers(asix), release, TAG, "transfers");

    /* MAC address lives in EEPROM words 0x04..0x06, two bytes at a time. */
    for (int i = 0; i < ETH_ALEN / 2; i++) {
        ESP_GOTO_ON_ERROR(asix_read_cmd(asix, AX_CMD_READ_EEPROM, 0x04 + i, 0,
                                        &asix->mac[i * 2], 2),
                          release, TAG, "read MAC");
    }
    ESP_LOGI(TAG, "MAC %02x:%02x:%02x:%02x:%02x:%02x",
             asix->mac[0], asix->mac[1], asix->mac[2],
             asix->mac[3], asix->mac[4], asix->mac[5]);

    uint8_t phy_buf[2] = { 0 };
    ESP_GOTO_ON_ERROR(asix_read_cmd(asix, AX_CMD_READ_PHY_ID, 0, 0, phy_buf, 2),
                      release, TAG, "read PHY id");
    asix->phy_addr = phy_buf[1];        /* [1] is the internal PHY */
    asix->embd_phy = ((asix->phy_addr & 0x1f) == AX_EMBD_PHY_ADDR);

    ESP_GOTO_ON_ERROR(asix_read_cmd(asix, AX_CMD_STATMNGSTS_REG, 0, 0, &asix->chipcode, 1),
                      release, TAG, "read chipcode");
    asix->chipcode &= AX_CHIPCODE_MASK;
    ESP_LOGI(TAG, "PHY 0x%02x (%s), chipcode 0x%02x",
             asix->phy_addr, asix->embd_phy ? "internal" : "external", asix->chipcode);

    if (asix->chipcode == AX_AX88772_CHIPCODE) {
        ESP_GOTO_ON_ERROR(ax88772_hw_reset(asix), release, TAG, "hw reset");
    } else {
        ESP_GOTO_ON_ERROR(ax88772a_hw_reset(asix), release, TAG, "hw reset");
    }

    if (asix->mediator) {
        asix->mediator->on_stage_changed(asix->mediator, IOT_ETH_STAGE_LL_INIT, NULL);
    }

    if (asix->started) {
        ESP_GOTO_ON_ERROR(asix_start_rx(asix), release, TAG, "start rx");
    }

    return ESP_OK;

release:
    asix_free_transfers(asix);
    usb_host_interface_release(asix->client, dev, asix->itf_num);
detach:
    asix->attached = false;
    asix->dev = NULL;
close:
    usb_host_device_close(asix->client, dev);
    return ret;
}

static void asix_close_device(asix_t *asix)
{
    if (!asix->attached) {
        return;
    }

    asix->attached = false;
    asix_report_link(asix, false);

    /* Transfers are cancelled by the halt/flush pair before the buffers
     * they point at are freed. */
    if (asix->ep_in) {
        usb_host_endpoint_halt(asix->dev, asix->ep_in);
        usb_host_endpoint_flush(asix->dev, asix->ep_in);
    }
    if (asix->ep_out) {
        usb_host_endpoint_halt(asix->dev, asix->ep_out);
        usb_host_endpoint_flush(asix->dev, asix->ep_out);
    }
    if (asix->ep_int) {
        usb_host_endpoint_halt(asix->dev, asix->ep_int);
        usb_host_endpoint_flush(asix->dev, asix->ep_int);
    }

    /* Flushing only cancels. The cancelled transfers' callbacks are still
     * to come, delivered by the event task, and usb_host_transfer_free()
     * refuses a transfer that has not been returned yet. This wait lets
     * them drain; the callbacks see `attached == false` and do not
     * resubmit. A counted drain would be tighter than a fixed delay, and
     * is the obvious improvement if unplug ever logs a free failure. */
    vTaskDelay(pdMS_TO_TICKS(50));

    asix_free_transfers(asix);
    usb_host_interface_release(asix->client, asix->dev, asix->itf_num);
    usb_host_device_close(asix->client, asix->dev);

    asix->dev = NULL;
    asix->ep_in = asix->ep_out = asix->ep_int = 0;

    if (asix->mediator) {
        asix->mediator->on_stage_changed(asix->mediator, IOT_ETH_STAGE_LL_DEINIT, NULL);
    }

    ESP_LOGI(TAG, "adapter removed");
}

/* ------------------------------------------------------------------ */
/* Tasks                                                               */
/* ------------------------------------------------------------------ */

static void client_event_cb(const usb_host_client_event_msg_t *msg, void *arg)
{
    asix_t *asix = (asix_t *)arg;
    asix_cmd_t cmd = { 0 };

    switch (msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        cmd.type = ASIX_CMD_NEW_DEV;
        cmd.addr = msg->new_dev.address;
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        cmd.type = ASIX_CMD_DEV_GONE;
        cmd.dev = msg->dev_gone.dev_hdl;
        break;
    default:
        return;
    }

    /* Never block here: this runs inside usb_host_client_handle_events(),
     * which has to stay free to pump transfer completions. */
    if (xQueueSend(asix->cmd_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGW(TAG, "command queue full, event dropped");
    }
}

static void event_task(void *arg)
{
    asix_t *asix = (asix_t *)arg;

    while (asix->running) {
        usb_host_client_handle_events(asix->client, pdMS_TO_TICKS(50));
    }

    vTaskDelete(NULL);
}

static void worker_task(void *arg)
{
    asix_t *asix = (asix_t *)arg;
    asix_cmd_t cmd;

    while (asix->running) {
        if (xQueueReceive(asix->cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }

        switch (cmd.type) {
        case ASIX_CMD_NEW_DEV: {
            const esp_err_t err = asix_open_device(asix, cmd.addr);
            if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) {
                ESP_LOGE(TAG, "bring-up failed: %s", esp_err_to_name(err));
            }
            break;
        }
        case ASIX_CMD_DEV_GONE:
            if (asix->attached && cmd.dev == asix->dev) {
                asix_close_device(asix);
            }
            break;
        case ASIX_CMD_LINK:
            if (asix->attached && asix->started) {
                asix_report_link(asix, cmd.link_up);
            }
            break;
        }
    }

    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/* iot_eth_driver_t interface                                          */
/* ------------------------------------------------------------------ */

static esp_err_t asix_set_mediator(iot_eth_driver_t *driver, iot_eth_mediator_t *eth)
{
    ESP_RETURN_ON_FALSE(eth, ESP_ERR_INVALID_ARG, TAG, "mediator is NULL");
    ((asix_t *)driver)->mediator = eth;
    return ESP_OK;
}

static esp_err_t asix_init(iot_eth_driver_t *driver)
{
    asix_t *asix = (asix_t *)driver;
    esp_err_t ret = ESP_OK;

    if (!asix->cfg.skip_init_usb_host_driver) {
        const usb_host_config_t host_cfg = {
            .skip_phy_setup = false,
            .intr_flags = ESP_INTR_FLAG_LEVEL1,
        };
        ESP_RETURN_ON_ERROR(usb_host_install(&host_cfg), TAG, "usb_host_install");
    }

    const usb_host_client_config_t client_cfg = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = asix,
        },
    };
    ESP_GOTO_ON_ERROR(usb_host_client_register(&client_cfg, &asix->client),
                      err_host, TAG, "client register");

    asix->running = true;

    const uint32_t stack = asix->cfg.task_stack_size ? asix->cfg.task_stack_size : ASIX_DEFAULT_STACK;
    const UBaseType_t prio = asix->cfg.task_priority ? asix->cfg.task_priority : ASIX_DEFAULT_PRIO;
    const BaseType_t core = (asix->cfg.task_coreid < 0) ? tskNO_AFFINITY : asix->cfg.task_coreid;

    if (xTaskCreatePinnedToCore(event_task, "asix_ev", stack, asix, prio,
                                &asix->event_task, core) != pdPASS) {
        ret = ESP_ERR_NO_MEM;
        goto err_client;
    }
    if (xTaskCreatePinnedToCore(worker_task, "asix_wk", stack, asix, prio - 1,
                                &asix->worker_task, core) != pdPASS) {
        ret = ESP_ERR_NO_MEM;
        goto err_client;
    }

    return ESP_OK;

err_client:
    asix->running = false;
    usb_host_client_deregister(asix->client);
    asix->client = NULL;
err_host:
    if (!asix->cfg.skip_init_usb_host_driver) {
        usb_host_uninstall();
    }
    return ret;
}

static esp_err_t asix_deinit(iot_eth_driver_t *driver)
{
    asix_t *asix = (asix_t *)driver;

    asix->running = false;
    /* Let both tasks reach their timeout and exit. */
    vTaskDelay(pdMS_TO_TICKS(200));

    asix_close_device(asix);

    if (asix->client) {
        usb_host_client_deregister(asix->client);
        asix->client = NULL;
    }
    if (!asix->cfg.skip_init_usb_host_driver) {
        usb_host_uninstall();
    }

    return ESP_OK;
}

static esp_err_t asix_start(iot_eth_driver_t *driver)
{
    asix_t *asix = (asix_t *)driver;

    asix->started = true;

    /* An adapter already plugged in when start() is called needs its
     * transfers kicked off now; one plugged in later gets them from the
     * bring-up path. */
    if (asix->attached) {
        return asix_start_rx(asix);
    }
    return ESP_OK;
}

static esp_err_t asix_stop(iot_eth_driver_t *driver)
{
    asix_t *asix = (asix_t *)driver;

    asix->started = false;

    if (asix->attached) {
        asix_report_link(asix, false);
        usb_host_endpoint_halt(asix->dev, asix->ep_in);
        usb_host_endpoint_flush(asix->dev, asix->ep_in);
        if (asix->ep_int) {
            usb_host_endpoint_halt(asix->dev, asix->ep_int);
            usb_host_endpoint_flush(asix->dev, asix->ep_int);
        }
    }

    return ESP_OK;
}

static esp_err_t asix_get_addr(iot_eth_driver_t *driver, uint8_t *mac_address)
{
    asix_t *asix = (asix_t *)driver;

    ESP_RETURN_ON_FALSE(mac_address, ESP_ERR_INVALID_ARG, TAG, "mac_address is NULL");
    ESP_RETURN_ON_FALSE(asix->attached, ESP_ERR_INVALID_STATE, TAG, "no adapter attached");

    memcpy(mac_address, asix->mac, ETH_ALEN);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

esp_err_t esp_usbh_asix_new_eth(const esp_usbh_asix_config_t *config, iot_eth_driver_t **ret_handle)
{
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "config is NULL");
    ESP_RETURN_ON_FALSE(ret_handle, ESP_ERR_INVALID_ARG, TAG, "ret_handle is NULL");

    asix_t *asix = calloc(1, sizeof(asix_t));
    ESP_RETURN_ON_FALSE(asix, ESP_ERR_NO_MEM, TAG, "no memory for driver");

    memcpy(&asix->cfg, config, sizeof(esp_usbh_asix_config_t));

    asix->ctrl_done = xSemaphoreCreateBinary();
    asix->tx_done = xSemaphoreCreateBinary();
    asix->tx_lock = xSemaphoreCreateMutex();
    asix->cmd_queue = xQueueCreate(4, sizeof(asix_cmd_t));

    if (!asix->ctrl_done || !asix->tx_done || !asix->tx_lock || !asix->cmd_queue) {
        goto err;
    }

    asix->base.name = "usb_asix";
    asix->base.set_mediator = asix_set_mediator;
    asix->base.init = asix_init;
    asix->base.deinit = asix_deinit;
    asix->base.start = asix_start;
    asix->base.stop = asix_stop;
    asix->base.transmit = asix_transmit;
    asix->base.get_addr = asix_get_addr;

    *ret_handle = &asix->base;
    return ESP_OK;

err:
    if (asix->ctrl_done) {
        vSemaphoreDelete(asix->ctrl_done);
    }
    if (asix->tx_done) {
        vSemaphoreDelete(asix->tx_done);
    }
    if (asix->tx_lock) {
        vSemaphoreDelete(asix->tx_lock);
    }
    if (asix->cmd_queue) {
        vQueueDelete(asix->cmd_queue);
    }
    free(asix);
    return ESP_ERR_NO_MEM;
}

esp_err_t esp_usbh_asix_get_chipcode(iot_eth_driver_t *handle, uint8_t *chipcode)
{
    ESP_RETURN_ON_FALSE(handle && chipcode, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    asix_t *asix = (asix_t *)handle;
    ESP_RETURN_ON_FALSE(asix->attached, ESP_ERR_INVALID_STATE, TAG, "no adapter attached");

    *chipcode = asix->chipcode;
    return ESP_OK;
}
