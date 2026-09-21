/*
 * SPDX-FileCopyrightText: 2026 Sudrien
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "iot_eth_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief One VID/PID pair the driver should bind to.
 *
 * A list is terminated by an entry with both fields zero.
 */
typedef struct {
    uint16_t vid;   /*!< USB idVendor */
    uint16_t pid;   /*!< USB idProduct */
} esp_usbh_asix_id_t;

/**
 * @brief Configuration for the ASIX USB Ethernet driver.
 */
typedef struct {
    /**
     * Extra VID/PID pairs to bind to, on top of the built-in table
     * (AX88772 0x7720, AX88772A 0x772a, AX88772B 0x772b, all under
     * vendor 0x0b95). Rebadged adapters ship with their own IDs, so this
     * is how a device the driver has never heard of gets picked up.
     *
     * NULL means the built-in table only. The list is not copied: it has
     * to outlive the driver, which a static const array does.
     */
    const esp_usbh_asix_id_t *extra_id_list;

    /**
     * Skip usb_host_install() and the USB host library event task.
     *
     * Leave false and the driver owns the host stack. Set true when the
     * application already installed it -- a player that also has mass
     * storage and audio on the same port, say -- in which case the
     * driver registers as one more client and never touches the stack's
     * lifetime. Mirrors usbh_cdc_driver_config_t::skip_init_usb_host_driver
     * in espressif/iot_usbh_cdc, and for the same reason.
     */
    bool skip_init_usb_host_driver;

    uint32_t task_stack_size;   /*!< Driver task stack, 0 for the default */
    uint8_t task_priority;      /*!< Driver task priority, 0 for the default */
    int task_coreid;            /*!< Core to pin the task to, -1 for no affinity */
} esp_usbh_asix_config_t;

/**
 * @brief Create an ASIX USB Ethernet driver.
 *
 * The returned handle is an iot_eth_driver_t, to be passed to
 * iot_eth_install() from espressif/iot_eth. Nothing happens on the wire
 * until iot_eth_start(); the adapter may be plugged in before or after,
 * and may be unplugged and replugged freely.
 *
 * @param[in]  config    Driver configuration, must not be NULL
 * @param[out] ret_handle Where to store the driver handle
 *
 * @return
 *      - ESP_OK: driver created
 *      - ESP_ERR_INVALID_ARG: config or ret_handle is NULL
 *      - ESP_ERR_NO_MEM: out of memory
 */
esp_err_t esp_usbh_asix_new_eth(const esp_usbh_asix_config_t *config, iot_eth_driver_t **ret_handle);

/**
 * @brief Read the chip code of the attached adapter.
 *
 * Useful for logging and for bug reports: the reset sequence the driver
 * runs depends on it. Values are AX_AX88772_CHIPCODE (0x00),
 * AX_AX88772A_CHIPCODE (0x10) and AX_AX88772B_CHIPCODE (0x20).
 *
 * @param[in]  handle    Driver handle from esp_usbh_asix_new_eth()
 * @param[out] chipcode  Where to store the chip code
 *
 * @return
 *      - ESP_OK: chip code stored
 *      - ESP_ERR_INVALID_ARG: handle or chipcode is NULL
 *      - ESP_ERR_INVALID_STATE: no adapter is attached
 */
esp_err_t esp_usbh_asix_get_chipcode(iot_eth_driver_t *handle, uint8_t *chipcode);

#ifdef __cplusplus
}
#endif
