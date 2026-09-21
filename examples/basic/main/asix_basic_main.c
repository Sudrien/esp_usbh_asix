/*
 * SPDX-FileCopyrightText: 2026 Sudrien
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal use of esp_usbh_asix: bring up an ASIX adapter, get an address
 * by DHCP, and log it. Plug the adapter in before or after boot.
 */

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"

#include "esp_usbh_asix.h"
#include "iot_eth.h"
#include "iot_eth_netif_glue.h"

static const char *TAG = "asix_basic";

static void on_got_ip(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)data;
    ESP_LOGI(TAG, "got IP " IPSTR ", gateway " IPSTR,
             IP2STR(&event->ip_info.ip), IP2STR(&event->ip_info.gw));
}

static void on_eth_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    case IOT_ETH_EVENT_CONNECTED:
        ESP_LOGI(TAG, "cable connected");
        break;
    case IOT_ETH_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "cable disconnected");
        break;
    default:
        break;
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, on_got_ip, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IOT_ETH_EVENT, ESP_EVENT_ANY_ID, on_eth_event, NULL));

    /* The driver installs the USB host stack itself here. An application
     * that already owns the stack sets skip_init_usb_host_driver. */
    const esp_usbh_asix_config_t asix_cfg = {
        .skip_init_usb_host_driver = false,
        .task_coreid = -1,
    };
    iot_eth_driver_t *driver = NULL;
    ESP_ERROR_CHECK(esp_usbh_asix_new_eth(&asix_cfg, &driver));

    const iot_eth_config_t eth_cfg = {
        .driver = driver,
        .stack_input = NULL,
    };
    iot_eth_handle_t eth = NULL;
    ESP_ERROR_CHECK(iot_eth_install(&eth_cfg, &eth));

    esp_netif_inherent_config_t inherent = ESP_NETIF_INHERENT_DEFAULT_ETH();
    inherent.if_key = "USB_ASIX";
    inherent.if_desc = "usb asix";
    const esp_netif_config_t netif_cfg = {
        .base = &inherent,
        .driver = NULL,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH,
    };
    esp_netif_t *netif = esp_netif_new(&netif_cfg);
    ESP_RETURN_VOID_ON_FALSE(netif, TAG, "esp_netif_new failed");

    iot_eth_netif_glue_handle_t glue = iot_eth_new_netif_glue(eth);
    ESP_RETURN_VOID_ON_FALSE(glue, TAG, "iot_eth_new_netif_glue failed");
    ESP_ERROR_CHECK(esp_netif_attach(netif, glue));

    ESP_ERROR_CHECK(iot_eth_start(eth));
    ESP_LOGI(TAG, "waiting for an ASIX adapter");
}
