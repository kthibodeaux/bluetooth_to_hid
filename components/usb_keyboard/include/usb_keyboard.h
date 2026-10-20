#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Length of a boot-protocol keyboard report: modifiers, reserved, 6 keycodes. */
#define USB_KEYBOARD_REPORT_LEN 8

/** Simultaneous consumer (media key) usages per consumer report. */
#define USB_KEYBOARD_CONSUMER_MAX 4

/**
 * Install the TinyUSB driver on the native USB port and present a
 * boot-protocol keyboard interface, plus a Consumer Control interface for
 * media keys if CONFIG_USB_KEYBOARD_MEDIA_KEYS is set. Returns once the
 * driver is installed; the host may enumerate the device some time later.
 */
esp_err_t usb_keyboard_init(void);

/** True once the host has configured the device and the HID endpoint is idle. */
bool usb_keyboard_is_ready(void);

/**
 * Send one 8-byte boot keyboard report to the host. Returns
 * ESP_ERR_INVALID_STATE if the host isn't connected, ESP_ERR_TIMEOUT if the
 * previous report never went out (e.g. host suspended). Doesn't log failures;
 * that's left to the caller.
 */
esp_err_t usb_keyboard_send_report(const uint8_t report[USB_KEYBOARD_REPORT_LEN]);

/**
 * Send the consumer control usages currently held (e.g. 0xE9 Volume Up),
 * zero-filled; all zero releases. Same return values as
 * usb_keyboard_send_report(), plus ESP_ERR_NOT_SUPPORTED when media keys are
 * disabled in menuconfig.
 */
esp_err_t usb_keyboard_send_consumer(const uint16_t usages[USB_KEYBOARD_CONSUMER_MAX]);

/** Release every key, modifier and media key on the host. */
esp_err_t usb_keyboard_release_all(void);

#ifdef __cplusplus
}
#endif
