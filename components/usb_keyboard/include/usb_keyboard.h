#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Length of a boot-protocol keyboard report: modifiers, reserved, 6 keycodes. */
#define USB_KEYBOARD_REPORT_LEN 8

/**
 * Install the TinyUSB driver on the native USB port and present a single
 * boot-protocol keyboard interface. Returns once the driver is installed;
 * the host may enumerate the device some time later.
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

/** Send an all-zero report so no key or modifier stays held on the host. */
esp_err_t usb_keyboard_release_all(void);

#ifdef __cplusplus
}
#endif
