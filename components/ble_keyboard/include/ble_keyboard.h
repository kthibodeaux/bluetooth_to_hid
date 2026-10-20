#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Length of a boot keyboard input report (characteristic 0x2A22). */
#define BLE_KEYBOARD_REPORT_LEN 8

/** Max simultaneous consumer (media key) usages delivered per report. */
#define BLE_KEYBOARD_CONSUMER_MAX 4

typedef enum {
    BLE_KEYBOARD_STATE_SCANNING,     /**< Looking for a keyboard (bonded or new). */
    BLE_KEYBOARD_STATE_CONNECTING,   /**< Link up; pairing / GATT discovery in progress. */
    BLE_KEYBOARD_STATE_READY,        /**< Subscribed to boot reports; reports will flow. */
    BLE_KEYBOARD_STATE_DISCONNECTED, /**< Link lost; scanning resumes automatically. */
} ble_keyboard_state_t;

/**
 * Called from the BLE host task for every boot report received. Keep it short:
 * copy the report into a queue rather than doing real work here.
 */
typedef void (*ble_keyboard_report_cb_t)(const uint8_t report[BLE_KEYBOARD_REPORT_LEN], void *ctx);

/**
 * Called from the BLE host task with the consumer control usages currently
 * held (e.g. 0xE9 Volume Up, 0xCD Play/Pause), zero-filled; all zero means
 * released. Only delivered for keyboards read in report mode.
 */
typedef void (*ble_keyboard_consumer_cb_t)(const uint16_t usages[BLE_KEYBOARD_CONSUMER_MAX], void *ctx);

/** Called from the BLE host task whenever the connection state changes. */
typedef void (*ble_keyboard_state_cb_t)(ble_keyboard_state_t state, void *ctx);

typedef struct {
    ble_keyboard_report_cb_t on_report;
    ble_keyboard_consumer_cb_t on_consumer; /**< Optional; NULL ignores media keys. */
    ble_keyboard_state_cb_t on_state;
    void *ctx;              /**< Passed back to all callbacks. */
    bool clear_bonds;       /**< Erase stored bonds before starting (pairing mode). */
} ble_keyboard_config_t;

/**
 * Start the BLE host. With a stored bond it reconnects only to that keyboard;
 * otherwise it scans for any device advertising the HID service and pairs.
 * NVS must already be initialized.
 */
esp_err_t ble_keyboard_init(const ble_keyboard_config_t *config);

#ifdef __cplusplus
}
#endif
