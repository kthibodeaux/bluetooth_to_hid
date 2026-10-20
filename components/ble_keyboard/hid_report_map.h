#pragma once

// Minimal HID report descriptor parsing: finds the keyboard and consumer
// (media key) input reports in a device's Report Map and translates them into
// the fixed formats the bridge forwards. Pure C with no BLE dependency.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define HID_MAX_FIELDS          8
#define HID_FIELD_MAX_USAGES    16
#define HID_CONSUMER_MAX_USAGES 4

/** One non-constant input field of the chosen report. */
typedef struct {
    uint16_t bit_offset;
    uint8_t size;           // bits per element
    uint8_t count;          // number of elements
    uint16_t usage_min;
    uint16_t usage_max;
    int32_t logical_min;
    bool is_array;          // array of usage indices (vs. one bit/value per usage)
    uint8_t usage_count;    // >0: usages listed individually in `usages`
    uint16_t usages[HID_FIELD_MAX_USAGES];
} hid_field_t;

typedef struct {
    uint8_t report_id;      // 0 if the descriptor doesn't use report IDs
    uint16_t report_bits;   // total input report size for this ID, in bits
    int field_count;
    hid_field_t fields[HID_MAX_FIELDS];
} hid_layout_t;

/**
 * Describe the keyboard input report: the first report ID that carries
 * keyboard-page (0x07) keys, not just modifiers. Returns false if none.
 */
bool hid_kbd_layout_parse(const uint8_t *map, size_t len, hid_layout_t *out);

/**
 * Translate one keyboard input report (without a report ID byte, as HOGP
 * delivers it) into a boot report: modifiers, reserved, 6 keycodes. More than
 * six keys down fills the key slots with ErrorRollOver (0x01), as a boot
 * keyboard would.
 */
void hid_kbd_translate(const hid_layout_t *layout, const uint8_t *data, size_t len,
                       uint8_t out[8]);

/**
 * Describe the consumer control input report: the first report ID with
 * consumer-page (0x0C) input fields. Returns false if none.
 */
bool hid_consumer_layout_parse(const uint8_t *map, size_t len, hid_layout_t *out);

/**
 * Translate one consumer input report into the active consumer usages
 * (e.g. 0xE9 Volume Up), up to HID_CONSUMER_MAX_USAGES, zero-filled.
 */
void hid_consumer_translate(const hid_layout_t *layout, const uint8_t *data, size_t len,
                            uint16_t out[HID_CONSUMER_MAX_USAGES]);
