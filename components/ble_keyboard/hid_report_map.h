#pragma once

// Minimal HID report descriptor parsing: finds the keyboard input report in a
// device's Report Map and translates that report into the 8-byte boot
// keyboard format. Pure C with no BLE dependency.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define HID_KBD_MAX_FIELDS 8

/** One keyboard-page (0x07) input field inside the keyboard report. */
typedef struct {
    uint16_t bit_offset;
    uint8_t size;           // bits per element
    uint8_t count;          // number of elements
    uint16_t usage_min;
    uint16_t usage_max;
    int32_t logical_min;
    bool is_array;          // array of keycodes (vs. one bit per usage)
} hid_kbd_field_t;

typedef struct {
    uint8_t report_id;      // 0 if the descriptor doesn't use report IDs
    uint16_t report_bits;   // total input report size for this ID, in bits
    int field_count;
    hid_kbd_field_t fields[HID_KBD_MAX_FIELDS];
} hid_kbd_layout_t;

/**
 * Parse a HID report descriptor and describe its keyboard input report: the
 * first report ID that carries keyboard-page keys (not just modifiers).
 * Returns false if there is none.
 */
bool hid_kbd_layout_parse(const uint8_t *map, size_t len, hid_kbd_layout_t *out);

/**
 * Translate one keyboard input report (without a report ID byte, as HOGP
 * delivers it) into a boot report: modifiers, reserved, 6 keycodes. More than
 * six keys down fills the key slots with ErrorRollOver (0x01), as a boot
 * keyboard would.
 */
void hid_kbd_translate(const hid_kbd_layout_t *layout, const uint8_t *data, size_t len,
                       uint8_t out[8]);
