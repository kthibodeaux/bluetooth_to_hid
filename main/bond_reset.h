#pragma once

#include <stdbool.h>

/**
 * True if the previous run asked for stored bonds to be cleared on this boot
 * (BOOT held long enough, followed by a software restart). Consumes the
 * request: call once, early in app_main.
 */
bool bond_reset_requested(void);

/**
 * Watch the BOOT button (GPIO0). Holding it for BOND_RESET_HOLD_MS while the
 * bridge runs requests a bond reset and restarts.
 *
 * GPIO0 is a strapping pin: held during power-on or reset, it puts the chip
 * into ROM download mode and this firmware never runs. So the reset is a
 * long press while running, not "held at startup".
 */
void bond_reset_button_start(void);
