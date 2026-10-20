#pragma once

/**
 * If enabled in menuconfig, start a one-shot task that waits for the USB host
 * and types the configured test string. No-op otherwise.
 */
void usb_selftest_start(void);
