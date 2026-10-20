#include "usb_selftest.h"

#include "sdkconfig.h"

#if CONFIG_BRIDGE_USB_SELFTEST

#include <stdbool.h>
#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "usb_keyboard.h"

static const char *TAG = "usb_selftest";

#define MOD_LEFT_SHIFT 0x02

// US-layout ASCII to HID usage. Returns false for characters we don't map.
static bool ascii_to_hid(char c, uint8_t *keycode, uint8_t *modifier)
{
    static const char *shifted_digits = ")!@#$%^&*(";

    *modifier = 0;
    if (c >= 'a' && c <= 'z') {
        *keycode = 0x04 + (c - 'a');
    } else if (c >= 'A' && c <= 'Z') {
        *keycode = 0x04 + (c - 'A');
        *modifier = MOD_LEFT_SHIFT;
    } else if (c >= '1' && c <= '9') {
        *keycode = 0x1E + (c - '1');
    } else if (c == '0') {
        *keycode = 0x27;
    } else if (c == ' ') {
        *keycode = 0x2C;
    } else if (c == '-') {
        *keycode = 0x2D;
    } else if (c == '.') {
        *keycode = 0x37;
    } else if (c == ',') {
        *keycode = 0x36;
    } else if (c == '/') {
        *keycode = 0x38;
    } else {
        for (int i = 0; i < 10; i++) {
            if (c == shifted_digits[i]) {
                *keycode = (i == 0) ? 0x27 : 0x1E + (i - 1);
                *modifier = MOD_LEFT_SHIFT;
                return true;
            }
        }
        return false;
    }
    return true;
}

static void selftest_task(void *arg)
{
    (void)arg;

    while (!usb_keyboard_is_ready()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "USB ready; typing test string in %d ms", CONFIG_BRIDGE_USB_SELFTEST_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(CONFIG_BRIDGE_USB_SELFTEST_DELAY_MS));

    for (const char *p = CONFIG_BRIDGE_USB_SELFTEST_TEXT; *p != '\0'; p++) {
        uint8_t report[USB_KEYBOARD_REPORT_LEN] = {0};
        if (!ascii_to_hid(*p, &report[2], &report[0])) {
            ESP_LOGW(TAG, "skipping unmapped character 0x%02x", (unsigned char)*p);
            continue;
        }
        usb_keyboard_send_report(report);
        vTaskDelay(pdMS_TO_TICKS(20));
        usb_keyboard_release_all();
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    ESP_LOGI(TAG, "test string sent");
    vTaskDelete(NULL);
}

void usb_selftest_start(void)
{
    xTaskCreate(selftest_task, "usb_selftest", 3072, NULL, 4, NULL);
}

#else

void usb_selftest_start(void)
{
}

#endif
