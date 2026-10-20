#include "bond_reset.h"

#include <stdint.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bond_reset";

#define BOOT_BUTTON_GPIO    GPIO_NUM_0
#define BOND_RESET_HOLD_MS  3000
#define POLL_MS             20
#define REQUEST_MAGIC       0xB0D5C1EAu

// Survives esp_restart() but not a power cycle, which is all we need to carry
// the request across the reboot.
static RTC_NOINIT_ATTR uint32_t s_request;

bool bond_reset_requested(void)
{
    bool requested = s_request == REQUEST_MAGIC && esp_reset_reason() == ESP_RST_SW;
    s_request = 0;
    return requested;
}

static void button_task(void *arg)
{
    (void)arg;
    TickType_t pressed_at = 0;
    bool pressed = false;

    for (;;) {
        bool down = gpio_get_level(BOOT_BUTTON_GPIO) == 0; // active low
        TickType_t now = xTaskGetTickCount();

        if (down && !pressed) {
            pressed = true;
            pressed_at = now;
            ESP_LOGI(TAG, "BOOT pressed; keep holding %d s to clear bonds", BOND_RESET_HOLD_MS / 1000);
        } else if (!down && pressed) {
            pressed = false;
            ESP_LOGI(TAG, "BOOT released; bond reset cancelled");
        } else if (down && now - pressed_at >= pdMS_TO_TICKS(BOND_RESET_HOLD_MS)) {
            ESP_LOGW(TAG, "clearing bonds: restarting into pairing mode");
            s_request = REQUEST_MAGIC;
            vTaskDelay(pdMS_TO_TICKS(100)); // let the log line out
            esp_restart();
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

void bond_reset_button_start(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << BOOT_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    xTaskCreate(button_task, "bond_reset", 2560, NULL, 2, NULL);
}
