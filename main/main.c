#include <stdbool.h>
#include <string.h>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "ble_keyboard.h"
#include "bond_reset.h"
#include "usb_keyboard.h"
#include "usb_selftest.h"

static const char *TAG = "main";

_Static_assert(BLE_KEYBOARD_REPORT_LEN == USB_KEYBOARD_REPORT_LEN,
               "BLE and USB report formats must match");

#define REPORT_LEN          USB_KEYBOARD_REPORT_LEN
#define QUEUE_DEPTH         32
// While a report is stuck (host busy or suspended), how often to retry it.
#define RETRY_INTERVAL_MS   10

// A report or a release-all both travel as the full key state to send. Going
// through one queue keeps a release-all ordered after reports ahead of it.
typedef struct {
    uint8_t report[REPORT_LEN];
} bridge_msg_t;

static QueueHandle_t s_queue;

static void enqueue(const uint8_t report[REPORT_LEN])
{
    bridge_msg_t msg;
    memcpy(msg.report, report, REPORT_LEN);
    if (xQueueSend(s_queue, &msg, 0) != pdTRUE) {
        // Each report is the keyboard's whole state, so if we fall behind,
        // dropping the oldest still ends on the right state.
        bridge_msg_t dropped;
        xQueueReceive(s_queue, &dropped, 0);
        xQueueSend(s_queue, &msg, 0);
        ESP_LOGW(TAG, "report queue full; dropped oldest report");
    }
}

// Both callbacks run on the BLE host task: hand off and return.

static void on_ble_report(const uint8_t report[BLE_KEYBOARD_REPORT_LEN], void *ctx)
{
    (void)ctx;
    enqueue(report);
}

static void on_ble_state(ble_keyboard_state_t state, void *ctx)
{
    (void)ctx;
    if (state == BLE_KEYBOARD_STATE_DISCONNECTED) {
        // Whatever was held when the link dropped will never get its key-up.
        static const uint8_t released[REPORT_LEN] = {0};
        enqueue(released);
    }
}

// Sends queued reports to USB. If a send fails while the host is attached,
// the report is kept and retried (newer reports replace it) so the host never
// misses the latest state, especially a key release. With no host attached,
// pending state is dropped: a host that attaches later starts with no keys down.
static void bridge_task(void *arg)
{
    (void)arg;
    bridge_msg_t pending;
    bool have_pending = false;
    bool failing = false;

    for (;;) {
        bridge_msg_t msg;
        TickType_t wait = have_pending ? pdMS_TO_TICKS(RETRY_INTERVAL_MS) : portMAX_DELAY;
        if (xQueueReceive(s_queue, &msg, wait) == pdTRUE) {
            pending = msg;
            have_pending = true;
        }
        if (!have_pending) {
            continue;
        }

        esp_err_t err = usb_keyboard_send_report(pending.report);
        if (err == ESP_OK) {
            have_pending = false;
            if (failing) {
                ESP_LOGI(TAG, "USB sends recovered");
                failing = false;
            }
        } else if (err == ESP_ERR_INVALID_STATE) {
            have_pending = false; // no USB host; nothing to keep in sync
        } else if (!failing) {
            ESP_LOGW(TAG, "USB send failing (%s); retrying", esp_err_to_name(err));
            failing = true;
        }
    }
}

static void init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS unusable (%s); erasing it, which also drops stored bonds",
                 esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void app_main(void)
{
    ESP_LOGI(TAG, "reset reason: %d", esp_reset_reason());
    const bool clear_bonds = bond_reset_requested();
    init_nvs();

    s_queue = xQueueCreate(QUEUE_DEPTH, sizeof(bridge_msg_t));
    assert(s_queue != NULL);

    ESP_ERROR_CHECK(usb_keyboard_init());
    usb_selftest_start();

    xTaskCreate(bridge_task, "bridge", 3072, NULL, 5, NULL);

    const ble_keyboard_config_t ble_config = {
        .on_report = on_ble_report,
        .on_state = on_ble_state,
        .ctx = NULL,
        .clear_bonds = clear_bonds,
    };
    ESP_ERROR_CHECK(ble_keyboard_init(&ble_config));
    bond_reset_button_start();

    ESP_LOGI(TAG, "bridge started");
}
