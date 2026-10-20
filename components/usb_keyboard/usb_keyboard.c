#include "usb_keyboard.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "class/hid/hid_device.h"

static const char *TAG = "usb_keyboard";

// How long send_report() waits for the previous report to leave the endpoint
// (or for the host to resume us after a remote wakeup).
#define SEND_TIMEOUT_MS 100

// TinyUSB numbers HID instances in interface order.
#define HID_INSTANCE_KEYBOARD   0
#define HID_INSTANCE_CONSUMER   1
#define EPNUM_KEYBOARD_IN       0x81
#define EPNUM_CONSUMER_IN       0x82
// Polling interval in ms. 10 ms matches typical wired keyboards; KVMs that
// emulate the keyboard downstream are known to be fussy about anything unusual.
#define HID_POLL_MS     10

#define CONSUMER_REPORT_LEN (USB_KEYBOARD_CONSUMER_MAX * 2)

enum {
    ITF_NUM_KEYBOARD = 0,
#if CONFIG_USB_KEYBOARD_MEDIA_KEYS
    ITF_NUM_CONSUMER,
#endif
    ITF_NUM_TOTAL,
};

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_KEYBOARD_ITF,
    STRID_CONSUMER_ITF,
};

// Standard boot keyboard layout with no report ID, so report mode and boot
// mode put identical 8-byte reports on the wire.
static const uint8_t s_keyboard_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(),
};

#if CONFIG_USB_KEYBOARD_MEDIA_KEYS
// Consumer Control: an array of up to four 16-bit usages (0 = none), no
// report ID. Covers the common media usages (volume, mute, playback, ...).
static const uint8_t s_consumer_report_descriptor[] = {
    HID_USAGE_PAGE(HID_USAGE_PAGE_CONSUMER),
    HID_USAGE(HID_USAGE_CONSUMER_CONTROL),
    HID_COLLECTION(HID_COLLECTION_APPLICATION),
        HID_LOGICAL_MIN(0x00),
        HID_LOGICAL_MAX_N(0x03FF, 2),
        HID_USAGE_MIN(0x00),
        HID_USAGE_MAX_N(0x03FF, 2),
        HID_REPORT_SIZE(16),
        HID_REPORT_COUNT(USB_KEYBOARD_CONSUMER_MAX),
        HID_INPUT(HID_DATA | HID_ARRAY | HID_ABSOLUTE),
    HID_COLLECTION_END,
};
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + 2 * TUD_HID_DESC_LEN)
#else
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)
#endif

static const uint8_t s_config_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_DESCRIPTOR(ITF_NUM_KEYBOARD, STRID_KEYBOARD_ITF, HID_ITF_PROTOCOL_KEYBOARD,
                       sizeof(s_keyboard_report_descriptor), EPNUM_KEYBOARD_IN,
                       CFG_TUD_HID_EP_BUFSIZE, HID_POLL_MS),
#if CONFIG_USB_KEYBOARD_MEDIA_KEYS
    TUD_HID_DESCRIPTOR(ITF_NUM_CONSUMER, STRID_CONSUMER_ITF, HID_ITF_PROTOCOL_NONE,
                       sizeof(s_consumer_report_descriptor), EPNUM_CONSUMER_IN,
                       CFG_TUD_HID_EP_BUFSIZE, HID_POLL_MS),
#endif
};

static const char *s_strings[] = {
    [STRID_LANGID] = (const char[]){0x09, 0x04}, // English (0x0409)
    [STRID_MANUFACTURER] = "Espressif",
    [STRID_PRODUCT] = "BLE to USB Keyboard",
    [STRID_SERIAL] = NULL,                       // filled from the eFuse MAC
    [STRID_KEYBOARD_ITF] = "Boot Keyboard",
    [STRID_CONSUMER_ITF] = "Media Keys",
};

static SemaphoreHandle_t s_send_lock;

// Last reports sent, returned to the host on GET_REPORT.
static uint8_t s_last_report[USB_KEYBOARD_REPORT_LEN];
static uint8_t s_last_consumer[CONSUMER_REPORT_LEN];

static void usb_event_cb(tinyusb_event_t *event, void *arg)
{
    (void)arg;
    switch (event->id) {
    case TINYUSB_EVENT_ATTACHED:
        ESP_LOGI(TAG, "host configured the keyboard");
        break;
    case TINYUSB_EVENT_DETACHED:
        ESP_LOGI(TAG, "host detached");
        break;
    case TINYUSB_EVENT_SUSPENDED:
        ESP_LOGD(TAG, "suspended (remote wakeup %s)",
                 event->suspended.remote_wakeup ? "enabled" : "disabled");
        break;
    case TINYUSB_EVENT_RESUMED:
        ESP_LOGD(TAG, "resumed");
        break;
    }
}

esp_err_t usb_keyboard_init(void)
{
    s_send_lock = xSemaphoreCreateMutex();
    if (s_send_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    tinyusb_config_t config = TINYUSB_DEFAULT_CONFIG(usb_event_cb);
    config.descriptor.device = NULL; // esp_tinyusb default: class 0, Espressif VID
    config.descriptor.string = s_strings;
    config.descriptor.string_count = sizeof(s_strings) / sizeof(s_strings[0]);
    config.descriptor.full_speed_config = s_config_descriptor;

    esp_err_t err = tinyusb_driver_install(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "tinyusb_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "TinyUSB installed, waiting for host");
    return ESP_OK;
}

bool usb_keyboard_is_ready(void)
{
    return tud_mounted() && tud_hid_n_ready(HID_INSTANCE_KEYBOARD);
}

// Send `len` bytes on HID `instance`, waking a suspended host first, and
// remember them in `last` for GET_REPORT.
static esp_err_t send_on(uint8_t instance, const uint8_t *data, uint16_t len, uint8_t *last)
{
    if (s_send_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!tud_mounted()) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_send_lock, portMAX_DELAY);

    // A keypress while the host sleeps should wake it, like a real keyboard.
    // This only works if the host enabled remote wakeup; otherwise we time out
    // below and the report is dropped.
    if (tud_suspended()) {
        tinyusb_remote_wakeup();
    }

    esp_err_t result = ESP_ERR_TIMEOUT;
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(SEND_TIMEOUT_MS);
    do {
        if (tud_hid_n_ready(instance)) {
            if (tud_hid_n_report(instance, 0, data, len)) {
                memcpy(last, data, len);
                result = ESP_OK;
            } else {
                result = ESP_FAIL;
            }
            break;
        }
        vTaskDelay(1);
    } while ((int32_t)(deadline - xTaskGetTickCount()) > 0);

    xSemaphoreGive(s_send_lock);

    if (result != ESP_OK) {
        // The caller decides whether to retry or warn; keep this quiet.
        ESP_LOGD(TAG, "report not sent: %s", esp_err_to_name(result));
    }
    return result;
}

esp_err_t usb_keyboard_send_report(const uint8_t report[USB_KEYBOARD_REPORT_LEN])
{
    return send_on(HID_INSTANCE_KEYBOARD, report, USB_KEYBOARD_REPORT_LEN, s_last_report);
}

esp_err_t usb_keyboard_send_consumer(const uint16_t usages[USB_KEYBOARD_CONSUMER_MAX])
{
#if CONFIG_USB_KEYBOARD_MEDIA_KEYS
    uint8_t report[CONSUMER_REPORT_LEN];
    for (int i = 0; i < USB_KEYBOARD_CONSUMER_MAX; i++) {
        report[2 * i] = usages[i] & 0xFF;
        report[2 * i + 1] = usages[i] >> 8;
    }
    return send_on(HID_INSTANCE_CONSUMER, report, sizeof(report), s_last_consumer);
#else
    (void)usages;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t usb_keyboard_release_all(void)
{
    static const uint8_t empty[USB_KEYBOARD_REPORT_LEN] = {0};
    esp_err_t err = usb_keyboard_send_report(empty);
#if CONFIG_USB_KEYBOARD_MEDIA_KEYS
    static const uint16_t no_usages[USB_KEYBOARD_CONSUMER_MAX] = {0};
    esp_err_t consumer_err = usb_keyboard_send_consumer(no_usages);
    if (err == ESP_OK) {
        err = consumer_err;
    }
#endif
    return err;
}

// ---- TinyUSB HID callbacks ----

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
#if CONFIG_USB_KEYBOARD_MEDIA_KEYS
    if (instance == HID_INSTANCE_CONSUMER) {
        return s_consumer_report_descriptor;
    }
#endif
    (void)instance;
    return s_keyboard_report_descriptor;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                               uint8_t *buffer, uint16_t reqlen)
{
    (void)report_id;
    const uint8_t *last = s_last_report;
    uint16_t len = USB_KEYBOARD_REPORT_LEN;
    if (instance == HID_INSTANCE_CONSUMER) {
        last = s_last_consumer;
        len = CONSUMER_REPORT_LEN;
    }
    if (report_type != HID_REPORT_TYPE_INPUT || reqlen < len) {
        return 0;
    }
    memcpy(buffer, last, len);
    return len;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize)
{
    (void)report_id;
    if (instance != HID_INSTANCE_KEYBOARD) {
        return;
    }
    // LED output report (Num/Caps/Scroll Lock). Not forwarded to the BLE
    // keyboard yet.
    if (report_type == HID_REPORT_TYPE_OUTPUT && bufsize >= 1) {
        ESP_LOGD(TAG, "host LEDs: 0x%02x", buffer[0]);
    }
}

void tud_hid_set_protocol_cb(uint8_t instance, uint8_t protocol)
{
    if (instance != HID_INSTANCE_KEYBOARD) {
        return;
    }
    ESP_LOGI(TAG, "host selected %s protocol",
             protocol == HID_PROTOCOL_BOOT ? "boot" : "report");
}
