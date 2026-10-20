#include "ble_keyboard.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_random.h"
#include "nvs.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nimble/nimble_npl.h"
#include "sdkconfig.h"
#include "services/gap/ble_svc_gap.h"

#include "hid_report_map.h"

#if CONFIG_BT_HCI_LOG_DEBUG_EN
#include "hci_log/bt_hci_log.h"
#endif

static const char *TAG = "ble_keyboard";

// Provided by NimBLE's NVS-backed store (CONFIG_BT_NIMBLE_NVS_PERSIST) but not
// declared in any public header.
void ble_store_config_init(void);

// Exported by NimBLE but only declared in its private ble_sm_priv.h: the
// number of security procedures in flight. Needed to work around a NimBLE
// quirk: calling ble_gap_security_initiate() while a security procedure is
// already running returns BLE_HS_EALREADY *and removes the running
// procedure*. The link still gets encrypted, but BLE_GAP_EVENT_ENC_CHANGE is
// never delivered, so we'd wait forever. Keyboards commonly send a Security
// Request the moment they connect, which starts such a procedure before we
// even see BLE_GAP_EVENT_CONNECT.
int ble_sm_num_procs(void);

#define UUID_HID_SERVICE        0x1812
#define UUID_BOOT_KB_INPUT      0x2A22
#define UUID_PROTOCOL_MODE      0x2A4E
#define UUID_REPORT_MAP         0x2A4B
#define UUID_REPORT             0x2A4D
#define UUID_CCCD               0x2902
#define UUID_REPORT_REFERENCE   0x2908
#define APPEARANCE_HID_GENERIC  0x03C0
#define APPEARANCE_KEYBOARD     0x03C1

#define PROTOCOL_MODE_BOOT      0x00
#define PROTOCOL_MODE_REPORT    0x01
#define REPORT_TYPE_INPUT       0x01
#define CONNECT_TIMEOUT_MS      10000
#define MAX_HID_SERVICES        3
#define MAX_BONDED_PEERS        MYNEWT_VAL(BLE_STORE_MAX_BONDS)
#define PAIRING_RETRY_DELAY_MS  5000
#define NVS_NAMESPACE           "ble_keyboard"
#define NVS_KEY_OWN_ADDR        "own_addr"
#define MAX_REPORT_CHRS         8
#define MAX_REPORT_MAP_LEN      512   // HOGP caps the Report Map at 512 bytes
#define MAX_INPUT_REPORT_LEN    64

// Everything below is only touched from the NimBLE host task (GAP/GATT
// callbacks and sync), so it needs no locking.

static ble_keyboard_config_t s_config;
static uint8_t s_own_addr_type;

// After a failed pairing, wait before scanning again instead of hammering the
// keyboard with back-to-back attempts.
static struct ble_npl_callout s_rescan_timer;
static bool s_delay_next_scan;

typedef struct {
    uint16_t start_handle;
    uint16_t end_handle;
} hid_service_t;

// A notifying characteristic and the descriptors we need from it.
typedef struct {
    uint16_t val_handle;
    uint16_t end_handle;        // last handle that can hold its descriptors
    uint16_t cccd_handle;
    uint16_t ref_handle;        // Report Reference (report characteristics only)
    uint8_t report_id;
    uint8_t report_type;
} report_chr_t;

static struct {
    uint16_t conn_handle;
    hid_service_t services[MAX_HID_SERVICES];
    int service_count;
    int service_index;          // service currently being searched
    report_chr_t boot_in;       // Boot Keyboard Input Report, if present
    report_chr_t reports[MAX_REPORT_CHRS]; // input Report characteristics
    int report_count;
    int ref_index;              // next Report Reference to read
    uint16_t report_map_handle;
    uint16_t protocol_mode_handle;
    bool report_mode;           // true: translate via s_layout; false: boot reports
    uint16_t input_val_handle;  // the keyboard characteristic we subscribed to
    bool have_consumer;         // s_consumer_layout is valid (report mode only)
    uint16_t consumer_val_handle; // consumer characteristic we subscribed to, if any
    bool discovery_started;
} s_conn;

static uint8_t s_report_map[MAX_REPORT_MAP_LEN];
static uint16_t s_report_map_len;
static hid_layout_t s_layout;
static hid_layout_t s_consumer_layout;

_Static_assert(BLE_KEYBOARD_CONSUMER_MAX == HID_CONSUMER_MAX_USAGES,
               "consumer usage counts must match");

static void start_scan(void);
static void discover_characteristics(void);

// ---- helpers ----

static void set_state(ble_keyboard_state_t state)
{
    if (s_config.on_state != NULL) {
        s_config.on_state(state, s_config.ctx);
    }
}

static const char *addr_str(const ble_addr_t *addr)
{
    static char buf[18];
    const uint8_t *v = addr->val;
    snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x",
             v[5], v[4], v[3], v[2], v[1], v[0]);
    return buf;
}

static void reset_conn_state(void)
{
    memset(&s_conn, 0, sizeof(s_conn));
    s_conn.conn_handle = BLE_HS_CONN_HANDLE_NONE;
}

// Start tracking a connection the first time any event names it. A no-op if
// we're already tracking it, so discovery progress isn't lost.
static void adopt_connection(uint16_t conn_handle)
{
    if (s_conn.conn_handle != conn_handle) {
        reset_conn_state();
        s_conn.conn_handle = conn_handle;
    }
}

static void disconnect(void)
{
    if (s_conn.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(s_conn.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
}

static int bonded_peers(ble_addr_t *out, int max)
{
    int count = 0;
    if (ble_store_util_bonded_peers(out, &count, max) != 0) {
        return 0;
    }
    return count;
}

// An advertiser matches a bond if its identity address matches. When the
// keyboard advertises with a resolvable private address, the controller
// resolves it (NimBLE loads bonded IRKs into the resolving list at startup)
// and reports the identity address with type PUBLIC_ID / RANDOM_ID, so
// compare the public/random bit only.
static bool is_bonded_addr(const ble_addr_t *addr)
{
    ble_addr_t peers[MAX_BONDED_PEERS];
    int count = bonded_peers(peers, MAX_BONDED_PEERS);
    for (int i = 0; i < count; i++) {
        if ((peers[i].type & 1) == (addr->type & 1) &&
            memcmp(peers[i].val, addr->val, sizeof(addr->val)) == 0) {
            return true;
        }
    }
    return false;
}

// Pairing-mode filter: advertises the HID service or a keyboard appearance,
// and doesn't advertise some other HID appearance (mouse, gamepad, ...).
static bool adv_is_keyboard(const struct ble_hs_adv_fields *fields)
{
    if (fields->appearance_is_present && fields->appearance != APPEARANCE_KEYBOARD &&
        fields->appearance != APPEARANCE_HID_GENERIC) {
        return false;
    }
    for (int i = 0; i < fields->num_uuids16; i++) {
        if (ble_uuid_u16(&fields->uuids16[i].u) == UUID_HID_SERVICE) {
            return true;
        }
    }
    return fields->appearance_is_present && fields->appearance == APPEARANCE_KEYBOARD;
}

// ---- report delivery ----

static void deliver_report(const uint8_t report[BLE_KEYBOARD_REPORT_LEN])
{
#if CONFIG_BLE_KEYBOARD_LOG_REPORTS
    ESP_LOGI(TAG, "report %02x %02x %02x %02x %02x %02x %02x %02x",
             report[0], report[1], report[2], report[3],
             report[4], report[5], report[6], report[7]);
#endif
    if (s_config.on_report != NULL) {
        s_config.on_report(report, s_config.ctx);
    }
}

static void handle_input_notification(struct os_mbuf *om)
{
    uint8_t report[BLE_KEYBOARD_REPORT_LEN] = {0};

    if (s_conn.report_mode) {
        uint8_t raw[MAX_INPUT_REPORT_LEN] = {0};
        uint16_t len = OS_MBUF_PKTLEN(om);
        if (len > sizeof(raw)) {
            len = sizeof(raw);
        }
        os_mbuf_copydata(om, 0, len, raw);
        hid_kbd_translate(&s_layout, raw, len, report);
    } else {
        uint16_t len = OS_MBUF_PKTLEN(om);
        if (len > BLE_KEYBOARD_REPORT_LEN) {
            len = BLE_KEYBOARD_REPORT_LEN;
        }
        os_mbuf_copydata(om, 0, len, report);
    }
    deliver_report(report);
}

static void handle_consumer_notification(struct os_mbuf *om)
{
    uint8_t raw[MAX_INPUT_REPORT_LEN] = {0};
    uint16_t len = OS_MBUF_PKTLEN(om);
    if (len > sizeof(raw)) {
        len = sizeof(raw);
    }
    os_mbuf_copydata(om, 0, len, raw);

    uint16_t usages[BLE_KEYBOARD_CONSUMER_MAX];
    hid_consumer_translate(&s_consumer_layout, raw, len, usages);
#if CONFIG_BLE_KEYBOARD_LOG_REPORTS
    ESP_LOGI(TAG, "consumer %04x %04x %04x %04x", usages[0], usages[1], usages[2], usages[3]);
#endif
    if (s_config.on_consumer != NULL) {
        s_config.on_consumer(usages, s_config.ctx);
    }
}

// ---- GATT: subscribe ----

static void become_ready(void)
{
    ESP_LOGI(TAG, "keyboard ready (%s protocol%s)", s_conn.report_mode ? "report" : "boot",
             s_conn.consumer_val_handle != 0 ? ", media keys" : "");
    set_state(BLE_KEYBOARD_STATE_READY);
}

static int write_cccd(uint16_t cccd_handle, ble_gatt_attr_fn *cb)
{
    const uint8_t enable_notify[2] = {0x01, 0x00};
    return ble_gattc_write_flat(s_conn.conn_handle, cccd_handle,
                                enable_notify, sizeof(enable_notify), cb, NULL);
}

static int on_consumer_cccd_written(uint16_t conn_handle, const struct ble_gatt_error *error,
                                    struct ble_gatt_attr *attr, void *arg)
{
    (void)conn_handle;
    (void)attr;
    (void)arg;
    if (error->status != 0) {
        // Media keys are optional; the keyboard still works without them.
        ESP_LOGW(TAG, "subscribing to media keys failed (status %d)", error->status);
        s_conn.consumer_val_handle = 0;
    }
    become_ready();
    return 0;
}

static const report_chr_t *find_input_report(uint8_t report_id);

static void subscribe_consumer_report(void)
{
    const report_chr_t *r = s_conn.have_consumer
                                ? find_input_report(s_consumer_layout.report_id)
                                : NULL;
    if (r == NULL || r->cccd_handle == 0 || r->val_handle == s_conn.input_val_handle) {
        become_ready();
        return;
    }
    s_conn.consumer_val_handle = r->val_handle;
    int rc = write_cccd(r->cccd_handle, on_consumer_cccd_written);
    if (rc != 0) {
        ESP_LOGW(TAG, "media key CCCD write failed to start (rc %d)", rc);
        s_conn.consumer_val_handle = 0;
        become_ready();
    }
}

static int on_cccd_written(uint16_t conn_handle, const struct ble_gatt_error *error,
                           struct ble_gatt_attr *attr, void *arg)
{
    (void)conn_handle;
    (void)attr;
    (void)arg;
    if (error->status != 0) {
        ESP_LOGE(TAG, "subscribing to keyboard reports failed (status %d)", error->status);
        disconnect();
        return 0;
    }
    if (s_conn.report_mode) {
        subscribe_consumer_report();
    } else {
        become_ready();
    }
    return 0;
}

static void subscribe(uint16_t val_handle, uint16_t cccd_handle)
{
    s_conn.input_val_handle = val_handle;

    if (s_conn.protocol_mode_handle != 0) {
        // HOGP: Protocol Mode is written without response.
        const uint8_t mode = s_conn.report_mode ? PROTOCOL_MODE_REPORT : PROTOCOL_MODE_BOOT;
        int rc = ble_gattc_write_no_rsp_flat(s_conn.conn_handle, s_conn.protocol_mode_handle,
                                             &mode, sizeof(mode));
        if (rc != 0) {
            ESP_LOGW(TAG, "setting protocol mode failed (rc %d)", rc);
        }
    }

    int rc = write_cccd(cccd_handle, on_cccd_written);
    if (rc != 0) {
        ESP_LOGE(TAG, "CCCD write failed to start (rc %d)", rc);
        disconnect();
    }
}

// ---- GATT: report mode setup ----
//
// Report Map -> layout of the keyboard report (which report ID, which bits).
// Report Reference descriptors -> which Report characteristic carries that ID.

static void read_next_report_reference(void);

// The input Report characteristic carrying `report_id`. Without Report
// Reference descriptors, a lone input report is assumed to be the one.
static const report_chr_t *find_input_report(uint8_t report_id)
{
    for (int i = 0; i < s_conn.report_count; i++) {
        const report_chr_t *r = &s_conn.reports[i];
        bool id_matches = r->ref_handle != 0
                              ? (r->report_type == REPORT_TYPE_INPUT && r->report_id == report_id)
                              : (s_conn.report_count == 1);
        if (id_matches) {
            return r;
        }
    }
    return NULL;
}

static void subscribe_keyboard_report(void)
{
    const report_chr_t *r = find_input_report(s_layout.report_id);
    if (r == NULL) {
        ESP_LOGE(TAG, "no Report characteristic carries keyboard report ID %u", s_layout.report_id);
        return;
    }
    if (r->cccd_handle == 0) {
        ESP_LOGE(TAG, "keyboard input report has no CCCD; can't subscribe");
        return;
    }
    subscribe(r->val_handle, r->cccd_handle);
}

static int on_report_reference(uint16_t conn_handle, const struct ble_gatt_error *error,
                               struct ble_gatt_attr *attr, void *arg)
{
    (void)conn_handle;
    report_chr_t *r = arg;
    if (error->status == 0) {
        uint8_t ref[2] = {0};
        if (OS_MBUF_PKTLEN(attr->om) >= sizeof(ref)) {
            os_mbuf_copydata(attr->om, 0, sizeof(ref), ref);
            r->report_id = ref[0];
            r->report_type = ref[1];
        }
#if CONFIG_BLE_KEYBOARD_LOG_REPORTS
        ESP_LOGI(TAG, "  report handle %u: id %u type %u", r->val_handle, r->report_id, r->report_type);
#endif
    } else {
        ESP_LOGW(TAG, "reading Report Reference failed (status %d)", error->status);
    }
    s_conn.ref_index++;
    read_next_report_reference();
    return 0;
}

static void read_next_report_reference(void)
{
    while (s_conn.ref_index < s_conn.report_count) {
        report_chr_t *r = &s_conn.reports[s_conn.ref_index];
        if (r->ref_handle != 0) {
            int rc = ble_gattc_read(s_conn.conn_handle, r->ref_handle, on_report_reference, r);
            if (rc != 0) {
                ESP_LOGE(TAG, "Report Reference read failed to start (rc %d)", rc);
                disconnect();
            }
            return;
        }
        s_conn.ref_index++;
    }
    subscribe_keyboard_report();
}

static void log_layout(void)
{
    bool nkro = false;
    for (int i = 0; i < s_layout.field_count; i++) {
        const hid_field_t *f = &s_layout.fields[i];
        if (!f->is_array && f->usage_min < 0xE0 && f->count > 8) {
            nkro = true;
        }
    }
    ESP_LOGI(TAG, "keyboard report: id %u, %u bytes, %s", s_layout.report_id,
             (s_layout.report_bits + 7) / 8, nkro ? "NKRO bitmap" : "keycode array");
}

static int on_report_map(uint16_t conn_handle, const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg)
{
    (void)conn_handle;
    (void)arg;
    if (error->status == 0) {
        uint16_t chunk = OS_MBUF_PKTLEN(attr->om);
        if (attr->offset + chunk > sizeof(s_report_map)) {
            chunk = attr->offset < sizeof(s_report_map) ? sizeof(s_report_map) - attr->offset : 0;
        }
        if (chunk == 0) {
            return 0; // longer than HOGP allows; parse what fits
        }
        os_mbuf_copydata(attr->om, 0, chunk, &s_report_map[attr->offset]);
        if (attr->offset + chunk > s_report_map_len) {
            s_report_map_len = attr->offset + chunk;
        }
        return 0;
    }
    if (error->status != BLE_HS_EDONE) {
        ESP_LOGE(TAG, "reading Report Map failed (status %d)", error->status);
        disconnect();
        return 0;
    }

    if (!hid_kbd_layout_parse(s_report_map, s_report_map_len, &s_layout)) {
        // Stay connected rather than looping; see the note in on_characteristic().
        ESP_LOGE(TAG, "Report Map (%u bytes) has no keyboard input report", s_report_map_len);
        return 0;
    }
    if ((s_layout.report_bits + 7) / 8 > MAX_INPUT_REPORT_LEN) {
        ESP_LOGE(TAG, "keyboard report is %u bytes; only %d supported",
                 (s_layout.report_bits + 7) / 8, MAX_INPUT_REPORT_LEN);
        return 0;
    }
    log_layout();

    s_conn.have_consumer = hid_consumer_layout_parse(s_report_map, s_report_map_len, &s_consumer_layout) &&
                           (s_consumer_layout.report_bits + 7) / 8 <= MAX_INPUT_REPORT_LEN;
    if (s_conn.have_consumer) {
        ESP_LOGI(TAG, "media key report: id %u, %u bytes", s_consumer_layout.report_id,
                 (s_consumer_layout.report_bits + 7) / 8);
    }

    s_conn.ref_index = 0;
    read_next_report_reference();
    return 0;
}

static void read_report_map(void)
{
    s_report_map_len = 0;
    int rc = ble_gattc_read_long(s_conn.conn_handle, s_conn.report_map_handle, 0, on_report_map, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Report Map read failed to start (rc %d)", rc);
        disconnect();
    }
}

// ---- GATT: discovery ----

static int on_descriptor(uint16_t conn_handle, const struct ble_gatt_error *error,
                         uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc, void *arg)
{
    (void)conn_handle;
    (void)chr_val_handle;
    (void)arg;
    if (error->status == 0) {
        uint16_t uuid = ble_uuid_u16(&dsc->uuid.u);
        uint16_t h = dsc->handle;
        if (uuid != UUID_CCCD && uuid != UUID_REPORT_REFERENCE) {
            return 0;
        }
        if (s_conn.boot_in.val_handle != 0 &&
            h > s_conn.boot_in.val_handle && h <= s_conn.boot_in.end_handle) {
            if (uuid == UUID_CCCD) {
                s_conn.boot_in.cccd_handle = h;
            }
            return 0;
        }
        for (int i = 0; i < s_conn.report_count; i++) {
            report_chr_t *r = &s_conn.reports[i];
            if (h > r->val_handle && h <= r->end_handle) {
                if (uuid == UUID_CCCD) {
                    r->cccd_handle = h;
                } else {
                    r->ref_handle = h;
                }
                break;
            }
        }
        return 0;
    }
    if (error->status != BLE_HS_EDONE) {
        ESP_LOGE(TAG, "descriptor discovery failed (status %d)", error->status);
        disconnect();
        return 0;
    }

    if (!s_conn.report_mode) {
        if (s_conn.boot_in.cccd_handle == 0) {
            ESP_LOGE(TAG, "Boot Keyboard Input Report has no CCCD; can't subscribe");
            disconnect();
            return 0;
        }
        subscribe(s_conn.boot_in.val_handle, s_conn.boot_in.cccd_handle);
    } else {
        read_report_map();
    }
    return 0;
}

// Characteristic discovery only tells us where each characteristic starts;
// one ends where the next begins. s_pending_end points at the end_handle of
// the last characteristic we care about until the next one shows up.
static uint16_t *s_pending_end;

static int on_characteristic(uint16_t conn_handle, const struct ble_gatt_error *error,
                             const struct ble_gatt_chr *chr, void *arg)
{
    (void)arg;
    const hid_service_t *svc = &s_conn.services[s_conn.service_index];

    if (error->status == 0) {
#if CONFIG_BLE_KEYBOARD_LOG_REPORTS
        char uuid_buf[BLE_UUID_STR_LEN];
        ESP_LOGI(TAG, "  HID service %d: chr %s handle %u props 0x%02x",
                 s_conn.service_index, ble_uuid_to_str(&chr->uuid.u, uuid_buf),
                 chr->val_handle, chr->properties);
#endif
        if (s_pending_end != NULL) {
            *s_pending_end = chr->def_handle - 1;
            s_pending_end = NULL;
        }
        switch (ble_uuid_u16(&chr->uuid.u)) {
        case UUID_BOOT_KB_INPUT:
            if (s_conn.boot_in.val_handle == 0) {
                s_conn.boot_in.val_handle = chr->val_handle;
                s_pending_end = &s_conn.boot_in.end_handle;
            }
            break;
        case UUID_REPORT:
            // Only input reports notify; output/feature reports are skipped.
            if ((chr->properties & BLE_GATT_CHR_PROP_NOTIFY) &&
                s_conn.report_count < MAX_REPORT_CHRS) {
                report_chr_t *r = &s_conn.reports[s_conn.report_count++];
                r->val_handle = chr->val_handle;
                s_pending_end = &r->end_handle;
            }
            break;
        case UUID_REPORT_MAP:
            s_conn.report_map_handle = chr->val_handle;
            break;
        case UUID_PROTOCOL_MODE:
            s_conn.protocol_mode_handle = chr->val_handle;
            break;
        }
        return 0;
    }

    if (error->status != BLE_HS_EDONE) {
        ESP_LOGE(TAG, "characteristic discovery failed (status %d)", error->status);
        disconnect();
        return 0;
    }
    if (s_pending_end != NULL) {
        *s_pending_end = svc->end_handle;
        s_pending_end = NULL;
    }

    // Prefer the boot report: it's the fixed 8-byte format USB wants anyway.
    uint16_t first_val;
    if (s_conn.boot_in.val_handle != 0) {
        s_conn.report_mode = false;
        first_val = s_conn.boot_in.val_handle;
    } else if (s_conn.report_count > 0 && s_conn.report_map_handle != 0) {
        s_conn.report_mode = true;
        first_val = s_conn.reports[0].val_handle;
    } else if (s_conn.service_index + 1 < s_conn.service_count) {
        // Try the next HID service instance.
        s_conn.service_index++;
        s_conn.protocol_mode_handle = 0;
        s_conn.report_map_handle = 0;
        s_conn.report_count = 0;
        discover_characteristics();
        return 0;
    } else {
        // Stay connected (and keep the bond) rather than disconnecting into a
        // reconnect loop. Deleting the bond here would leave the keyboard
        // holding a bond we no longer have, and many keyboards then refuse to
        // pair again until it's cleared on their side.
        ESP_LOGE(TAG, "HID service has neither a Boot Keyboard Input Report (0x2A22) "
                      "nor input Reports with a Report Map; can't read this keyboard");
        return 0;
    }

    if (first_val >= svc->end_handle) {
        ESP_LOGE(TAG, "keyboard input report has no descriptors; can't subscribe");
        disconnect();
        return 0;
    }
    // NimBLE takes the characteristic *value* handle here and starts looking
    // at the handle after it.
    int rc = ble_gattc_disc_all_dscs(conn_handle, first_val, svc->end_handle,
                                     on_descriptor, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "descriptor discovery failed to start (rc %d)", rc);
        disconnect();
    }
    return 0;
}

static void discover_characteristics(void)
{
    const hid_service_t *svc = &s_conn.services[s_conn.service_index];
    s_pending_end = NULL;
    int rc = ble_gattc_disc_all_chrs(s_conn.conn_handle, svc->start_handle, svc->end_handle,
                                     on_characteristic, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "characteristic discovery failed to start (rc %d)", rc);
        disconnect();
    }
}

static int on_service(uint16_t conn_handle, const struct ble_gatt_error *error,
                      const struct ble_gatt_svc *service, void *arg)
{
    (void)arg;
    if (error->status == 0) {
        if (s_conn.service_count < MAX_HID_SERVICES) {
            s_conn.services[s_conn.service_count].start_handle = service->start_handle;
            s_conn.services[s_conn.service_count].end_handle = service->end_handle;
            s_conn.service_count++;
        }
        return 0;
    }
    if (error->status != BLE_HS_EDONE) {
        ESP_LOGE(TAG, "service discovery failed (status %d)", error->status);
        disconnect();
        return 0;
    }
    if (s_conn.service_count == 0) {
        ESP_LOGE(TAG, "device has no HID service (0x1812)");
        disconnect();
        return 0;
    }
    ESP_LOGI(TAG, "found %d HID service(s)", s_conn.service_count);
    s_conn.service_index = 0;
    discover_characteristics();
    return 0;
}

static void start_discovery(void)
{
    if (s_conn.discovery_started) {
        return;
    }
    s_conn.discovery_started = true;
    ble_uuid16_t hid_uuid = BLE_UUID16_INIT(UUID_HID_SERVICE);
    int rc = ble_gattc_disc_svc_by_uuid(s_conn.conn_handle, &hid_uuid.u, on_service, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "service discovery failed to start (rc %d)", rc);
        disconnect();
    }
}

// ---- GAP ----

static void handle_passkey_action(const struct ble_gap_event *event)
{
    struct ble_sm_io pkey = {0};
    pkey.action = event->passkey.params.action;

    switch (pkey.action) {
    case BLE_SM_IOACT_DISP:
        pkey.passkey = esp_random() % 1000000;
        ESP_LOGW(TAG, "**************************************************");
        ESP_LOGW(TAG, "  PAIRING: type %06" PRIu32 " on the keyboard, then Enter", pkey.passkey);
        ESP_LOGW(TAG, "**************************************************");
        break;
    case BLE_SM_IOACT_NUMCMP:
        // We have no buttons to confirm with; accept and show the number in case
        // the keyboard displays it.
        ESP_LOGW(TAG, "PAIRING: numeric comparison %06" PRIu32 ", accepting",
                 event->passkey.params.numcmp);
        pkey.numcmp_accept = 1;
        break;
    default:
        ESP_LOGE(TAG, "unsupported pairing action %d", pkey.action);
        return;
    }

    int rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_sm_inject_io failed (rc %d)", rc);
    }
}

static void log_security_failure(int status)
{
    if (status == BLE_HS_ERR_HCI_BASE + BLE_ERR_CONN_SPVN_TMO) {
        ESP_LOGE(TAG, "radio link lost while securing the connection (supervision "
                      "timeout); keyboard asleep, out of range, or busy with another host?");
    } else if (status == BLE_HS_ERR_HCI_BASE + BLE_ERR_PINKEY_MISSING) {
        ESP_LOGE(TAG, "keyboard no longer has our bond; reset this bridge's bonds "
                      "and pair again");
    } else if (status == BLE_HS_SM_PEER_ERR(BLE_SM_ERR_AUTHREQ)) {
        ESP_LOGE(TAG, "keyboard refused to pair (authentication requirements). It most "
                      "likely still holds a bond for this bridge's address: reset this "
                      "bridge's bonds (which also picks a new address), or clear the "
                      "bond on the keyboard (ZMK: BT_CLR on the active profile)");
    } else if (status >= BLE_HS_ERR_SM_PEER_BASE && status < BLE_HS_ERR_SM_PEER_BASE + 0x100) {
        ESP_LOGE(TAG, "keyboard rejected pairing (SM error 0x%02x)", status - BLE_HS_ERR_SM_PEER_BASE);
    } else if (status >= BLE_HS_ERR_SM_US_BASE && status < BLE_HS_ERR_SM_US_BASE + 0x100) {
        ESP_LOGE(TAG, "pairing aborted locally (SM error 0x%02x)", status - BLE_HS_ERR_SM_US_BASE);
    } else {
        ESP_LOGE(TAG, "encryption/pairing failed (status 0x%03x)", status);
    }
}

static int gap_event(struct ble_gap_event *event, void *arg);

static void try_connect(const struct ble_gap_disc_desc *disc, const char *name)
{
    int rc = ble_gap_disc_cancel();
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "scan cancel failed (rc %d)", rc);
        return;
    }

    ESP_LOGI(TAG, "connecting to %s%s%s", addr_str(&disc->addr),
             name[0] ? " " : "", name);
    set_state(BLE_KEYBOARD_STATE_CONNECTING);

    // NimBLE's defaults, except a 4 s supervision timeout instead of 2.56 s, so
    // a few missed packets while the link is being set up don't drop it. The
    // keyboard usually renegotiates parameters once connected anyway.
    const struct ble_gap_conn_params params = {
        .scan_itvl = 0x0010,
        .scan_window = 0x0010,
        .itvl_min = BLE_GAP_INITIAL_CONN_ITVL_MIN,
        .itvl_max = BLE_GAP_INITIAL_CONN_ITVL_MAX,
        .latency = 0,
        .supervision_timeout = 400, // x 10 ms
        .min_ce_len = 0,
        .max_ce_len = 0,
    };
    rc = ble_gap_connect(s_own_addr_type, &disc->addr, CONNECT_TIMEOUT_MS, &params, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "connect failed to start (rc %d)", rc);
        start_scan();
    }
}

static void handle_disc(const struct ble_gap_disc_desc *disc)
{
    struct ble_hs_adv_fields fields;
    char name[32] = "";
    if (ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data) == 0 && fields.name_len > 0) {
        int n = fields.name_len < sizeof(name) - 1 ? fields.name_len : sizeof(name) - 1;
        memcpy(name, fields.name, n);
        name[n] = '\0';
    } else {
        memset(&fields, 0, sizeof(fields));
    }

    ble_addr_t peers[1];
    bool have_bond = bonded_peers(peers, 1) > 0;

    if (have_bond) {
        // Only ever reconnect to a bonded keyboard. Directed advertising counts.
        if (is_bonded_addr(&disc->addr)) {
            try_connect(disc, name);
        }
        return;
    }

    if (disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND &&
        disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) {
        return; // not connectable (scan responses carry no new info we gate on)
    }
    if (adv_is_keyboard(&fields)) {
        ESP_LOGI(TAG, "found keyboard %s (%s), pairing", addr_str(&disc->addr),
                 name[0] ? name : "no name");
        try_connect(disc, name);
    }
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    struct ble_gap_conn_desc desc;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        handle_disc(&event->disc);
        return 0;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        // Only happens if scanning was stopped for a reason other than our
        // own connect; keep looking.
        if (s_conn.conn_handle == BLE_HS_CONN_HANDLE_NONE &&
            event->disc_complete.reason != 0) {
            start_scan();
        }
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            ESP_LOGW(TAG, "connection failed (status %d)", event->connect.status);
            start_scan();
            return 0;
        }
        adopt_connection(event->connect.conn_handle);
        // NimBLE delivers this event only after reading the peer's features
        // and version, and keyboards often request security as soon as the
        // link is up, so the link may already be encrypted by now (and
        // discovery already running).
        if (ble_gap_conn_find(s_conn.conn_handle, &desc) == 0 && desc.sec_state.encrypted) {
            ESP_LOGI(TAG, "connected (already encrypted)");
            start_discovery();
            return 0;
        }
        if (ble_sm_num_procs() > 0) {
            // The keyboard already asked for security and NimBLE is handling
            // it; ENC_CHANGE will follow. Starting our own would cancel it.
            ESP_LOGI(TAG, "connected; keyboard is securing the link");
            return 0;
        }
        ESP_LOGI(TAG, "connected; securing link");
        // Encrypt with the stored LTK, or pair if there's no bond yet. HID
        // characteristics usually require an encrypted link.
        {
            int rc = ble_gap_security_initiate(s_conn.conn_handle);
            if (rc != 0 && rc != BLE_HS_EALREADY) {
                ESP_LOGE(TAG, "security initiate failed (rc %d)", rc);
                disconnect();
            }
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected (reason 0x%03x)", event->disconnect.reason);
#if CONFIG_BT_HCI_LOG_DEBUG_EN
        // Diagnostic: dump the HCI trace (recorded with negligible timing cost)
        // the first time a connection times out before it became usable.
        {
            static bool dumped;
            if (!dumped && s_conn.input_val_handle == 0 &&
                event->disconnect.reason == BLE_HS_ERR_HCI_BASE + BLE_ERR_CONN_SPVN_TMO) {
                dumped = true;
                ESP_LOGW(TAG, "dumping HCI trace for this failed connection");
                bt_hci_log_hci_data_show();
            }
        }
#endif
        reset_conn_state();
        set_state(BLE_KEYBOARD_STATE_DISCONNECTED);
        if (s_delay_next_scan) {
            s_delay_next_scan = false;
            ESP_LOGI(TAG, "retrying in %d s", PAIRING_RETRY_DELAY_MS / 1000);
            ble_npl_callout_reset(&s_rescan_timer, ble_npl_time_ms_to_ticks32(PAIRING_RETRY_DELAY_MS));
        } else {
            start_scan();
        }
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        // May arrive before BLE_GAP_EVENT_CONNECT; see the note there.
        adopt_connection(event->enc_change.conn_handle);
        if (event->enc_change.status != 0) {
            log_security_failure(event->enc_change.status);
            s_delay_next_scan = true;
            disconnect();
            return 0;
        }
        if (ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0) {
            ESP_LOGI(TAG, "link encrypted (%s%s)",
                     desc.sec_state.bonded ? "bonded" : "not bonded",
                     desc.sec_state.authenticated ? ", authenticated" : "");
        }
        start_discovery();
        return 0;

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        handle_passkey_action(event);
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        // The keyboard wants to pair again with a peer we already have a bond
        // for (it lost its keys). Drop our stale bond and let pairing proceed.
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ESP_LOGW(TAG, "keyboard re-pairing; replacing old bond");
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    case BLE_GAP_EVENT_NOTIFY_RX:
        if (event->notify_rx.conn_handle == s_conn.conn_handle &&
            s_conn.input_val_handle != 0 &&
            event->notify_rx.attr_handle == s_conn.input_val_handle) {
            handle_input_notification(event->notify_rx.om);
        } else if (event->notify_rx.conn_handle == s_conn.conn_handle &&
                   s_conn.consumer_val_handle != 0 &&
                   event->notify_rx.attr_handle == s_conn.consumer_val_handle) {
            handle_consumer_notification(event->notify_rx.om);
        }
        return 0;

    default:
        return 0;
    }
}

static void start_scan(void)
{
    struct ble_gap_disc_params params = {
        .itvl = 0,       // stack defaults
        .window = 0,
        .filter_policy = 0,
        .limited = 0,
        .passive = 0,    // active, so scan responses (names) show up
        .filter_duplicates = 1,
    };

    int rc = ble_gap_disc(s_own_addr_type, BLE_HS_FOREVER, &params, gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "scan failed to start (rc %d)", rc);
        return;
    }

    ble_addr_t peers[1];
    if (bonded_peers(peers, 1) > 0) {
        ESP_LOGI(TAG, "scanning for bonded keyboard %s", addr_str(&peers[0]));
    } else {
        ESP_LOGI(TAG, "no bond stored; scanning for any keyboard to pair");
    }
    set_state(BLE_KEYBOARD_STATE_SCANNING);
}

// ---- host lifecycle ----

// NimBLE's default handler makes room in a full store by deleting the oldest
// bond. Keep that behavior, but say so: a silently lost bond means the
// keyboard has to be re-paired.
static int on_store_status(struct ble_store_status_event *event, void *arg)
{
    if (event->event_code == BLE_STORE_EVENT_OVERFLOW || event->event_code == BLE_STORE_EVENT_FULL) {
        ESP_LOGW(TAG, "bond store full (event %d); NimBLE will delete the oldest bond",
                 event->event_code);
    }
    return ble_store_util_status_rr(event, arg);
}

// The bridge identifies itself with a random static address kept in NVS
// rather than the chip's fixed public address. Clearing bonds also picks a new
// address, so a keyboard that still holds keys for the old one (and would
// refuse to re-pair with it) sees a brand-new device instead.
static void set_identity(bool regenerate)
{
    ble_addr_t addr = {.type = BLE_ADDR_RANDOM};
    nvs_handle_t nvs;
    bool loaded = false;

    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err == ESP_OK && !regenerate) {
        size_t len = sizeof(addr.val);
        loaded = nvs_get_blob(nvs, NVS_KEY_OWN_ADDR, addr.val, &len) == ESP_OK &&
                 len == sizeof(addr.val);
    }
    if (!loaded) {
        int rc = ble_hs_id_gen_rnd(0, &addr); // static random, not resolvable
        assert(rc == 0);
        if (err == ESP_OK) {
            nvs_set_blob(nvs, NVS_KEY_OWN_ADDR, addr.val, sizeof(addr.val));
            nvs_commit(nvs);
        }
        ESP_LOGI(TAG, "generated new bridge address");
    }
    if (err == ESP_OK) {
        nvs_close(nvs);
    } else {
        ESP_LOGW(TAG, "NVS unavailable (%s); address won't persist", esp_err_to_name(err));
    }

    int rc = ble_hs_id_set_rnd(addr.val);
    assert(rc == 0);
    s_own_addr_type = BLE_OWN_ADDR_RANDOM;
    ESP_LOGI(TAG, "own address %s (random static)", addr_str(&addr));
}

static void on_sync(void)
{
    int rc;
    bool clear = s_config.clear_bonds;
    s_config.clear_bonds = false;

    if (clear) {
        rc = ble_store_clear();
        ESP_LOGW(TAG, "stored bonds cleared%s", rc == 0 ? "" : " (with errors)");
    }
    set_identity(clear);

    start_scan();
}

static void on_rescan_timer(struct ble_npl_event *ev)
{
    (void)ev;
    start_scan();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "host reset (reason %d)", reason);
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_keyboard_init(const ble_keyboard_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_config = *config;
    reset_conn_state();

#if CONFIG_BT_NIMBLE_LOG_LEVEL_DEBUG
    // Global default stays at INFO; only raise the NimBLE host's own tag.
    esp_log_level_set("NimBLE", ESP_LOG_DEBUG);
#endif

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(err));
        return err;
    }

    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.store_status_cb = on_store_status;

    // We can show a passkey on the console but not type one, so DisplayOnly.
    // Keyboards with a keypad get passkey entry; others fall back to Just Works.
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_DISP_ONLY;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 1;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

#if CONFIG_BT_NIMBLE_GAP_SERVICE
    ble_svc_gap_device_name_set("BLE-USB Keyboard Bridge");
#endif

    ble_store_config_init();
    ble_npl_callout_init(&s_rescan_timer, nimble_port_get_dflt_eventq(), on_rescan_timer, NULL);
    nimble_port_freertos_init(host_task);
    return ESP_OK;
}
