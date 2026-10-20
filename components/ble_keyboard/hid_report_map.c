#include "hid_report_map.h"

#include <string.h>

#define ITEM_TYPE_MAIN      0
#define ITEM_TYPE_GLOBAL    1
#define ITEM_TYPE_LOCAL     2

#define MAIN_INPUT          0x8

#define GLOBAL_USAGE_PAGE   0x0
#define GLOBAL_LOGICAL_MIN  0x1
#define GLOBAL_REPORT_SIZE  0x7
#define GLOBAL_REPORT_ID    0x8
#define GLOBAL_REPORT_COUNT 0x9
#define GLOBAL_PUSH         0xA
#define GLOBAL_POP          0xB

#define LOCAL_USAGE         0x0
#define LOCAL_USAGE_MIN     0x1
#define LOCAL_USAGE_MAX     0x2

#define INPUT_FLAG_CONSTANT 0x01
#define INPUT_FLAG_VARIABLE 0x02

#define USAGE_PAGE_KEYBOARD 0x07
#define USAGE_PAGE_CONSUMER 0x0C
#define USAGE_FIRST_KEY     0x04   // usages below this are error codes
#define USAGE_LEFT_CTRL     0xE0
#define USAGE_RIGHT_GUI     0xE7

#define MAX_REPORT_IDS      16
#define MAX_CANDIDATES      16
#define GLOBAL_STACK_DEPTH  4

typedef struct {
    uint16_t usage_page;
    int32_t logical_min;
    uint32_t report_size;
    uint32_t report_count;
    uint8_t report_id;
} globals_t;

typedef struct {
    uint8_t report_id;
    hid_field_t field;
} candidate_t;

// Running input-report bit offset per report ID.
typedef struct {
    uint8_t id;
    uint16_t bits;
} id_offset_t;

// Parse scratch space. Static rather than on the stack: the parser runs on the
// BLE host task, whose stack is small. Not reentrant, which is fine there.
static candidate_t s_candidates[MAX_CANDIDATES];
static int s_candidate_count;
static id_offset_t s_offsets[MAX_REPORT_IDS];
static int s_offset_count;

static uint32_t item_unsigned(const uint8_t *p, int size)
{
    uint32_t v = 0;
    for (int i = 0; i < size; i++) {
        v |= (uint32_t)p[i] << (8 * i);
    }
    return v;
}

static int32_t item_signed(const uint8_t *p, int size)
{
    uint32_t v = item_unsigned(p, size);
    if (size > 0 && size < 4 && (v & (1u << (8 * size - 1)))) {
        v |= ~0u << (8 * size);
    }
    return (int32_t)v;
}

static uint16_t *offset_for(uint8_t id)
{
    for (int i = 0; i < s_offset_count; i++) {
        if (s_offsets[i].id == id) {
            return &s_offsets[i].bits;
        }
    }
    if (s_offset_count == MAX_REPORT_IDS) {
        return NULL;
    }
    s_offsets[s_offset_count].id = id;
    s_offsets[s_offset_count].bits = 0;
    return &s_offsets[s_offset_count++].bits;
}

// Walk the descriptor, tracking per-report-ID input offsets, and record every
// non-constant input field on `page` as a candidate.
static void collect_fields(const uint8_t *map, size_t len, uint16_t page)
{
    globals_t g = {0};
    globals_t stack[GLOBAL_STACK_DEPTH];
    int stack_depth = 0;

    // Locals reset after every main item.
    uint32_t usage_min = 0, usage_max = 0;
    bool have_range = false;
    uint16_t usage_list[HID_FIELD_MAX_USAGES];
    int usage_list_count = 0;

    s_candidate_count = 0;
    s_offset_count = 0;

    size_t i = 0;
    while (i < len) {
        uint8_t prefix = map[i];
        if (prefix == 0xFE) { // long item: skip
            if (i + 2 >= len) {
                break;
            }
            i += 3 + map[i + 1];
            continue;
        }
        int size = prefix & 0x03;
        if (size == 3) {
            size = 4;
        }
        int type = (prefix >> 2) & 0x03;
        int tag = prefix >> 4;
        if (i + 1 + size > len) {
            break;
        }
        const uint8_t *data = &map[i + 1];
        uint32_t uval = item_unsigned(data, size);
        i += 1 + size;

        if (type == ITEM_TYPE_GLOBAL) {
            switch (tag) {
            case GLOBAL_USAGE_PAGE:   g.usage_page = uval; break;
            case GLOBAL_LOGICAL_MIN:  g.logical_min = item_signed(data, size); break;
            case GLOBAL_REPORT_SIZE:  g.report_size = uval; break;
            case GLOBAL_REPORT_ID:    g.report_id = uval; break;
            case GLOBAL_REPORT_COUNT: g.report_count = uval; break;
            case GLOBAL_PUSH:
                if (stack_depth < GLOBAL_STACK_DEPTH) {
                    stack[stack_depth++] = g;
                }
                break;
            case GLOBAL_POP:
                if (stack_depth > 0) {
                    g = stack[--stack_depth];
                }
                break;
            }
            continue;
        }

        if (type == ITEM_TYPE_LOCAL) {
            // 4-byte usages carry their own page in the top 16 bits; only the
            // low 16 matter for matching within the page.
            switch (tag) {
            case LOCAL_USAGE:
                if (usage_list_count < HID_FIELD_MAX_USAGES) {
                    usage_list[usage_list_count++] = uval & 0xFFFF;
                }
                break;
            case LOCAL_USAGE_MIN: usage_min = uval & 0xFFFF; have_range = true; break;
            case LOCAL_USAGE_MAX: usage_max = uval & 0xFFFF; have_range = true; break;
            }
            continue;
        }

        if (type == ITEM_TYPE_MAIN) {
            if (tag == MAIN_INPUT) {
                uint16_t *offset = offset_for(g.report_id);
                uint32_t bits = g.report_size * g.report_count;
                bool constant = uval & INPUT_FLAG_CONSTANT;

                if (offset != NULL && !constant && g.usage_page == page &&
                    s_candidate_count < MAX_CANDIDATES && g.report_size > 0 &&
                    g.report_size <= 16 && g.report_count > 0 && g.report_count <= 255) {
                    candidate_t *c = &s_candidates[s_candidate_count++];
                    memset(c, 0, sizeof(*c));
                    c->report_id = g.report_id;
                    c->field.bit_offset = *offset;
                    c->field.size = g.report_size;
                    c->field.count = g.report_count;
                    c->field.logical_min = g.logical_min;
                    c->field.is_array = !(uval & INPUT_FLAG_VARIABLE);
                    if (have_range) {
                        c->field.usage_min = usage_min;
                        c->field.usage_max = usage_max;
                    } else if (usage_list_count > 0) {
                        c->field.usage_count = usage_list_count;
                        memcpy(c->field.usages, usage_list, usage_list_count * sizeof(uint16_t));
                        c->field.usage_min = usage_list[0];
                        c->field.usage_max = usage_list[0];
                        for (int u = 1; u < usage_list_count; u++) {
                            if (usage_list[u] > c->field.usage_max) {
                                c->field.usage_max = usage_list[u];
                            }
                        }
                    }
                }
                if (offset != NULL) {
                    *offset += bits;
                }
            }
            // Output/Feature/Collection items don't affect input offsets.
            usage_min = usage_max = 0;
            have_range = false;
            usage_list_count = 0;
        }
    }
}

// Fill `out` with all collected candidates belonging to report `id`.
static void build_layout(uint8_t id, hid_layout_t *out)
{
    memset(out, 0, sizeof(*out));
    out->report_id = id;
    for (int c = 0; c < s_candidate_count && out->field_count < HID_MAX_FIELDS; c++) {
        if (s_candidates[c].report_id == id) {
            out->fields[out->field_count++] = s_candidates[c].field;
        }
    }
    uint16_t *bits = offset_for(id);
    out->report_bits = bits ? *bits : 0;
}

bool hid_kbd_layout_parse(const uint8_t *map, size_t len, hid_layout_t *out)
{
    collect_fields(map, len, USAGE_PAGE_KEYBOARD);

    // Pick the first report ID that carries actual keys, not just modifiers.
    for (int c = 0; c < s_candidate_count; c++) {
        const hid_field_t *f = &s_candidates[c].field;
        if (f->is_array || (f->usage_max >= USAGE_FIRST_KEY && f->usage_min < USAGE_LEFT_CTRL)) {
            build_layout(s_candidates[c].report_id, out);
            return true;
        }
    }
    return false;
}

bool hid_consumer_layout_parse(const uint8_t *map, size_t len, hid_layout_t *out)
{
    collect_fields(map, len, USAGE_PAGE_CONSUMER);
    if (s_candidate_count == 0) {
        return false;
    }
    build_layout(s_candidates[0].report_id, out);
    return true;
}

static uint32_t extract_bits(const uint8_t *data, size_t len, uint32_t offset, uint8_t size)
{
    uint32_t v = 0;
    for (uint8_t b = 0; b < size; b++) {
        uint32_t bit = offset + b;
        if (bit / 8 >= len) {
            break;
        }
        if (data[bit / 8] & (1u << (bit % 8))) {
            v |= 1u << b;
        }
    }
    return v;
}

// Call `emit` with the usage of every active element of `layout`'s fields.
typedef void (*usage_fn)(void *ctx, uint32_t usage);

static void for_each_active_usage(const hid_layout_t *layout, const uint8_t *data, size_t len,
                                  usage_fn emit, void *ctx)
{
    for (int f = 0; f < layout->field_count; f++) {
        const hid_field_t *field = &layout->fields[f];
        for (uint32_t e = 0; e < field->count; e++) {
            uint32_t v = extract_bits(data, len, field->bit_offset + e * field->size, field->size);
            if (field->is_array) {
                int32_t index = (int32_t)v - field->logical_min;
                if (v == 0 || index < 0) {
                    continue; // empty slot
                }
                if (field->usage_count > 0) {
                    if (index < field->usage_count) {
                        emit(ctx, field->usages[index]);
                    }
                } else {
                    emit(ctx, field->usage_min + index);
                }
            } else if (v != 0) {
                if (field->usage_count > 0) {
                    // Extra elements reuse the last listed usage (HID 1.11 6.2.2.8).
                    uint32_t u = e < field->usage_count ? e : field->usage_count - 1u;
                    emit(ctx, field->usages[u]);
                } else {
                    emit(ctx, field->usage_min + e);
                }
            }
        }
    }
}

typedef struct {
    uint8_t *out;
    int keys;
    bool overflow;
} boot_builder_t;

static void add_key_usage(void *ctx, uint32_t usage)
{
    boot_builder_t *b = ctx;
    if (usage >= USAGE_LEFT_CTRL && usage <= USAGE_RIGHT_GUI) {
        b->out[0] |= 1u << (usage - USAGE_LEFT_CTRL);
    } else if (usage >= 0x01 && usage < USAGE_FIRST_KEY) {
        b->overflow = true; // keyboard reported its own rollover/error
    } else if (usage >= USAGE_FIRST_KEY && usage <= 0xFF) {
        if (b->keys < 6) {
            b->out[2 + b->keys++] = usage;
        } else {
            b->overflow = true;
        }
    }
}

void hid_kbd_translate(const hid_layout_t *layout, const uint8_t *data, size_t len,
                       uint8_t out[8])
{
    memset(out, 0, 8);
    boot_builder_t b = {.out = out};
    for_each_active_usage(layout, data, len, add_key_usage, &b);
    if (b.overflow) {
        memset(&out[2], 0x01, 6); // ErrorRollOver, modifiers still reported
    }
}

typedef struct {
    uint16_t *out;
    int count;
} consumer_builder_t;

static void add_consumer_usage(void *ctx, uint32_t usage)
{
    consumer_builder_t *b = ctx;
    if (usage == 0 || usage > 0xFFFF || b->count == HID_CONSUMER_MAX_USAGES) {
        return;
    }
    for (int i = 0; i < b->count; i++) {
        if (b->out[i] == usage) {
            return;
        }
    }
    b->out[b->count++] = usage;
}

void hid_consumer_translate(const hid_layout_t *layout, const uint8_t *data, size_t len,
                            uint16_t out[HID_CONSUMER_MAX_USAGES])
{
    memset(out, 0, HID_CONSUMER_MAX_USAGES * sizeof(uint16_t));
    consumer_builder_t b = {.out = out};
    for_each_active_usage(layout, data, len, add_consumer_usage, &b);
}
