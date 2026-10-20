/*
 * Vendor-defined USB HID interface reporting peripheral battery and active layer.
 *
 * Report 0x01: { percent[source] ... } one byte per peripheral, 0xFF = unknown
 * Report 0x02: { highest layer index, layer state bitmask (u32 LE), layer name (NUL-padded) }
 * Report 0x03: { selected transport, preferred transport, BLE profile (0xFF = n/a) }
 *              transport: 0 = none, 1 = USB, 2 = BLE
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/endpoints.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/keymap.h>
#include <zmk/usb.h>

LOG_MODULE_REGISTER(usb_hid_status, CONFIG_ZMK_LOG_LEVEL);

#define REPORT_ID_BATTERY 0x01
#define REPORT_ID_LAYER 0x02
#define REPORT_ID_OUTPUT 0x03

#define MAX_SOURCES CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS
#define LAYER_NAME_LEN 16

static const uint8_t report_desc[] = {
    0x06, 0x42, 0xFF, // Usage Page (Vendor 0xFF42)
    0x09, 0x01,       // Usage (1)
    0xA1, 0x01,       // Collection (Application)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08,       //   Report Size (8)

    0x85, REPORT_ID_BATTERY, //   Report ID
    0x09, 0x02,              //   Usage (2)
    0x95, MAX_SOURCES,       //   Report Count (one byte per peripheral)
    0x81, 0x02,              //   Input (Data, Var, Abs)

    0x85, REPORT_ID_LAYER, //   Report ID
    0x09, 0x03,            //   Usage (3)
    0x95, 5 + LAYER_NAME_LEN, //   Report Count (index, state u32, name)
    0x81, 0x02,            //   Input (Data, Var, Abs)

    0x85, REPORT_ID_OUTPUT, //   Report ID
    0x09, 0x04,             //   Usage (4)
    0x95, 0x03,             //   Report Count (selected, preferred, BLE profile)
    0x81, 0x02,             //   Input (Data, Var, Abs)
    0xC0,                   // End Collection
};

#define BATTERY_UNKNOWN 0xFF

struct battery_report {
    uint8_t id;
    uint8_t percent[MAX_SOURCES];
} __packed;

struct layer_report {
    uint8_t id;
    uint8_t highest;
    uint32_t state;
    char name[LAYER_NAME_LEN];
} __packed;

struct output_report {
    uint8_t id;
    uint8_t selected;
    uint8_t preferred;
    uint8_t ble_profile;
} __packed;

union status_report {
    uint8_t id;
    struct output_report output;
    struct battery_report battery;
    struct layer_report layer;
};

static const struct device *hid_dev;
static bool hid_ready;

static K_SEM_DEFINE(hid_sem, 1, 1);
K_MSGQ_DEFINE(report_q, sizeof(union status_report), 16, 1);

static struct battery_report battery_cache = {.id = REPORT_ID_BATTERY};

static void send_work_cb(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(send_work, send_work_cb);

static size_t report_len(const union status_report *r) {
    switch (r->id) {
    case REPORT_ID_BATTERY:
        return sizeof(struct battery_report);
    case REPORT_ID_OUTPUT:
        return sizeof(struct output_report);
    default:
        return sizeof(struct layer_report);
    }
}

static void make_battery_report(union status_report *r) {
    r->battery = battery_cache;
}

static void make_output_report(union status_report *r) {
    struct zmk_endpoint_instance preferred = zmk_endpoint_get_preferred();

    r->output.id = REPORT_ID_OUTPUT;
    r->output.selected = (uint8_t)zmk_endpoint_get_selected().transport;
    r->output.preferred = (uint8_t)preferred.transport;
    r->output.ble_profile =
        preferred.transport == ZMK_TRANSPORT_BLE ? (uint8_t)preferred.ble.profile_index : 0xFF;
}

static void make_layer_report(union status_report *r) {
    r->layer.id = REPORT_ID_LAYER;
    r->layer.highest = zmk_keymap_highest_layer_active();
    r->layer.state = (uint32_t)zmk_keymap_layer_state();

    const char *name = zmk_keymap_layer_name(zmk_keymap_layer_index_to_id(r->layer.highest));
    memset(r->layer.name, 0, sizeof(r->layer.name));
    if (name != NULL) {
        strncpy(r->layer.name, name, sizeof(r->layer.name) - 1);
    }
}

static void queue_report(const union status_report *r) {
    if (k_msgq_put(&report_q, r, K_NO_WAIT) != 0) {
        // Newest state wins: drop the oldest, retry once.
        union status_report old;
        k_msgq_get(&report_q, &old, K_NO_WAIT);
        k_msgq_put(&report_q, r, K_NO_WAIT);
    }
    k_work_schedule(&send_work, K_NO_WAIT);
}

static void send_work_cb(struct k_work *work) {
    ARG_UNUSED(work);
    union status_report r;

    while (k_msgq_peek(&report_q, &r) == 0) {
        if (!hid_ready || zmk_usb_get_conn_state() != ZMK_USB_CONN_HID) {
            k_msgq_purge(&report_q);
            return;
        }

        if (k_sem_take(&hid_sem, K_NO_WAIT) != 0) {
            // Endpoint busy, try again shortly.
            k_work_schedule(&send_work, K_MSEC(5));
            return;
        }

        k_msgq_get(&report_q, &r, K_NO_WAIT);

        int ret = hid_int_ep_write(hid_dev, (uint8_t *)&r, report_len(&r), NULL);
        if (ret < 0) {
            LOG_WRN("HID status write failed: %d", ret);
            k_sem_give(&hid_sem);
        }
    }
}

static void int_in_ready_cb(const struct device *dev) {
    ARG_UNUSED(dev);
    k_sem_give(&hid_sem);
}

static int get_report_cb(const struct device *dev, struct usb_setup_packet *setup, int32_t *len,
                         uint8_t **data) {
    static union status_report snapshot;
    uint8_t type = (setup->wValue >> 8) & 0xFF;
    uint8_t id = setup->wValue & 0xFF;

    if (type != 0x01) { // Input
        return -ENOTSUP;
    }

    switch (id) {
    case REPORT_ID_LAYER:
        make_layer_report(&snapshot);
        break;
    case REPORT_ID_BATTERY:
        make_battery_report(&snapshot);
        break;
    case REPORT_ID_OUTPUT:
        make_output_report(&snapshot);
        break;
    default:
        return -ENOTSUP;
    }

    *data = (uint8_t *)&snapshot;
    *len = report_len(&snapshot);
    return 0;
}

static const struct hid_ops ops = {
    .int_in_ready = int_in_ready_cb,
    .get_report = get_report_cb,
};

static void send_full_snapshot(void) {
    union status_report r;

    make_battery_report(&r);
    queue_report(&r);

    make_layer_report(&r);
    queue_report(&r);

    make_output_report(&r);
    queue_report(&r);
}

static int status_listener(const zmk_event_t *eh) {
    const struct zmk_peripheral_battery_state_changed *bat =
        as_zmk_peripheral_battery_state_changed(eh);
    if (bat != NULL) {
        if (bat->source < MAX_SOURCES) {
            battery_cache.percent[bat->source] = bat->state_of_charge;
        }
        union status_report r;
        make_battery_report(&r);
        queue_report(&r);
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (as_zmk_layer_state_changed(eh) != NULL) {
        union status_report r;
        make_layer_report(&r);
        queue_report(&r);
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (as_zmk_endpoint_changed(eh) != NULL) {
        union status_report r;
        make_output_report(&r);
        queue_report(&r);
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_usb_conn_state_changed *usb = as_zmk_usb_conn_state_changed(eh);
    if (usb != NULL && usb->conn_state == ZMK_USB_CONN_HID) {
        send_full_snapshot();
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(usb_hid_status, status_listener);
ZMK_SUBSCRIPTION(usb_hid_status, zmk_peripheral_battery_state_changed);
ZMK_SUBSCRIPTION(usb_hid_status, zmk_layer_state_changed);
ZMK_SUBSCRIPTION(usb_hid_status, zmk_endpoint_changed);
ZMK_SUBSCRIPTION(usb_hid_status, zmk_usb_conn_state_changed);

static int usb_hid_status_init(void) {
    memset(battery_cache.percent, BATTERY_UNKNOWN, sizeof(battery_cache.percent));

    hid_dev = device_get_binding("HID_1");
    if (hid_dev == NULL) {
        LOG_ERR("HID_1 not found, is USB_HID_DEVICE_COUNT >= 2?");
        return -ENODEV;
    }

    usb_hid_register_device(hid_dev, report_desc, sizeof(report_desc), &ops);

    int ret = usb_hid_init(hid_dev);
    if (ret != 0) {
        LOG_ERR("usb_hid_init failed: %d", ret);
        return ret;
    }

    hid_ready = true;
    return 0;
}

SYS_INIT(usb_hid_status_init, APPLICATION, CONFIG_ZMK_USB_HID_INIT_PRIORITY);
