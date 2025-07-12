#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

//For LCD
#include "fonts.h"
#include "lcd.h"

//For Wireless Communication
#include <zephyr/types.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

// UUIDs for your custom service/characteristics (replace with your own if you wish)
#define BT_UUID_WQ_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_WQ_SENSOR_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)
#define BT_UUID_WQ_O2_THRESH_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2)
#define BT_UUID_WQ_SAT_THRESH_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef3)
#define BT_UUID_WQ_TEMP_THRESH_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef4)

static bool notify_enabled = false;
static struct bt_uuid_128 wq_service_uuid   = BT_UUID_INIT_128(BT_UUID_WQ_SERVICE_VAL);
static struct bt_uuid_128 wq_sensor_uuid    = BT_UUID_INIT_128(BT_UUID_WQ_SENSOR_VAL);
static struct bt_uuid_128 wq_o2_thresh_uuid = BT_UUID_INIT_128(BT_UUID_WQ_O2_THRESH_VAL);
static struct bt_uuid_128 wq_sat_thresh_uuid= BT_UUID_INIT_128(BT_UUID_WQ_SAT_THRESH_VAL);
static struct bt_uuid_128 wq_temp_thresh_uuid=BT_UUID_INIT_128(BT_UUID_WQ_TEMP_THRESH_VAL);

// Data holders - REMEMBER these are in little-endian IEEE754 float format
static float sensor_data[3] = {0};   // [O2, Sat, Temp] // 12-byte float in Hex
static float o2_thresh = 50.0; // 4-byte float in Hex
static float sat_thresh = 80.0; // 4-byte float in Hex
static float temp_thresh = 25.0; // 4-byte float in Hex

// CCCD (for notifications)
static void sensor_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("Notify enabled: %d\n", notify_enabled);
}

// BLE read/write handlers for thresholds
static ssize_t read_o2_thresh(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &o2_thresh, sizeof(o2_thresh));
}
static ssize_t write_o2_thresh(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset)
{
    if (len != sizeof(o2_thresh)) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    memcpy(&o2_thresh, buf, sizeof(o2_thresh));
    printk("BLE: O2 Threshold updated to %.2f uM\n", o2_thresh);
    return len;
}


static ssize_t read_sat_thresh(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &sat_thresh, sizeof(sat_thresh));
}
static ssize_t write_sat_thresh(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset)
{
    if (len != sizeof(sat_thresh)) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    memcpy(&sat_thresh, buf, sizeof(sat_thresh));
    printk("BLE: Saturation Threshold updated to %.2f %%\n", sat_thresh);
    return len;
}

static ssize_t read_temp_thresh(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &temp_thresh, sizeof(temp_thresh));
}
static ssize_t write_temp_thresh(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset)
{
    if (len != sizeof(temp_thresh)) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    memcpy(&temp_thresh, buf, sizeof(temp_thresh));
    printk("BLE: Temperature Threshold updated to %.2f C\n", temp_thresh);
    return len;
}

// Custom Read Handler for Sensor Data
static ssize_t read_sensor_data(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                void *buf, uint16_t len, uint16_t offset)
{
    // sensor_data is a float[3] global
    return bt_gatt_attr_read(conn, attr, buf, len, offset, sensor_data, sizeof(sensor_data));
}


// GATT Table Defination
BT_GATT_SERVICE_DEFINE(wq_svc,
    BT_GATT_PRIMARY_SERVICE(&wq_service_uuid),
    
    
    //BT_GATT_CHARACTERISTIC(&wq_sensor_uuid.uuid,
      //  BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
       // BT_GATT_PERM_READ, 
        //bt_gatt_attr_read, NULL, sensor_data),

    BT_GATT_CHARACTERISTIC(&wq_sensor_uuid.uuid,
    BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
    BT_GATT_PERM_READ, 
    read_sensor_data, NULL, sensor_data),


    BT_GATT_CCC(sensor_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&wq_o2_thresh_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, 
        read_o2_thresh, write_o2_thresh, &o2_thresh),
    BT_GATT_CHARACTERISTIC(&wq_sat_thresh_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, 
        read_sat_thresh, write_sat_thresh, &sat_thresh),
    BT_GATT_CHARACTERISTIC(&wq_temp_thresh_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, 
        read_temp_thresh, write_temp_thresh, &temp_thresh),
);


// End of Wireless Communication //


bool parse_measurement(const char *buf, float *conc, float *sat, float *temp) {
    const char *cptr, *sptr, *tptr;
    cptr = strstr(buf, "[uM]");
    if (!cptr) return false;
    *conc = strtof(cptr + 4, NULL);

    sptr = strstr(buf, "[%]");
    if (!sptr) return false;
    *sat = strtof(sptr + 3, NULL);

    tptr = strstr(buf, "[Deg.C]");
    if (!tptr) return false;
    *temp = strtof(tptr + 7, NULL);

    return true;
}

void main(void) {
    const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));
    const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(uart_dev)) {
        printk("UART0 not ready!\n");
        return;
    }
    // Initialize LCD
    lcd_init(gpio_dev);
    lcd_clear(gpio_dev);

    char buf[256];
    int idx = 0;
    char c;
    float conc, sat, temp;

    // For Wireless Communication
    //
    int err;

    printk("Starting Oxygen2 BLE example...\n");

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }
    printk("Bluetooth initialized\n");

    struct bt_le_adv_param adv_params = {
        .options = BT_LE_ADV_OPT_CONNECTABLE,
        .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
        .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
    };

    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
    };

    err = bt_le_adv_start(&adv_params, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }
    printk("Advertising started\n");
    
    // End of Wireless

    while (1) {
        if (uart_poll_in(uart_dev, &c) == 0) {
            if (c == '\n' || c == '\r') {
                if (idx > 0) {
                    buf[idx] = '\0';

                    if (strstr(buf, "MEASUR")) {  // Only process real sensor data
                        //printk("RAW: %s\n", buf);

                        if (parse_measurement(buf, &conc, &sat, &temp)) {
                            //printk("PARSED: O2=%.2f uM  Sat=%.2f%%  T=%.2f C\n", conc, sat, temp);
                            
                            printk("PARSED: O2=%d.%02d uM  Sat=%d.%02d%%  T=%d.%02d C\n",
                                (int)conc,   ((int)(conc * 100)) % 100,
                                (int)sat,    ((int)(sat  * 100)) % 100,
                                (int)temp,   ((int)(temp * 100)) % 100
                            );
                            
                            // LCD display
                            char line1[32], line2[32], line3[32];
                            // snprintf(line1, sizeof(line1), "O2: %.2f uM", conc);
                            // snprintf(line2, sizeof(line2), "Sat: %.2f%%", sat);
                            // snprintf(line3, sizeof(line3), "T: %.2f C", temp);
                            
                            snprintf(line1, sizeof(line1), "O2: %d.%02d uM", (int)conc, ((int)(conc*100))%100);
                            snprintf(line2, sizeof(line2), "Sat: %d.%02d%%", (int)sat, ((int)(sat*100))%100);
                            snprintf(line3, sizeof(line3), "T: %d.%02d C", (int)temp, ((int)(temp*100))%100);
                            
                            //Updating Sensor Data for Bluetooth
                            sensor_data[0] = conc;
                            sensor_data[1] = sat;
                            sensor_data[2] = temp;

                            //sensor_data[0] = 123.45f;
                            //sensor_data[1] = 67.89f;
                            //sensor_data[2] = 20.12f;

                            // Send BLE notification if subscribed
                            //bt_gatt_notify(NULL, &wq_svc.attrs[1], sensor_data, sizeof(sensor_data));
                            
                            if (notify_enabled) {
                                int err = bt_gatt_notify(NULL, &wq_svc.attrs[1], sensor_data, sizeof(sensor_data));
                                if (err) {
                                    printk("bt_gatt_notify failed: %d\n", err);
                                }
                            }
                            
                            lcd_clear(gpio_dev);
                            lcd_draw_text(gpio_dev, 0, 0, line1);
                            lcd_draw_text(gpio_dev, 1, 0, line2);
                            lcd_draw_text(gpio_dev, 2, 0, line3);
                        }
                    }

                    idx = 0;
                }
            } else if (c >= 32 && c <= 126) {
                if (idx < sizeof(buf) - 1) buf[idx++] = c;
            } else if (c == '\t' && idx < sizeof(buf) - 1) {
                buf[idx++] = ' ';
            }
        }
        k_msleep(1);
    }
}