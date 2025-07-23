#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <nrfx_saadc.h>

#include "oxygen_sensor.h"
#include "turbidity_sensor.h"

#include "fonts.h"
#include "lcd.h"

#include <zephyr/types.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#define S0_PIN 11 // Selector pin (P0.11 -> 74HC4052 S0)
#define TURB_DE_PIN 25 // MAX3485 DE (P0.25)
#define BUZZER_PIN 24


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

#define ADC_NODE        DT_NODELABEL(adc)
#define ADC_CHANNEL_ID  0
#define ADC_RESOLUTION  10
#define ADC_GAIN        ADC_GAIN_1_6
#define ADC_REFERENCE   ADC_REF_INTERNAL
#define ADC_BUFFER_SIZE 1

static int16_t adc_sample_buffer[ADC_BUFFER_SIZE];

static const struct adc_channel_cfg my_channel_cfg = {
    .gain             = ADC_GAIN_1_6,
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = ADC_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
    .input_positive   = NRF_SAADC_INPUT_AIN0,
#endif
};

float read_battery_voltage(const struct device *adc_dev)
{
    struct adc_sequence sequence = {
        .channels    = BIT(ADC_CHANNEL_ID),
        .buffer      = adc_sample_buffer,
        .buffer_size = sizeof(adc_sample_buffer),
        .resolution  = ADC_RESOLUTION,
    };

    int ret = adc_read(adc_dev, &sequence);
    if (ret) {
        printk("ADC read err %d\n", ret);
        return -1.0f;
    }

    int16_t raw = adc_sample_buffer[0];
    float v_ain = ((float)raw / 1023.0f) * 3.6f;
    float v_bat = v_ain * ((30.0f + 100.0f) / 100.0f);
    return v_bat;
}

static bool notify_enabled = false;
static struct bt_uuid_128 wq_service_uuid   = BT_UUID_INIT_128(BT_UUID_WQ_SERVICE_VAL);
static struct bt_uuid_128 wq_sensor_uuid    = BT_UUID_INIT_128(BT_UUID_WQ_SENSOR_VAL);
static struct bt_uuid_128 wq_o2_thresh_uuid = BT_UUID_INIT_128(BT_UUID_WQ_O2_THRESH_VAL);
static struct bt_uuid_128 wq_sat_thresh_uuid= BT_UUID_INIT_128(BT_UUID_WQ_SAT_THRESH_VAL);
static struct bt_uuid_128 wq_temp_thresh_uuid=BT_UUID_INIT_128(BT_UUID_WQ_TEMP_THRESH_VAL);

// Data [O2, Sat, Temp, Battery %]
static float sensor_data[4] = {0};
static float o2_thresh = 190.0;
static float sat_thresh = 80.0;
static float temp_thresh = 25.0;

// CCCD for notifications
static void sensor_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("Notify enabled: %d\n", notify_enabled);
}

// BLE read/write handlers
static ssize_t read_o2_thresh(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{ return bt_gatt_attr_read(conn, attr, buf, len, offset, &o2_thresh, sizeof(o2_thresh)); }
static ssize_t write_o2_thresh(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset)
{
    if (len != sizeof(o2_thresh)) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    memcpy(&o2_thresh, buf, sizeof(o2_thresh));
    printk("BLE: O2 Threshold updated to %.2f uM\n", o2_thresh);
    return len;
}
static ssize_t read_sat_thresh(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{ return bt_gatt_attr_read(conn, attr, buf, len, offset, &sat_thresh, sizeof(sat_thresh)); }
static ssize_t write_sat_thresh(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset)
{
    if (len != sizeof(sat_thresh)) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    memcpy(&sat_thresh, buf, sizeof(sat_thresh));
    printk("BLE: Saturation Threshold updated to %.2f %%\n", sat_thresh);
    return len;
}
static ssize_t read_temp_thresh(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{ return bt_gatt_attr_read(conn, attr, buf, len, offset, &temp_thresh, sizeof(temp_thresh)); }
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
    return bt_gatt_attr_read(conn, attr, buf, len, offset, sensor_data, sizeof(sensor_data));
}

// GATT Table
BT_GATT_SERVICE_DEFINE(wq_svc,
    BT_GATT_PRIMARY_SERVICE(&wq_service_uuid),
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

void uart_flush(const struct device *uart_dev) {
    unsigned char c;
    while (uart_poll_in(uart_dev, &c) == 0) {
        // discard
    }
}

void main(void) {
    const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));
    const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(uart_dev) || !device_is_ready(gpio_dev)) {
        printk("UART0 or GPIO not ready!\n");
        return;
    }

    const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);
    if (!device_is_ready(adc_dev)) {
        printk("ADC device not ready!\n");
        return;
    }

    adc_channel_setup(adc_dev, &my_channel_cfg);

    gpio_pin_configure(gpio_dev, S0_PIN, GPIO_OUTPUT_ACTIVE); //Sets to 1
    gpio_pin_configure(gpio_dev, TURB_DE_PIN, GPIO_OUTPUT_INACTIVE); //Sets to 0
    gpio_pin_configure(gpio_dev, BUZZER_PIN, GPIO_OUTPUT_INACTIVE);//Sets to 0


    lcd_init(gpio_dev);
    lcd_clear(gpio_dev);

    int err;
    printk("Starting Oxygen2 BLE example...\n");
    err = bt_enable(NULL);
    printk("bt_enable returned %d\n", err);
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

    float conc = 0, sat = 0, temp = 0;
    gpio_pin_set(gpio_dev, S0_PIN, 1); //For Selecting Oxygen Sensor
    
    //float turbidity = 0;
    //gpio_pin_set(gpio_dev, S0_PIN, 0); //For Selecting Turbidity Sensor


    while (1) {
        /*
        turbidity_read_from_uart(uart_dev, gpio_dev, TURB_DE_PIN, &turbidity);
        printk("Turbidity: %d.%01d NTU\n",
            (int)turbidity, ((int)(turbidity * 10)) % 10
        );
        char line4[32];
        snprintf(line4, sizeof(line4), "Turb: %d.%01d NTU", (int)turbidity, ((int)(turbidity*10))%10);
        lcd_clear(gpio_dev);
        lcd_draw_text(gpio_dev, 0, 0, line4);
        */


        
        float vbat = read_battery_voltage(adc_dev);
        int battery_percent = (int)(((vbat - 3.0) / (4.2 - 3.0)) * 100);
        if (battery_percent > 100) battery_percent = 100;
        if (battery_percent < 0) battery_percent = 0;

        if (oxygen_read_from_uart(uart_dev, &conc, &sat, &temp)) {
            printk("PARSED: O2=%d.%02d uM  Sat=%d.%02d%%  T=%d.%02d C Battery=%d%%\n",
                (int)conc,   ((int)(conc * 100)) % 100,
                (int)sat,    ((int)(sat  * 100)) % 100,
                (int)temp,   ((int)(temp * 100)) % 100,
                battery_percent
            );
            
            sensor_data[0] = conc;
            sensor_data[1] = sat;
            sensor_data[2] = temp;
            sensor_data[3] = (float)battery_percent;
            
            // --- Temperature alert logic ---
            if (conc < o2_thresh) {
                gpio_pin_set(gpio_dev, BUZZER_PIN, 1); // Buzzer ON
                printk("ALERT: Oxygen below threshold! Buzzer ON\n");
            } else {
                gpio_pin_set(gpio_dev, BUZZER_PIN, 0); // Buzzer OFF
            }

            // Print and update
            char line1[32], line2[32], line3[32], line5[32];
            snprintf(line1, sizeof(line1), "O2: %d.%02d uM", (int)conc, ((int)(conc*100))%100);
            snprintf(line2, sizeof(line2), "Sat: %d.%02d%%", (int)sat, ((int)(sat*100))%100);
            snprintf(line3, sizeof(line3), "T: %d.%02d C", (int)temp, ((int)(temp*100))%100);
            snprintf(line5, sizeof(line5), "Battery: %d%%", battery_percent);

            lcd_clear(gpio_dev);
            lcd_draw_text(gpio_dev, 0, 0, line1);
            lcd_draw_text(gpio_dev, 1, 0, line2);
            lcd_draw_text(gpio_dev, 2, 0, line3);
            lcd_draw_text(gpio_dev, 3, 0, line5);

            if (notify_enabled) {
                int err1 = bt_gatt_notify(NULL, &wq_svc.attrs[1], sensor_data, sizeof(sensor_data));
                if (err1) {
                    printk("bt_gatt_notify(sensor_data) failed: %d\n", err1);
                }
            }
        }
       
        k_msleep(1); //1 seconds for Oxygen Sensor
        //k_msleep(20000); //20 seconds for Turbidity Sensor
    }
}
