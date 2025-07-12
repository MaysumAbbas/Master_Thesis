#include "turbidity_sensor.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <string.h>

uint16_t modbus_crc16(const uint8_t *buf, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else
                crc >>= 1;
        }
    }
    return crc;
}

void send_modbus_read(uint8_t *tx_buf, size_t *tx_len) {
    tx_buf[0] = 0x01;      // address (change if you set sensor differently)
    tx_buf[1] = 0x03;      // function: read holding registers
    tx_buf[2] = 0x00;      // reg high
    tx_buf[3] = 0x00;      // reg low (start at 0x0000)
    tx_buf[4] = 0x00;      // number of regs high
    tx_buf[5] = 0x02;      // number of regs low (read 2 regs)
    uint16_t crc = modbus_crc16(tx_buf, 6);
    tx_buf[6] = crc & 0xFF;
    tx_buf[7] = (crc >> 8) & 0xFF;
    *tx_len = 8;
}

bool read_turbidity_sensor(const struct device *uart_dev, const struct device *gpio_dev, int rs485_de_pin, float *turb, float *temp) {
    uint8_t tx_buf[8];
    uint8_t rx_buf[16];
    size_t tx_len = 0, rx_len = 0;

    send_modbus_read(tx_buf, &tx_len);

    // Set DE high (transmit)
    gpio_pin_set(gpio_dev, rs485_de_pin, 1);
    k_sleep(K_MSEC(10)); // Allow line to settle

    // Send the query
    for (size_t i = 0; i < tx_len; i++)
        uart_poll_out(uart_dev, tx_buf[i]);
    k_sleep(K_MSEC(2)); // Wait for last byte

    // Set DE low (receive)
    gpio_pin_set(gpio_dev, rs485_de_pin, 0);

    // Wait for response
    int timeout = 200; // ms
    int t = 0;
    rx_len = 0;
    while (t < timeout && rx_len < sizeof(rx_buf)) {
        uint8_t c;
        if (uart_poll_in(uart_dev, &c) == 0) {
            rx_buf[rx_len++] = c;
        } else {
            k_sleep(K_MSEC(2));
            t += 2;
        }
    }

    // Parse response (address, func, byte count, data[4], CRC[2])
    if (rx_len >= 9 && rx_buf[1] == 0x03 && rx_buf[2] == 0x04) {
        int turb_raw = (rx_buf[3] << 8) | rx_buf[4];
        int temp_raw = (rx_buf[5] << 8) | rx_buf[6];
        *turb = turb_raw / 10.0f;
        *temp = temp_raw / 10.0f;
        return true;
    } else {
        *turb = 0;
        *temp = 0;
        return false;
    }
}

void build_set_baud9600(uint8_t *tx_buf, size_t *tx_len) {
    tx_buf[0] = 0x01;      // Sensor address (change if needed)
    tx_buf[1] = 0x06;      // Function: Write single register
    tx_buf[2] = 0x07;      // Register high byte (0x07D1)
    tx_buf[3] = 0xD1;      // Register low byte
    tx_buf[4] = 0x00;      // Value high byte
    tx_buf[5] = 0x02;      // Value low byte (2 = 9600)
    uint16_t crc = modbus_crc16(tx_buf, 6);
    tx_buf[6] = crc & 0xFF;
    tx_buf[7] = (crc >> 8) & 0xFF;
    *tx_len = 8;
}

void turbidity_set_baud_9600(const struct device *uart_dev, const struct device *gpio_dev, uint8_t de_pin) {
    uint8_t tx_buf[8];
    uint8_t rx_buf[16];
    size_t tx_len, rx_len;

    build_set_baud9600(tx_buf, &tx_len);

    gpio_pin_set(gpio_dev, de_pin, 1);
    k_sleep(K_MSEC(10));
    for (size_t i = 0; i < tx_len; i++)
        uart_poll_out(uart_dev, tx_buf[i]);
    k_sleep(K_MSEC(2));
    gpio_pin_set(gpio_dev, de_pin, 0);

    rx_len = 0;
    int timeout = 200, t = 0;
    while (t < timeout && rx_len < sizeof(rx_buf)) {
        uint8_t c;
        if (uart_poll_in(uart_dev, &c) == 0) {
            rx_buf[rx_len++] = c;
        } else {
            k_sleep(K_MSEC(2));
            t += 2;
        }
    }
    printk("Sent frame to set baud to 9600.\n");
    printk("Raw bytes received (%d): ", rx_len);
    for (int i = 0; i < rx_len; i++) {
        printk("%02X ", rx_buf[i]);
    }
    printk("\n");
    if (rx_len == 8 && memcmp(rx_buf, tx_buf, 8) == 0) {
        printk("Success! Sensor replied with mirror frame. Baud rate now set to 9600.\n");
    } else {
        printk("Warning: Unexpected reply. Check connection and retry if needed.\n");
    }
    printk("** Now power-cycle or reset the sensor, and update your UART overlay and code to 9600. **\n");
    while (1) { k_sleep(K_SECONDS(1)); }
}
