#ifndef TURBIDITY_SENSOR_H
#define TURBIDITY_SENSOR_H

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <stdint.h>
#include <stddef.h>

uint16_t modbus_crc16(const uint8_t *buf, uint16_t len);
void send_modbus_read(uint8_t *tx_buf, size_t *tx_len);
bool turbidity_read_from_uart(const struct device *uart_dev, const struct device *gpio_dev, uint8_t de_pin, float *turbidity);

void build_set_baud9600(uint8_t *tx_buf, size_t *tx_len);
void turbidity_set_baud_9600(const struct device *uart_dev, const struct device *gpio_dev, uint8_t de_pin);

#endif // TURBIDITY_SENSOR_H
