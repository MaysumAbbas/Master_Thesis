#ifndef OXYGEN_SENSOR_H
#define OXYGEN_SENSOR_H

#include <zephyr/device.h>
#include <stdbool.h>

bool oxygen_read_from_uart(const struct device *uart_dev, float *conc, float *sat, float *temp);

#endif
