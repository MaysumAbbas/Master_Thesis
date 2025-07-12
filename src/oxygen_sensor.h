#ifndef OXYGEN_SENSOR_H
#define OXYGEN_SENSOR_H

#include <zephyr/device.h>
#include <stdbool.h>

bool parse_measurement(const char *buf, float *conc, float *sat, float *temp);
bool get_oxygen_reading(const struct device *uart_dev, float *conc, float *sat, float *temp);

#endif // OXYGEN_SENSOR_H
