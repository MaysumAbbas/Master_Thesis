#include "oxygen_sensor.h"
#include <string.h>
#include <stdlib.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

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

bool get_oxygen_reading(const struct device *uart_dev, float *conc, float *sat, float *temp) {
    char buf[256];
    int idx = 0;
    char c;
    int timeout = 21000; // ms
    int elapsed = 0;
    printk("Waiting for O2 sensor...\n");
    while (elapsed < timeout) {
        if (uart_poll_in(uart_dev, &c) == 0) {
            // Buffer the incoming char
            if (c == '\n' || c == '\r') {
                if (idx > 0) {
                    buf[idx] = '\0';
                    printk("UART LINE: %s\n", buf);  // <--- Print the whole line here!
                    if (strstr(buf, "MEASUR")) {
                        printk("Found MEASUR line, trying to parse...\n");
                        if (parse_measurement(buf, conc, sat, temp)) {
                            printk("PARSE SUCCESS: %.2f, %.2f, %.2f\n", *conc, *sat, *temp);
                            return true;
                        } else {
                            printk("MEASUR line could not be parsed\n");
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
        k_msleep(2);
        elapsed += 2;
    }
    printk("Timeout waiting for oxygen sensor data\n");
    return false;
}

