#include "oxygen_sensor.h"
#include <string.h>
#include <stdlib.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

static bool parse_measurement(const char *buf, float *conc, float *sat, float *temp) {
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

bool oxygen_read_from_uart(const struct device *uart_dev, float *conc, float *sat, float *temp) {
    static char buf[256];
    static int idx = 0;
    unsigned char c;

    // This while is only needed if you want to flush the whole UART buffer, else just poll once per main loop!
    while (uart_poll_in(uart_dev, &c) == 0) {
        if (c == '\n' || c == '\r') {
            if (idx > 0) {
                buf[idx] = '\0';
                idx = 0;
                if (strstr(buf, "MEASUR")) {
                    return parse_measurement(buf, conc, sat, temp);
                }
            }
        } else if (c >= 32 && c <= 126) {
            if (idx < sizeof(buf) - 1) buf[idx++] = c;
        } else if (c == '\t' && idx < sizeof(buf) - 1) {
            buf[idx++] = ' ';
        }
    }
    return false;
}