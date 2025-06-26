#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// Add your LCD includes!
#include "fonts.h"
#include "lcd.h"

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
    // Initialize LCD (add your LCD init code)
    lcd_init(gpio_dev);
    lcd_clear(gpio_dev);

    char buf[256];
    int idx = 0;
    char c;
    float conc, sat, temp;

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
                            //snprintf(line1, sizeof(line1), "O2: %.2f uM", conc);
                            //snprintf(line2, sizeof(line2), "Sat: %.2f%%", sat);
                            //snprintf(line3, sizeof(line3), "T: %.2f C", temp);
                            
                            snprintf(line1, sizeof(line1), "O2: %d.%02d uM", (int)conc, ((int)(conc*100))%100);
                            snprintf(line2, sizeof(line2), "Sat: %d.%02d%%", (int)sat, ((int)(sat*100))%100);
                            snprintf(line3, sizeof(line3), "T: %d.%02d C", (int)temp, ((int)(temp*100))%100);


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
