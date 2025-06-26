#ifndef LCD_H_
#define LCD_H_

#include <zephyr/device.h>     // This fixes struct device visibility!
#include <zephyr/drivers/gpio.h> // This gets GPIO_OUTPUT_LOW and friends
#include <stdint.h>          // For uint8_t etc.
#include <stdbool.h>         // For bool type


void lcd_init(const struct device *gpio_dev);
void lcd_clear(const struct device *gpio_dev);
void lcd_set_page(const struct device *gpio_dev, uint8_t page);
void lcd_set_column(const struct device *gpio_dev, uint8_t col);
void lcd_draw_char(const struct device *gpio_dev, uint8_t page, uint8_t col, char c);
void lcd_draw_text(const struct device *gpio_dev, uint8_t page, uint8_t col, const char *str);

#endif // LCD_H_
