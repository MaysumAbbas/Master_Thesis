#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <string.h>
#include "lcd.h"
#include "fonts.h"

// LCD Pin Definitions (update as needed)
#define LCD_CS_PIN    3   // Chip Select
#define LCD_RES_PIN   4   // Reset
#define LCD_A0_PIN    28  // Data/Command
#define LCD_SCL_PIN   29  // Clock
#define LCD_SI_PIN    30  // Data In


// Bit-banged SPI send (command/data)
static void spi_send(const struct device *gpio_dev, uint8_t byte, bool is_data) {
    gpio_pin_set(gpio_dev, LCD_CS_PIN, 0);
    gpio_pin_set(gpio_dev, LCD_A0_PIN, is_data ? 1 : 0);

    for (int bit = 7; bit >= 0; bit--) {
        gpio_pin_set(gpio_dev, LCD_SI_PIN, (byte >> bit) & 1);
        gpio_pin_set(gpio_dev, LCD_SCL_PIN, 1);
        k_busy_wait(1);
        gpio_pin_set(gpio_dev, LCD_SCL_PIN, 0);
    }
    gpio_pin_set(gpio_dev, LCD_CS_PIN, 1);
}

// Helper functions
static void lcd_write_command(const struct device *gpio_dev, uint8_t cmd) {
    spi_send(gpio_dev, cmd, false);
}

static void lcd_write_data(const struct device *gpio_dev, uint8_t data) {
    spi_send(gpio_dev, data, true);
}

// Initialization (from your previous working code)
void lcd_init(const struct device *gpio_dev) {
    // Setup all pins as output and LOW
    gpio_pin_configure(gpio_dev, LCD_CS_PIN,  GPIO_OUTPUT_LOW);
    gpio_pin_configure(gpio_dev, LCD_RES_PIN, GPIO_OUTPUT_LOW);
    gpio_pin_configure(gpio_dev, LCD_A0_PIN,  GPIO_OUTPUT_LOW);
    gpio_pin_configure(gpio_dev, LCD_SCL_PIN, GPIO_OUTPUT_LOW);
    gpio_pin_configure(gpio_dev, LCD_SI_PIN,  GPIO_OUTPUT_LOW);

    // Reset sequence
    gpio_pin_set(gpio_dev, LCD_RES_PIN, 0);
    k_msleep(5);
    gpio_pin_set(gpio_dev, LCD_RES_PIN, 1);
    k_msleep(5);

    // Send LCD init commands (from your working code)
    lcd_write_command(gpio_dev, 0xA2);
    lcd_write_command(gpio_dev, 0xA0);
    lcd_write_command(gpio_dev, 0xC8);
    lcd_write_command(gpio_dev, 0x40);
    lcd_write_command(gpio_dev, 0x25);
    lcd_write_command(gpio_dev, 0x81);
    lcd_write_command(gpio_dev, 0x19);
    lcd_write_command(gpio_dev, 0x2F);
    lcd_write_command(gpio_dev, 0xAF);
}

// Set current page (0-7)
void lcd_set_page(const struct device *gpio_dev, uint8_t page) {
    lcd_write_command(gpio_dev, 0xB0 | (page & 0x07));
}

// Set current column (0-127)
void lcd_set_column(const struct device *gpio_dev, uint8_t col) {
    lcd_write_command(gpio_dev, 0x10 | ((col >> 4) & 0x0F));
    lcd_write_command(gpio_dev, 0x00 | (col & 0x0F));
}

// Clear display (set all pixels off)
void lcd_clear(const struct device *gpio_dev) {
    for (uint8_t page = 0; page < 8; page++) {
        lcd_set_page(gpio_dev, page);
        lcd_set_column(gpio_dev, 0);
        for (uint8_t col = 0; col < 128; col++) {
            lcd_write_data(gpio_dev, 0x00);
        }
    }
}

// Draw a single char from font table at page, column
void lcd_draw_char(const struct device *gpio_dev, uint8_t page, uint8_t col, char c) {
    lcd_set_page(gpio_dev, page);
    lcd_set_column(gpio_dev, col);

    // Handle font lookup (assume font8, adjust as needed)
    uint8_t idx = (uint8_t)c;
    if (idx < 32 || idx > 126) idx = 32; // blank for unprintables

    for (int i = 0; i < 8; i++) {
        lcd_write_data(gpio_dev, font8[idx - 32][i]);
    }
}

// Draw string at page, column (chars will wrap page if needed)
void lcd_draw_text(const struct device *gpio_dev, uint8_t page, uint8_t col, const char *str) {
    while (*str && col < 128) {
        lcd_draw_char(gpio_dev, page, col, *str++);
        col += 8; // for 8x8 font, adjust for 5x7, etc.
    }
}
