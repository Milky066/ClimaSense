#include "ssd1283a.h"

static uint8_t _cs, _rst, _cd, _led;

void lcd_initilize(uint8_t cs_pin, uint8_t rst_pin, uint8_t cd_pin, uint8_t led_pin){
    _cs = cs_pin;
    _rst = rst_pin;
    _cd = cd_pin;
    _led = led_pin;
}