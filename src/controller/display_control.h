#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void cv_gate_init(void);
void display_update(const uint8_t* buffer, uint16_t size);
void display_clear(void);
bool display_has_oled(void);

void pad_led_set_rgb(uint8_t col, uint8_t row, uint8_t r, uint8_t g, uint8_t b);
void pad_led_flush_dirty(void);
void pad_led_set_all(const uint8_t* data, uint16_t len);
void pad_led_clear_all(void);

void hardware_led_set(uint8_t led_index, bool on);

void cv_set(uint8_t channel, uint16_t value);
void gate_set(uint8_t channel, bool on);

#ifdef __cplusplus
}
#endif
