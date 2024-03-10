/* Inputs handling (debouncing) */

#include "inputs.h"
#include "gpio.h"

bool btn_pressed = false;

#define DEBOUNCE_THRESHOLD 20 // 10 ms
uint8_t _btn_debounce_counter = 0;

void btn_debounce_update(void) {
    if (gpio_pin_read(pin_btn)) {
        if (_btn_debounce_counter > 0) {
            _btn_debounce_counter--;
            if ((_btn_debounce_counter) == 0 && (btn_pressed)) {
                btn_pressed = false;
                btn_on_released();
            }
        }
    } else {
        if (_btn_debounce_counter < DEBOUNCE_THRESHOLD) {
            _btn_debounce_counter++;
            if ((_btn_debounce_counter == DEBOUNCE_THRESHOLD) && (!btn_pressed)) {
                btn_pressed = true;
                btn_on_pressed();
            }
        }
    }
}
