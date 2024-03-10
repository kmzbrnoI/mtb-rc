/* Inputs debouncing */

#pragma once

#include <stdint.h>
#include <stdbool.h>

extern bool btn_pressed;
void btn_on_pressed(void);
void btn_on_released(void);

// This function should be called each 500 us
void btn_debounce_update(void);
