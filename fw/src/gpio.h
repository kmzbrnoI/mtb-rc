/* Low-level GPIO functions, pin definitions. */

#pragma once

#include <stdbool.h>
#include "stm32f1xx_hal.h"

#define MTBBUS_ADDR_INPUTS 8

typedef struct {
    GPIO_TypeDef* port;
    uint32_t pin;
} PinDef;

static inline bool pindef_eq(PinDef a, PinDef b) {
    return (a.port == b.port) && (a.pin == b.pin);
}

extern const PinDef pin_led_red;
extern const PinDef pin_led_green;
extern const PinDef pin_led_blue;
extern const PinDef pin_btn;

extern const PinDef pins_addr[MTBBUS_ADDR_INPUTS];

extern const PinDef pin_dcc;
extern const PinDef pin_cutout;
extern const PinDef pin_rc1;
extern const PinDef pin_rc2;
extern const PinDef pin_mult1;
extern const PinDef pin_mult2;

extern const PinDef pin_mtbbus_tx;
extern const PinDef pin_mtbbus_rx;
extern const PinDef pin_mtbbus_te;

extern const PinDef pin_debug_1;
extern const PinDef pin_debug_2;


void gpio_init(void);

void gpio_pin_init(PinDef pin, uint32_t mode, uint32_t pull, uint32_t speed, bool de_init_first);
void gpio_pin_deinit(PinDef pin);
bool gpio_pin_read(PinDef pin);
void gpio_pin_write(PinDef pin, bool value);
void gpio_pin_toggle(PinDef pin);

// events
void gpio_on_cutout_change(void);
void gpio_on_dcc_fall(void);

void gpio_dcc_enable_fall_irq(void);
void gpio_dcc_disable_fall_irq(void);

/* Higher-level functions ----------------------------------------------------*/

static inline void gpio_uart_out(void) { gpio_pin_write(pin_mtbbus_te, true); }
static inline void gpio_uart_in(void) { gpio_pin_write(pin_mtbbus_te, false); }
uint8_t gpio_mtbbus_addr(void);
