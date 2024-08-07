/* Low-level GPIO functions, pin definitions. */

#pragma once

#include <stdbool.h>
#include <stm32f1xx_ll_gpio.h>

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

extern const PinDef pin_mtbbus_tx;
extern const PinDef pin_mtbbus_rx;
extern const PinDef pin_mtbbus_te;

extern const PinDef pin_debug_1;
extern const PinDef pin_debug_2;


void gpio_init(void);

void gpio_pin_init(PinDef pin, uint32_t mode, uint32_t pull, uint32_t speed, uint32_t outputType);
//void gpio_pin_deinit(PinDef pin);
bool gpio_pin_read(PinDef pin);
void gpio_pin_write(PinDef pin, bool value);
void gpio_pin_toggle(PinDef pin);

/* Higher-level functions ----------------------------------------------------*/

static inline void gpio_uart_out(void) { gpio_pin_write(pin_mtbbus_te, true); }
static inline void gpio_uart_in(void) { gpio_pin_write(pin_mtbbus_te, false); }
uint8_t gpio_mtbbus_addr(void);
