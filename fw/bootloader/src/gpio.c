#include "gpio.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_bus.h"

const PinDef pin_led_red = {GPIOB, LL_GPIO_PIN_12};
const PinDef pin_led_green = {GPIOB, LL_GPIO_PIN_13};
const PinDef pin_led_blue = {GPIOB, LL_GPIO_PIN_14};
const PinDef pin_btn = {GPIOB, LL_GPIO_PIN_15};

const PinDef pins_addr[] = {
    {GPIOB, LL_GPIO_PIN_0},
    {GPIOB, LL_GPIO_PIN_1},
    {GPIOB, LL_GPIO_PIN_2},
    {GPIOB, LL_GPIO_PIN_3},
    {GPIOB, LL_GPIO_PIN_4},
    {GPIOB, LL_GPIO_PIN_5},
    {GPIOB, LL_GPIO_PIN_6},
    {GPIOB, LL_GPIO_PIN_7},
};

const PinDef pin_mtbbus_tx = {GPIOB, LL_GPIO_PIN_10};
const PinDef pin_mtbbus_rx = {GPIOB, LL_GPIO_PIN_11};
const PinDef pin_mtbbus_te = {GPIOB, LL_GPIO_PIN_9};

const PinDef pin_debug_1 = {GPIOA, LL_GPIO_PIN_5};
const PinDef pin_debug_2 = {GPIOA, LL_GPIO_PIN_6};

/* Local prototypes ----------------------------------------------------------*/

void gpio_pins_init(GPIO_TypeDef* port, uint32_t pin, uint32_t mode,
                    uint32_t pull, uint32_t speed, uint32_t outputType);

/* Implementation ------------------------------------------------------------*/

void gpio_init(void) {
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

    gpio_pin_init(pin_debug_1, LL_GPIO_MODE_OUTPUT, 0, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_OUTPUT_PUSHPULL);
    gpio_pin_init(pin_debug_2, LL_GPIO_MODE_OUTPUT, 0, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_OUTPUT_PUSHPULL);

    gpio_pin_init(pin_led_red, LL_GPIO_MODE_OUTPUT, 0, LL_GPIO_SPEED_FREQ_LOW, LL_GPIO_OUTPUT_PUSHPULL);
    gpio_pin_init(pin_led_green, LL_GPIO_MODE_OUTPUT, 0, LL_GPIO_SPEED_FREQ_LOW, LL_GPIO_OUTPUT_PUSHPULL);
    gpio_pin_init(pin_led_blue, LL_GPIO_MODE_OUTPUT, 0, LL_GPIO_SPEED_FREQ_LOW, LL_GPIO_OUTPUT_PUSHPULL);

    for (size_t i = 0; i < MTBBUS_ADDR_INPUTS; i++)
        gpio_pin_init(pins_addr[i], LL_GPIO_MODE_INPUT, LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_LOW, 0);

    gpio_pin_init(pin_btn, LL_GPIO_MODE_INPUT, LL_GPIO_PULL_UP, LL_GPIO_SPEED_FREQ_LOW, 0);
}

void gpio_pins_init(GPIO_TypeDef* port, uint32_t pin, uint32_t mode,
                    uint32_t pull, uint32_t speed, uint32_t outputType) {
    LL_GPIO_InitTypeDef init;
    init.Pin = pin;
    init.Mode = mode;
    init.Speed = speed;
    init.OutputType = outputType;
    init.Pull = pull;
    assert_param(LL_GPIO_Init(port, &init) == SUCCESS);
}

inline void gpio_pin_init(PinDef pin, uint32_t mode, uint32_t pull, uint32_t speed,
                          uint32_t outputType) {
    gpio_pins_init(pin.port, pin.pin, mode, pull, speed, outputType);
}

bool gpio_pin_read(PinDef pin) {
    // LL_GPIO_PIN8-15 is not (1 << GPIO), see stm32f1xx_ll_gpio.h
    // -> transform to (1 << GPIO)
    uint32_t _pin = pin.pin;
    if (_pin> 0xFFFF)
        _pin <<= 8;
    return (LL_GPIO_ReadInputPort(pin.port) & _pin) > 0;
}

void gpio_pin_write(PinDef pin, bool value) {
    if (value) {
        LL_GPIO_SetOutputPin(pin.port, pin.pin);
    } else {
        LL_GPIO_ResetOutputPin(pin.port, pin.pin);
    }
}

void gpio_pin_toggle(PinDef pin) {
    LL_GPIO_TogglePin(pin.port, pin.pin);
}

uint8_t gpio_mtbbus_addr(void) {
    uint8_t result = 0;
    for (size_t i = 0; i < MTBBUS_ADDR_INPUTS; i++)
        if (!gpio_pin_read(pins_addr[i]))
            result |= (1 << i);
    return result;
}
