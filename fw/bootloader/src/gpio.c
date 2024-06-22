#include "gpio.h"
#include "stm32f1xx_hal.h"

const PinDef pin_led_red = {GPIOB, GPIO_PIN_12};
const PinDef pin_led_green = {GPIOB, GPIO_PIN_13};
const PinDef pin_led_blue = {GPIOB, GPIO_PIN_14};
const PinDef pin_btn = {GPIOB, GPIO_PIN_15};

const PinDef pins_addr[] = {
    {GPIOB, GPIO_PIN_0},
    {GPIOB, GPIO_PIN_1},
    {GPIOB, GPIO_PIN_2},
    {GPIOB, GPIO_PIN_3},
    {GPIOB, GPIO_PIN_4},
    {GPIOB, GPIO_PIN_5},
    {GPIOB, GPIO_PIN_6},
    {GPIOB, GPIO_PIN_7},
};

const PinDef pin_mtbbus_tx = {GPIOB, GPIO_PIN_10};
const PinDef pin_mtbbus_rx = {GPIOB, GPIO_PIN_11};
const PinDef pin_mtbbus_te = {GPIOB, GPIO_PIN_9};

const PinDef pin_debug_1 = {GPIOA, GPIO_PIN_5};
const PinDef pin_debug_2 = {GPIOA, GPIO_PIN_6};

/* Local prototypes ----------------------------------------------------------*/

void gpio_pins_init(GPIO_TypeDef* port, uint32_t pinMask, uint32_t mode,
                    uint32_t pull, uint32_t speed, bool de_init_first);

/* Implementation ------------------------------------------------------------*/

void gpio_init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // GPIOA outputs
    gpio_pins_init(
        GPIOA,
        pin_debug_1.pin | pin_debug_2.pin,
        GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, false
    );

    // GPIOB outputs
    gpio_pins_init(
        GPIOB,
        pin_led_blue.pin | pin_led_red.pin | pin_led_green.pin,
        GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, false
    );

    for (size_t i = 0; i < MTBBUS_ADDR_INPUTS; i++)
        gpio_pin_init(pins_addr[i], GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, false);

    gpio_pin_init(pin_btn, GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW, false);
}

void gpio_pins_init(GPIO_TypeDef* port, uint32_t pinMask, uint32_t mode,
                    uint32_t pull, uint32_t speed, bool de_init_first) {

    // HAL_GPIO_Init leaves some flags set if called multiple times
    // on the same pin
    if (de_init_first)
        HAL_GPIO_DeInit(port, pinMask);

    GPIO_InitTypeDef init;
    init.Pin = pinMask;
    init.Mode = mode;
    init.Pull = pull;
    init.Speed = speed;
    HAL_GPIO_Init(port, &init);
}

inline void gpio_pin_init(PinDef pin, uint32_t mode, uint32_t pull, uint32_t speed, bool de_init_first) {
    gpio_pins_init(pin.port, pin.pin, mode, pull, speed, de_init_first);
}

void gpio_pin_deinit(PinDef pin) {
    HAL_GPIO_DeInit(pin.port, pin.pin);
}

bool gpio_pin_read(PinDef pin) {
    return HAL_GPIO_ReadPin(pin.port, pin.pin) == GPIO_PIN_SET;
}

void gpio_pin_write(PinDef pin, bool value) {
    HAL_GPIO_WritePin(pin.port, pin.pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void gpio_pin_toggle(PinDef pin) {
    HAL_GPIO_TogglePin(pin.port, pin.pin);
}

uint8_t gpio_mtbbus_addr(void) {
    uint8_t result = 0;
    for (size_t i = 0; i < MTBBUS_ADDR_INPUTS; i++)
        if (!gpio_pin_read(pins_addr[i]))
            result |= (1 << i);
    return result;
}
