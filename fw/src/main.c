#include <stdbool.h>
#include "dwt_delay.h"
#include "assert.h"
#include "main.h"
#include "gpio.h"
#include "mtbbus.h"
#include "railcom_ll.h"
#include "config.h"
#include "diag.h"
#include "inputs.h"

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2; // general-purpose @ 10 ms
TIM_HandleTypeDef htim3; // general-purpose @ 500 us

#define LED_GR_ON 5
#define LED_GR_OFF 2
uint8_t led_gr_counter = 0;

#define LED_RED_OK_ON 40
#define LED_RED_OK_OFF 20
#define LED_RED_ERR_ON 100
#define LED_RED_ERR_OFF 50
uint8_t led_red_counter = 0;

bool beacon = false;

#define LED_BLUE_BEACON_ON 100
#define LED_BLUE_BEACON_OFF 50
uint8_t led_blue_counter = 0;

volatile bool btn_debounce_to_update = false;

#define MTBBUS_TIMEOUT_MAX 100 // 1 s
volatile uint8_t mtbbus_timeout = MTBBUS_TIMEOUT_MAX; // increment each 10 ms

#define BTN_PRESS_1S 100
volatile uint8_t btn_press_time = 0;

volatile bool mtbbus_auto_speed_in_progress = false;
volatile uint8_t mtbbus_auto_speed_timer = 0;
volatile MtbBusSpeed mtbbus_auto_speed_last;
#define MTBBUS_AUTO_SPEED_TIMEOUT 20 // 200 ms

volatile bool t2_elapsed = false;

/* Private function prototypes -----------------------------------------------*/

static void init(void);
static void init_clock(void);
static void init_tim1(void);
static void init_tim2(void);
static void init_tim3(void);

void mtbbus_received(bool broadcast, uint8_t command_code, uint8_t *data, uint8_t data_len); // intentionally not static
static void mtbbus_send_ack(void);
static void mtbbus_send_inputs(uint8_t message_code);
static void mtbbus_send_error(uint8_t code);
static void mtbbus_update_polarity(void);
static inline bool mtbbus_addressed(void);

static void autodetect_mtbbus_speed(void);
static void autodetect_mtbbus_speed_stop(void);
static void mtbbus_auto_speed_next(void);
static void mtbbus_auto_speed_received(void);

static void leds_update(void);
static void led_red_ok(void);

static void btn_short_press(void);
static void btn_long_press(void);

/* Implementation ------------------------------------------------------------*/

int main(void) {
    init();

    while (true) {
        mtbbus_update();

        if (btn_debounce_to_update) {
            btn_debounce_to_update = false;
            btn_debounce_update();
        }

        if (config_write) {
            config_write = false;
            config_save();
        }

        if (btn_press_time == BTN_PRESS_1S) {
            btn_press_time = 0xFF;
            btn_long_press();
        }

        if ((mtbbus_auto_speed_in_progress) && (mtbbus_auto_speed_timer == MTBBUS_AUTO_SPEED_TIMEOUT))
            mtbbus_auto_speed_next();

        if (t2_elapsed) {
            t2_elapsed = false;
            leds_update();
        }
    }
}

/* Init ----------------------------------------------------------------------*/

void init(void) {
    bool success = (HAL_Init() == HAL_OK);
    assert_param(success);

    init_clock();
    DWT_Init();

    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    gpio_init();

    gpio_pin_write(pin_led_red, true);
    gpio_pin_write(pin_led_green, true);
    gpio_pin_write(pin_led_blue, true);

    //init_tim1();
    init_tim2();
    init_tim3();

    config_load();

    uint8_t _mtbbus_addr = gpio_mtbbus_addr();
    error_flags.bits.addr_zero = (_mtbbus_addr == 0);
    mtbbus_init(_mtbbus_addr, config_mtbbus_speed);
    mtbbus_on_receive = mtbbus_received;
    mtbbus_update_polarity();

    railcom_ll_init();

    HAL_Delay(200);

    gpio_pin_write(pin_led_red, false);
    gpio_pin_write(pin_led_green, false);
    gpio_pin_write(pin_led_blue, false);
}

void init_clock(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
    assert_param(HAL_RCC_OscConfig(&RCC_OscInitStruct) == HAL_OK);

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    assert_param(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) == HAL_OK);

    /** Enables the Clock Security System
    */
    HAL_RCC_EnableCSS();
}

void init_tim1(void) {
    //TIM_IC_InitTypeDef sConfigIC = {0};

    __HAL_RCC_TIM1_CLK_ENABLE();

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 65535;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    assert_param(HAL_TIM_Base_Init(&htim1) == HAL_OK);

    /*sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    assert_param(HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) == HAL_OK);*/

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
}

void init_tim2(void) {
    // General-purpose TIM2 @ 10 ms
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 100;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 4800;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    assert_param(HAL_TIM_Base_Init(&htim2) == HAL_OK);

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    HAL_TIM_Base_Start_IT(&htim2);
}

void init_tim3(void) {
    // General-purpose TIM3 @ 500 us
    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 100;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 240;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    assert_param(HAL_TIM_Base_Init(&htim3) == HAL_OK);

    /* TIM3 interrupt Init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    HAL_TIM_Base_Start_IT(&htim3);
}

/* System stuff --------------------------------------------------------------*/

// Non-maskable interrupt
void NMI_Handler(void) {
    HAL_RCC_NMI_IRQHandler();
    while (1) {
    }
}

void HardFault_Handler(void) {
    while (1) {
    }
}

void MemManage_Handler(void) {
    while (1) {
    }
}

void BusFault_Handler(void) {
    while (1) {
    }
}

void UsageFault_Handler(void) {
    while (1) {
    }
}

// System service call via SWI instruction
void SVC_Handler(void) {
}

// Debug monitor
void DebugMon_Handler(void) {
}

// Pendable request for system service
void PendSV_Handler(void) {
}

// System tick timer
void SysTick_Handler(void) {
    HAL_IncTick();
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
    __disable_irq();

    gpio_pin_write(pin_led_green, false);
    gpio_pin_write(pin_led_blue, false);
    while (1) {
        gpio_pin_write(pin_led_red, true);
        for (size_t i = 0; i < (1U << 20); i++) { __asm__("NOP"); }
        gpio_pin_write(pin_led_red, false);
        for (size_t i = 0; i < (1U << 20); i++) { __asm__("NOP"); }
    }
}
#endif /* USE_FULL_ASSERT */

/* "Application" code --------------------------------------------------------*/

// TIM1 global interrupt
void TIM1_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim1);
}

// General-purpose TIM2 @ 10 ms
void TIM2_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim2);
    t2_elapsed = true;

    if (mtbbus_timeout < MTBBUS_TIMEOUT_MAX)
        mtbbus_timeout++;

    if ((btn_pressed) && (btn_press_time < BTN_PRESS_1S))
        btn_press_time++;

    if ((mtbbus_auto_speed_in_progress) && (mtbbus_auto_speed_timer < MTBBUS_AUTO_SPEED_TIMEOUT))
        mtbbus_auto_speed_timer++;
}

// General-purpose TIM3 @ 500 us
void TIM3_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim3);
    btn_debounce_to_update = true;
}

/* MTBbus --------------------------------------------------------------------*/

void mtbbus_received(bool broadcast, uint8_t command_code, uint8_t *data, uint8_t data_len) {
    //if (!initialized)
    //    return;

    error_flags.bits.bad_mtbbus_polarity = false;
    if (led_gr_counter == 0) {
        gpio_pin_write(pin_led_green, true);
        led_gr_counter = LED_GR_ON;
    }
    DWT_Delay(2);

    mtbbus_timeout = 0;
    if (mtbbus_auto_speed_in_progress)
        mtbbus_auto_speed_received();

    switch (command_code) {

    case MTBBUS_CMD_MOSI_MODULE_INQUIRY:
        if ((!broadcast) && (data_len >= 1)) {
            //static bool last_input_changed = false;
            //static bool last_diag_changed = false;
            //static bool first_scan = true;
            //bool last_ok = data[0] & 0x01;
            mtbbus_send_ack();
        } else { goto INVALID_MSG; }
        break;

    case MTBBUS_CMD_MOSI_INFO_REQ:
        if (!broadcast) {
            uint16_t bootloader_ver = config_bootloader_version();
            mtbbus_output_buf[0] = 9;
            mtbbus_output_buf[1] = MTBBUS_CMD_MISO_MODULE_INFO;
            mtbbus_output_buf[2] = CONFIG_MODULE_TYPE;
            mtbbus_output_buf[3] = (mtbbus_warn_flags.all > 0) << 2;
            mtbbus_output_buf[4] = CONFIG_FW_MAJOR;
            mtbbus_output_buf[5] = CONFIG_FW_MINOR;
            mtbbus_output_buf[6] = CONFIG_PROTO_MAJOR;
            mtbbus_output_buf[7] = CONFIG_PROTO_MINOR;
            mtbbus_output_buf[8] = bootloader_ver >> 8;
            mtbbus_output_buf[9] = bootloader_ver & 0xFF;
            mtbbus_send_buf_autolen();
        } else { goto INVALID_MSG; }
        break;

    case MTBBUS_CMD_MOSI_SET_CONFIG:
        if ((data_len == 0) && (!broadcast)) { // no configuration for MTB-RC module
            mtbbus_send_ack();
        } else { goto INVALID_MSG; }
        break;

    case MTBBUS_CMD_MOSI_GET_CONFIG:
        if (!broadcast) {
            mtbbus_output_buf[0] = 1;
            mtbbus_output_buf[1] = MTBBUS_CMD_MISO_MODULE_CONFIG;
            mtbbus_send_buf_autolen();
        } else { goto INVALID_MSG; }
        break;

    case MTBBUS_CMD_MOSI_BEACON:
        if (data_len >= 1) {
            beacon = data[0];
            if (!broadcast)
                mtbbus_send_ack();
        } else { goto INVALID_MSG; }
        break;

    case MTBBUS_CMD_MOSI_GET_INPUT:
        if (!broadcast) {
            mtbbus_send_inputs(MTBBUS_CMD_MISO_INPUT_STATE);
        } else { goto INVALID_MSG; }
        break;

    case MTBBUS_CMD_MOSI_RESET_OUTPUTS:
        if (!broadcast)
            mtbbus_send_ack();
        break;

    case MTBBUS_CMD_MOSI_CHANGE_ADDR:
        if ((data_len >= 1) && (!broadcast)) {
            mtbbus_send_error(MTBBUS_ERROR_UNSUPPORTED_COMMAND);
        } else { goto INVALID_MSG; }
        break;

    case MTBBUS_CMD_MOSI_CHANGE_SPEED:
        if (data_len >= 1) {
            config_mtbbus_speed = data[0];
            config_write = true;
            mtbbus_set_speed(data[0]);

            if (!broadcast)
                mtbbus_send_ack();
        } else { goto INVALID_MSG; }
        break;

    case MTBBUS_CMD_MOSI_FWUPGD_REQUEST:
        if ((data_len >= 1) && (!broadcast)) {
            config_boot_fwupgd();
            mtbbus_on_sent = &NVIC_SystemReset;
            mtbbus_send_ack();
        } else { goto INVALID_MSG; }
        break;

    case MTBBUS_CMD_MOSI_REBOOT:
        if (broadcast) {
            NVIC_SystemReset();
        } else {
            mtbbus_on_sent = &NVIC_SystemReset;
            mtbbus_send_ack();
        }
        break;

/*    case MTBBUS_CMD_MOSI_DIAG_VALUE_REQ:
        if (data_len >= 1) {
            send_diag_value(data[0]);
        } else { goto INVALID_MSG; }
        break; */

INVALID_MSG:
    default:
        if (!broadcast)
            mtbbus_send_error(MTBBUS_ERROR_UNKNOWN_COMMAND);

    };
}

// Warning: functions below don't check mtbbus_can_fill_output_buf(), bacause
// they should be called ONLY from mtbbus_received event (as MTBbus is
// request-response based bus).

void mtbbus_send_ack(void) {
    mtbbus_output_buf[0] = 1;
    mtbbus_output_buf[1] = MTBBUS_CMD_MISO_ACK;
    mtbbus_send_buf_autolen();
}

void mtbbus_send_inputs(uint8_t message_code) {
    mtbbus_output_buf[0] = 2;
    mtbbus_output_buf[1] = message_code;
    mtbbus_output_buf[2] = 0; // TODO
    mtbbus_send_buf_autolen();
}

void mtbbus_send_error(uint8_t code) {
    mtbbus_output_buf[0] = 2;
    mtbbus_output_buf[1] = MTBBUS_CMD_MISO_ERROR;
    mtbbus_output_buf[2] = code;
    mtbbus_send_buf_autolen();
}

void mtbbus_update_polarity(void) {
    error_flags.bits.bad_mtbbus_polarity = !gpio_pin_read(pin_mtbbus_rx);
}

/* MTBbus speed autodetection ------------------------------------------------*/

void autodetect_mtbbus_speed(void) {
    gpio_pin_write(pin_led_blue, true);
    mtbbus_auto_speed_in_progress = true;
    mtbbus_auto_speed_last = MTBBUS_SPEED_38400;
    mtbbus_auto_speed_next();
}

void mtbbus_auto_speed_next(void) {
    mtbbus_auto_speed_timer = 0;
    mtbbus_auto_speed_last++; // relies on continuous interval of speeds
    if (mtbbus_auto_speed_last > MTBBUS_SPEED_MAX)
        mtbbus_auto_speed_last = MTBBUS_SPEED_38400;
    mtbbus_set_speed(mtbbus_auto_speed_last);
}

void mtbbus_auto_speed_received(void) {
    mtbbus_auto_speed_in_progress = false;
    config_mtbbus_speed = mtbbus_speed;
    config_write = true;
    gpio_pin_write(pin_led_blue, false);
}

void autodetect_mtbbus_speed_stop(void) {
    if (mtbbus_auto_speed_in_progress) {
        mtbbus_auto_speed_in_progress = false;
        gpio_pin_write(pin_led_blue, false);
    }
}

bool mtbbus_addressed(void) {
    return mtbbus_timeout < MTBBUS_TIMEOUT_MAX;
}

/* LEDs ----------------------------------------------------------------------*/

void leds_update(void) {
    if (led_gr_counter > 0) {
        led_gr_counter--;
        if (led_gr_counter == LED_GR_OFF)
            gpio_pin_write(pin_led_green, false);
    }

    bool led_red_flashing = error_flags.all;

    if (led_red_counter > 0) {
        led_red_counter--;
        if (((!led_red_flashing) && (led_red_counter == LED_RED_OK_OFF)) ||
            ((led_red_flashing) && (led_red_counter == LED_RED_ERR_OFF)))
            gpio_pin_write(pin_led_red, false);
    }
    if ((led_red_flashing) && (led_red_counter == 0)) {
        led_red_counter = LED_RED_ERR_ON;
        gpio_pin_write(pin_led_red, true);
    }

    if (led_blue_counter > 0) {
        led_blue_counter--;
        if (led_blue_counter == LED_BLUE_BEACON_OFF)
            gpio_pin_write(pin_led_blue, false);
    }
    if ((beacon) && (led_blue_counter == 0)) {
        led_blue_counter = LED_BLUE_BEACON_ON;
        gpio_pin_write(pin_led_blue, true);
    }
}

void led_red_ok(void) {
    if (led_red_counter == 0) {
        led_red_counter = LED_RED_OK_ON;
        gpio_pin_write(pin_led_red, true);
    }
}

/* Button --------------------------------------------------------------------*/

void btn_on_pressed(void) {
    btn_press_time = 0;
}

void btn_on_released(void) {
    if (btn_press_time < BTN_PRESS_1S)
        btn_short_press();
}

void btn_short_press(void) {
    if (mtbbus_auto_speed_in_progress) {
        autodetect_mtbbus_speed_stop();
        return;
    }

    uint8_t _mtbbus_addr = gpio_mtbbus_addr();
    error_flags.bits.addr_zero = (_mtbbus_addr == 0);
    mtbbus_addr = _mtbbus_addr;
    if (mtbbus_addr != 0)
        led_red_ok();
    mtbbus_update_polarity();
}

void btn_long_press(void) {
    if (!mtbbus_addressed())
        autodetect_mtbbus_speed();
}
