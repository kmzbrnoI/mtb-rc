#include <stdbool.h>
#include <string.h>
#include <stm32f1xx_ll_rcc.h>
#include <stm32f1xx_ll_system.h>
#include <stm32f1xx_ll_utils.h>
#include <stm32f1xx_ll_bus.h>
#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_tim.h>

#include "dwt_delay.h"
#include "stm32_assert.h"
#include "main.h"
#include "gpio.h"
#include "mtbbus.h"
#include "eeprom.h"

/* Private variables ---------------------------------------------------------*/

//TIM_HandleTypeDef htim3; // general-purpose @ 500 us

/* Private function prototypes -----------------------------------------------*/

static void init_clock(void);
static void init_tim3(void);
static void timer_10ms(void);
static void _mtbbus_init(void);

void mtbbus_received(bool broadcast, uint8_t command_code, uint8_t *data, uint8_t data_len); // intentionally not static
static void mtbbus_send_ack(void);
static void mtbbus_send_error(uint8_t code);

static void check_and_boot(void);

/* Defines -------------------------------------------------------------------*/

#define EEPROM_ADDR_VERSION                (0x00) // uint16
#define EEPROM_ADDR_MTBBUS_SPEED           (0x02) // uint16
#define EEPROM_ADDR_BOOT                   (0x04) // uint16
#define EEPROM_ADDR_BOOTLOADER_VER         (0x06) // uint16

#define CONFIG_MODULE_TYPE 0x30
#define CONFIG_FW_MAJOR 1
#define CONFIG_FW_MINOR 0
#define CONFIG_PROTO_MAJOR 4
#define CONFIG_PROTO_MINOR 1

#define CONFIG_BOOT_FWUPGD 0x01
#define CONFIG_BOOT_NORMAL 0x00

/* Global variables ----------------------------------------------------------*/

typedef union {
    struct {
        bool addr_zero : 1;
        bool crc: 1;
    } bits;
    uint8_t all;
} error_flags_t;

volatile error_flags_t error_flags = {0};

/* Implementation ------------------------------------------------------------*/

int main(void) {
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    /* System interrupt init */
    //NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* SysTick_IRQn interrupt configuration */
    //NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));

    /* NOJTAG: JTAG-DP Disabled and SW-DP Enabled */
    LL_GPIO_AF_Remap_SWJ_NOJTAG();

    init_clock();

    /* Initialize all configured peripherals */
    gpio_init();
    //MX_DMA_Init();
    //MX_USART1_UART_Init();
    //MX_USART2_UART_Init();
    //MX_USART3_UART_Init();
    //MX_TIM1_Init();
    //MX_TIM2_Init();

    init_tim3();

    // gpio_pin_write(pin_led_green, false); // TODO enable?
    // gpio_pin_write(pin_led_blue, false); // TODO enable?

    /* uint8_t boot = eeprom_read_byte(EEPROM_ADDR_BOOT);
    if (boot != CONFIG_BOOT_NORMAL)
        eeprom_write_byte(EEPROM_ADDR_BOOT, CONFIG_BOOT_NORMAL);

    eeprom_update_byte(EEPROM_ADDR_BOOTLOADER_VER_MAJOR, CONFIG_FW_MAJOR);
    eeprom_update_byte(EEPROM_ADDR_BOOTLOADER_VER_MINOR, CONFIG_FW_MINOR);*/

    if (/*(boot != CONFIG_BOOT_FWUPGD) && */(!gpio_pin_read(pin_btn)))
        check_and_boot();

    // Not booting → start MTBbus
    _mtbbus_init();

    while (true) {
        mtbbus_update();

        if (LL_TIM_IsActiveFlag_CC1(TIM3)) {
            LL_TIM_IsActiveFlag_CC1(TIM3);

            static bool state = false;
            gpio_pin_write(pin_led_green, state);
            state = !state;
        }
    }
}

void init_clock(void) {
    // Configura flash latency
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1);

    // Configure HSE
    LL_RCC_HSE_Enable();
    while (!LL_RCC_HSE_IsReady());

    // Configure PLL
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_4);
    LL_RCC_PLL_Enable();
    while (LL_RCC_PLL_IsReady() != 1);

    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

    // Set System Clock Mux clock source
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

    LL_Init1msTick(48000000);
    LL_SetSystemCoreClock(48000000);
}

void init_tim3(void) {
    // General-purpose TIM3 @ 10 ms
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

    LL_TIM_InitTypeDef TIM_InitStruct = {0};
    TIM_InitStruct.Prescaler = 48;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 10000;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM3, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM3);
    LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM3);
}

void _mtbbus_init(void) {
    uint8_t config_mtbbus_speed = ee_read_uint16(EEPROM_ADDR_MTBBUS_SPEED);
    if (config_mtbbus_speed > MTBBUS_SPEED_MAX)
        config_mtbbus_speed = MTBBUS_SPEED_38400;

    uint8_t _mtbbus_addr = gpio_mtbbus_addr();
    error_flags.bits.addr_zero = (_mtbbus_addr == 0);

    mtbbus_init(_mtbbus_addr, config_mtbbus_speed);
    mtbbus_on_receive = mtbbus_received;
}

/* System stuff --------------------------------------------------------------*/

// Non-maskable interrupt
void NMI_Handler(void) {
    fail();
}

void HardFault_Handler(void) {
    fail();
}

void MemManage_Handler(void) {
    fail();
}

void BusFault_Handler(void) {
    fail();
}

void UsageFault_Handler(void) {
    fail();
}

// System service call via SWI instruction
void SVC_Handler(void) {
    fail();
}

// Debug monitor
void DebugMon_Handler(void) {
    fail();
}

// Pendable request for system service
void PendSV_Handler(void) {
    fail();
}

// System tick timer
void SysTick_Handler(void) {
    fail();
}

#ifdef USE_FULL_ASSERT
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
        LL_mDelay(200);
        gpio_pin_write(pin_led_red, false);
        LL_mDelay(200);
    }
}
#endif /* USE_FULL_ASSERT */

/* "Application" code --------------------------------------------------------*/

// General-purpose TIM3 @ 500 us
void TIM3_IRQHandler(void) {
    //HAL_TIM_IRQHandler(&htim3);
}

/* MTBbus --------------------------------------------------------------------*/

void mtbbus_received(bool broadcast, uint8_t command_code, uint8_t *data, uint8_t data_len) {
    DWT_Delay(2);

    switch (command_code) {

    case MTBBUS_CMD_MOSI_MODULE_INQUIRY:
        if (!broadcast) {
            mtbbus_send_ack();
        } else { goto INVALID_MSG; }
        break;

    case MTBBUS_CMD_MOSI_INFO_REQ:
        if (!broadcast) {
            mtbbus_output_buf[0] = 7;
            mtbbus_output_buf[1] = MTBBUS_CMD_MISO_MODULE_INFO;
            mtbbus_output_buf[2] = CONFIG_MODULE_TYPE;
            mtbbus_output_buf[3] = (error_flags.bits.crc) ? 2 : 1;
            mtbbus_output_buf[4] = CONFIG_FW_MAJOR;
            mtbbus_output_buf[5] = CONFIG_FW_MINOR;
            mtbbus_output_buf[6] = CONFIG_PROTO_MAJOR;
            mtbbus_output_buf[7] = CONFIG_PROTO_MINOR;
            mtbbus_send_buf_autolen();
        } else { goto INVALID_MSG; }
        break;

    case MTBBUS_CMD_MOSI_CHANGE_SPEED:
        if (data_len >= 1) {
            mtbbus_set_speed(data[0]);
            if (!broadcast)
                mtbbus_send_ack();
            // eeprom_update_byte(EEPROM_ADDR_MTBBUS_SPEED, data[0]);
        } else { goto INVALID_MSG; }
        break;

    case MTBBUS_CMD_MOSI_WRITE_FLASH:
        if ((data_len >= 1) && (!broadcast)) {
            // TODO
            mtbbus_send_ack();
        } else { goto INVALID_MSG; }
        break;

    case MTBBUS_CMD_MOSI_WRITE_FLASH_STATUS_REQ:
        if (!broadcast) {
            // TODO
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

    case MTBBUS_CMD_MOSI_FWUPGD_REQUEST:
        if (!broadcast) {
            mtbbus_send_ack();
        } else { goto INVALID_MSG; }
        break;


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

void mtbbus_send_error(uint8_t code) {
    mtbbus_output_buf[0] = 2;
    mtbbus_output_buf[1] = MTBBUS_CMD_MISO_ERROR;
    mtbbus_output_buf[2] = code;
    mtbbus_send_buf_autolen();
}

///////////////////////////////////////////////////////////////////////////////

void check_and_boot(void) {
    while (true) {
        gpio_pin_write(pin_led_blue, true);
        LL_mDelay(500);
        gpio_pin_write(pin_led_blue, false);
        LL_mDelay(500);
    }
}
