#include <stdbool.h>
#include <string.h>
#include "dwt_delay.h"
#include "assert.h"
#include "main.h"
#include "gpio.h"
#include "mtbbus.h"
#include "eeprom.h"

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim3; // general-purpose @ 500 us

/* Private function prototypes -----------------------------------------------*/

static void init_clock(void);
static void init_tim3(void);
static void timer_10ms(void);
static void _mtbbus_init(void);

void mtbbus_received(bool broadcast, uint8_t command_code, uint8_t *data, uint8_t data_len); // intentionally not static
static void mtbbus_send_ack(void);
static void mtbbus_send_error(uint8_t code);

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
    bool success = (HAL_Init() == HAL_OK);
    assert_param(success);

    init_clock();
    DWT_Init();

    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    init_tim3();

    gpio_pin_write(pin_led_red, true);
    // gpio_pin_write(pin_led_green, false); // TODO enable?
    // gpio_pin_write(pin_led_blue, false); // TODO enable?

    /* uint8_t boot = eeprom_read_byte(EEPROM_ADDR_BOOT);
    if (boot != CONFIG_BOOT_NORMAL)
        eeprom_write_byte(EEPROM_ADDR_BOOT, CONFIG_BOOT_NORMAL);

    eeprom_update_byte(EEPROM_ADDR_BOOTLOADER_VER_MAJOR, CONFIG_FW_MAJOR);
    eeprom_update_byte(EEPROM_ADDR_BOOTLOADER_VER_MINOR, CONFIG_FW_MINOR);

    if ((boot != CONFIG_BOOT_FWUPGD) && (io_button()))
        check_and_boot(); */

    // Not booting → start MTBbus
    _mtbbus_init();

    while (true) {
        mtbbus_update();
    }
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

// General-purpose TIM3 @ 500 us
void TIM3_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim3);
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
