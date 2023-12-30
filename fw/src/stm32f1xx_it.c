/* Interrupt Service Routines */

#include "main.h"
#include "stm32f1xx_it.h"

/* External variables --------------------------------------------------------*/

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

/* Implementation ------------------------------------------------------------*/

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

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

// TIM1 capture compare interrupt
void TIM1_CC_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim1);
}

// TIM2 global interrupt.
void TIM2_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim2);
}
