/**
  * STM32F103x8 FLASH memory access.
  * This file in inspired by stm32f1xx_hal_flash.h.
  */

#ifndef __FLASH_H__
#define __FLASH_H__

#include <stdint.h>
#include <stdbool.h>
#include <stm32f1xx.h>

/* Exported constants --------------------------------------------------------*/

#define FLASH_ERROR_NONE                       0x00U  /*!< No error */
#define FLASH_ERROR_PROG                       0x01U  /*!< Programming error */
#define FLASH_ERROR_WRP                        0x02U  /*!< Write protection error */
#define FLASH_ERROR_OPTV                       0x04U  /*!< Option validity error */
#define FLASH_ERROR_UNFINISHED                 0x08U

#define FLASH_PAGE_SIZE                       0x400U

/* Exported functions --------------------------------------------------------*/

uint32_t FLASH_ProgramHalfWord(uint32_t address, uint16_t data);
uint32_t FLASH_ProgramFinish(void);
bool FLASH_LastOperationIsFinished(void);
uint32_t FLASH_WaitForLastOperation(void);
void FLASH_PageErase(uint32_t pageAddress);

ErrorStatus HAL_FLASH_Unlock(void);
void HAL_FLASH_Lock(void);

#endif /* __FLASH_H__ */
