/* Assert definition */

#pragma once

#include <stdint.h>

// Code below is taken from stm32f1xx_hal_conf.h so assert is independent
// on HAL configuration.

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the
  *        HAL drivers code
  */
#define USE_FULL_ASSERT    1U

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr If expr is false, it calls assert_failed function
  *         which reports the name of the source file and the source
  *         line number of the call that failed.
  *         If expr is true, it returns no value.
  * @retval None
  */
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
#define fail() assert_failed((uint8_t *)__FILE__, __LINE__)

void assert_failed(uint8_t* file, uint32_t line);
