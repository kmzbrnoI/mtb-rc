/* Includes ------------------------------------------------------------------*/

#include "flash.h"
#include "stm32_assert.h"

/* Private variables ---------------------------------------------------------*/

/* Private macros & constants ------------------------------------------------*/

#define FLASH_SIZE_DATA_REGISTER     0x1FFFF7E0U
#define OBR_REG_INDEX                1U

#define IS_FLASH_PROGRAM_ADDRESS(ADDRESS) (((ADDRESS) >= FLASH_BASE) && (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x80U) ? \
                                           ((ADDRESS) <= FLASH_BANK1_END) :  (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x40U) ? \
                                           ((ADDRESS) <= 0x0800FFFF) :  (((*((uint16_t *)FLASH_SIZE_DATA_REGISTER)) == 0x20U) ? \
                                           ((ADDRESS) <= 0x08007FFF) :  ((ADDRESS) <= 0x08003FFFU)))))

#define FLASH_FLAG_BSY             FLASH_SR_BSY              /*!< FLASH Busy flag                          */
#define FLASH_FLAG_PGERR           FLASH_SR_PGERR            /*!< FLASH Programming error flag             */
#define FLASH_FLAG_WRPERR          FLASH_SR_WRPRTERR         /*!< FLASH Write protected error flag         */
#define FLASH_FLAG_EOP             FLASH_SR_EOP              /*!< FLASH End of Operation flag              */

#define FLASH_FLAG_OPTVERR         ((OBR_REG_INDEX << 8U | FLASH_OBR_OPTERR)) /*!< Option Byte Error        */

/**
  * @brief  Get the specified FLASH flag status.
  * @param  __FLAG__ specifies the FLASH flag to check.
  *          This parameter can be one of the following values:
  *            @arg @ref FLASH_FLAG_EOP    FLASH End of Operation flag
  *            @arg @ref FLASH_FLAG_WRPERR FLASH Write protected error flag
  *            @arg @ref FLASH_FLAG_PGERR  FLASH Programming error flag
  *            @arg @ref FLASH_FLAG_BSY    FLASH Busy flag
  *            @arg @ref FLASH_FLAG_OPTVERR  Loaded OB and its complement do not match
  * @retval The new state of __FLAG__ (SET or RESET).
  */
#define __FLASH_GET_FLAG(__FLAG__)  (((__FLAG__) == FLASH_FLAG_OPTVERR) ? \
                                            (FLASH->OBR & FLASH_OBR_OPTERR) : \
                                            (FLASH->SR & (__FLAG__)))
/**
  * @brief  Clear the specified FLASH flag.
  * @param  __FLAG__ specifies the FLASH flags to clear.
  *          This parameter can be any combination of the following values:
  *            @arg @ref FLASH_FLAG_EOP    FLASH End of Operation flag
  *            @arg @ref FLASH_FLAG_WRPERR FLASH Write protected error flag
  *            @arg @ref FLASH_FLAG_PGERR  FLASH Programming error flag
  *            @arg @ref FLASH_FLAG_OPTVERR  Loaded OB and its complement do not match
  * @retval none
  */
#define __FLASH_CLEAR_FLAG(__FLAG__)   do { \
                          /* Clear FLASH_FLAG_OPTVERR flag */ \
                          if ((__FLAG__) == FLASH_FLAG_OPTVERR) \
                          { \
                            CLEAR_BIT(FLASH->OBR, FLASH_OBR_OPTERR); \
                          } \
                          else { \
                            /* Clear Flag in Bank1 */ \
                            FLASH->SR  = (__FLAG__); \
                          } \
                    } while(0U)

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Program halfword, word or double word at a specified address
  * @note   The function HAL_FLASH_Unlock() should be called before to unlock the FLASH interface
  *         The function HAL_FLASH_Lock() should be called after to lock the FLASH interface
  *
  * @note   If an erase and a program operations are requested simultaneously,
  *         the erase operation is performed before the program one.
  *
  * @note   FLASH should be previously erased before new programmation (only exception to this
  *         is when 0x0000 is programmed)
  *
  * @note   After programming the halfword, poll FLASH_LastOperationIsFinished,
  *         when the operation is finished, call FLASH_ProgramFinish.
  *
  * @param  address:      Specifies the address to be programmed.
  * @param  data:         Specifies the data to be programmed
  */
uint32_t FLASH_ProgramHalfWord(uint32_t address, uint16_t data) {
    assert_param(IS_FLASH_PROGRAM_ADDRESS(address));

    /* Wait for last operation to be completed */
    uint32_t status = FLASH_WaitForLastOperation();
    if (status != SUCCESS)
        return status;

    /* Proceed to program the new data */
    SET_BIT(FLASH->CR, FLASH_CR_PG);

    /* Write data in the address */
    *(__IO uint16_t*)address = data;

    return SUCCESS;
}

// Finish sungle HalfWord programming
uint32_t FLASH_ProgramFinish(void) {
    if (!FLASH_LastOperationIsFinished())
        return FLASH_ERROR_UNFINISHED;

    /* If the program operation is completed, disable the PG Bit */
    CLEAR_BIT(FLASH->CR, FLASH_CR_PG);

    return SUCCESS;
}

// Unlock the FLASH control register access
ErrorStatus HAL_FLASH_Unlock(void) {
    if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET) {
        /* Authorize the FLASH Registers access */
        WRITE_REG(FLASH->KEYR, FLASH_KEY1);
        WRITE_REG(FLASH->KEYR, FLASH_KEY2);

        /* Verify Flash is unlocked */
        if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET)
            return ERROR;
    }

    return SUCCESS;
}

// Locks the FLASH control register access
void HAL_FLASH_Lock(void) {
    /* Set the LOCK Bit to lock the FLASH Registers access */
    SET_BIT(FLASH->CR, FLASH_CR_LOCK);
}

bool FLASH_LastOperationIsFinished(void) {
    return (__FLASH_GET_FLAG(FLASH_FLAG_BSY) == 0);
}

// Wait for a FLASH operation to complete.
uint32_t FLASH_WaitForLastOperation(void) {
    /* Wait for the FLASH operation to complete by polling on BUSY flag to be reset.
       Even if the FLASH operation fails, the BUSY flag will be reset and an error
       flag will be set */

    while (__FLASH_GET_FLAG(FLASH_FLAG_BSY));

    /* Check FLASH End of Operation flag  */
    if (__FLASH_GET_FLAG(FLASH_FLAG_EOP)) {
        /* Clear FLASH End of Operation pending bit */
        __FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
    }

    uint32_t error = SUCCESS;
    uint32_t flags = 0;
    if (__FLASH_GET_FLAG(FLASH_FLAG_WRPERR)) {
        error |= FLASH_ERROR_WRP;
        flags |= FLASH_FLAG_WRPERR;
    }
    if (__FLASH_GET_FLAG(FLASH_FLAG_PGERR)) {
        error |= FLASH_ERROR_PROG;
        flags |= FLASH_FLAG_PGERR;
    }
    if (__FLASH_GET_FLAG(FLASH_FLAG_OPTVERR)) {
        error |= FLASH_ERROR_OPTV;
        __FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
    }

    /* Clear FLASH error pending bits */
    __FLASH_CLEAR_FLAG(flags);

    return error;
}

void FLASH_PageErase(uint32_t pageAddress) {
    SET_BIT(FLASH->CR, FLASH_CR_PER);
    WRITE_REG(FLASH->AR, pageAddress);
    SET_BIT(FLASH->CR, FLASH_CR_STRT);
}
