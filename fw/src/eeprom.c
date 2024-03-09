#include "eeprom.h"
#include "assert.h"

///////////////////////////////////////////////////////////////////////////////

void ee_format(void) {
    assert_param(HAL_FLASH_Unlock() == HAL_OK);

    FLASH_EraseInitTypeDef flashErase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .PageAddress = EE_START,
        .NbPages = 1
    };
    uint32_t error;
    assert_param(HAL_FLASHEx_Erase(&flashErase, &error) == HAL_OK);
    assert_param(error == 0xFFFFFFFF);

    assert_param(HAL_FLASH_Lock() == HAL_OK);
}

void ee_read(uint32_t virtAddr, uint32_t len, uint8_t* data) {
    assert_param(virtAddr < FLASH_PAGE_SIZE);
    assert_param(data != NULL);

    for (uint32_t i = 0; i < len; i++, data++)
        *data = (*(__IO uint8_t*) (virtAddr+i + EE_START));
}

void ee_write(uint32_t virtAddr, uint32_t len, uint8_t* data) {
    // Page must be erased (ee_format) before it's written!
    assert_param(virtAddr < FLASH_PAGE_SIZE);
    assert_param((virtAddr%2) == 0);
    assert_param(data != NULL);
    assert_param((len%2) == 0);

    assert_param(HAL_FLASH_Unlock() == HAL_OK);
    for (uint32_t i = 0; i < len; i += 2)
        assert_param(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, EE_START+virtAddr+i, data[i] | (data[i+1] << 8)) == HAL_OK);
    assert_param(HAL_FLASH_Lock() == HAL_OK);
}

uint16_t ee_read_uint16(uint32_t virtAddr) {
    uint16_t result;
    ee_read(virtAddr, sizeof(result), (uint8_t*)&result);
    return result;
}

void ee_write_uint16(uint32_t virtAddr, uint16_t value) {
    ee_write(virtAddr, sizeof(value), (uint8_t*)&value);
}
