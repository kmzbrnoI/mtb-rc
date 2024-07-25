/* Persistent memory access
 * In STM32, flash memory is used to store persistent data.
 * EEPROM data must be aligned to 16-bit blocks, as writing to 8-bit
 * block causes writing to already-written block, which is forbidden.
 * Each write to a 16-bits block must be directly preceeded with clearing of
 * a page with this block.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define EE_FLASH_PAGE 63 // STM21F103C8 has 64 kB of FLASH, use latest page
#define EE_START (0x08000000 + FLASH_PAGE_SIZE*EE_FLASH_PAGE)

void ee_format(void);
void ee_read(uint32_t virtAdd, uint32_t len, uint8_t* data);
void ee_write(uint32_t virtAddr, uint32_t len, uint8_t* data);

uint16_t ee_read_uint16(uint32_t virtAddr);
void ee_write_uint16(uint32_t virtAddr, uint16_t value);
