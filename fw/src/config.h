/* General configuration of a MTB-RC module
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

extern bool config_write;
extern uint16_t config_mtbbus_speed;

// Warning: these functions take longer time to execute
void config_load(void);
void config_save(void);

void config_boot_fwupgd(void);
void config_boot_normal(void);

uint16_t config_bootloader_version(void);

#define CONFIG_MODULE_TYPE 0x30
#define CONFIG_FW_MAJOR 1
#define CONFIG_FW_MINOR 0
#define CONFIG_PROTO_MAJOR 4
#define CONFIG_PROTO_MINOR 1

#define CONFIG_BOOT_FWUPGD 0x01
#define CONFIG_BOOT_NORMAL 0x00
