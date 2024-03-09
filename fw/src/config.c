#include "config.h"
#include "eeprom.h"
#include "mtbbus.h"

bool config_write = false;
uint16_t config_mtbbus_speed;

#define EEPROM_ADDR_VERSION                (0x00) // uint16
#define EEPROM_ADDR_MTBBUS_SPEED           (0x02) // uint16
#define EEPROM_ADDR_BOOT                   (0x04) // uint16
#define EEPROM_ADDR_BOOTLOADER_VER         (0x06) // uint16


void config_load(void) {
    uint16_t version = ee_read_uint16(EEPROM_ADDR_VERSION);
    if (version == 0xFFFF) {
        // default EEPROM content → reset config
        config_mtbbus_speed = MTBBUS_SPEED_38400;
        config_save();
        return;
    }

    config_mtbbus_speed = ee_read_uint16(EEPROM_ADDR_MTBBUS_SPEED);
    if (config_mtbbus_speed > MTBBUS_SPEED_MAX)
        config_mtbbus_speed = MTBBUS_SPEED_38400;

    uint16_t boot = ee_read_uint16(EEPROM_ADDR_BOOT);
    if (boot != CONFIG_BOOT_NORMAL)
        config_save();
}

void config_save(void) {
    ee_format();
    ee_write_uint16(EEPROM_ADDR_VERSION, 1);
    ee_write_uint16(EEPROM_ADDR_MTBBUS_SPEED, config_mtbbus_speed);
    ee_write_uint16(EEPROM_ADDR_BOOT, CONFIG_BOOT_NORMAL);
}

void config_boot_fwupgd(void) {
    config_save(); // must format flash before saving any data
    ee_write_uint16(EEPROM_ADDR_BOOT, CONFIG_BOOT_FWUPGD);
}

void config_boot_normal(void) {
    config_save(); // must format flash before saving any data
    ee_write_uint16(EEPROM_ADDR_BOOT, CONFIG_BOOT_NORMAL);
}

uint16_t config_bootloader_version() {
    return ee_read_uint16(EEPROM_ADDR_BOOTLOADER_VER);
}
