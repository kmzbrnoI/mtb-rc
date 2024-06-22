TARGET = mtb-rc-bootloader
DEBUG = 1
OPT = -Os
BUILD_DIR = build

ifndef STM32_CUBE_PATH_185
$(error STM32_CUBE_PATH_185 is not set)
endif

STM32_SRC_PATH = $(STM32_CUBE_PATH_185)/Drivers/STM32F1xx_HAL_Driver/Src
STM32_DRIVERS_PATH = $(STM32_CUBE_PATH_185)/Drivers
# Something like '~/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.5/Drivers/STM32F1xx_HAL_Driver/Src/'

C_SOURCES = \
	$(STM32_SRC_PATH)/stm32f1xx_hal_gpio_ex.c \
	$(STM32_SRC_PATH)/stm32f1xx_hal_tim.c \
	$(STM32_SRC_PATH)/stm32f1xx_hal_tim_ex.c \
	$(STM32_SRC_PATH)/stm32f1xx_hal.c \
	$(STM32_SRC_PATH)/stm32f1xx_hal_rcc.c \
	$(STM32_SRC_PATH)/stm32f1xx_hal_rcc_ex.c \
	$(STM32_SRC_PATH)/stm32f1xx_hal_gpio.c \
	$(STM32_SRC_PATH)/stm32f1xx_hal_dma.c \
	$(STM32_SRC_PATH)/stm32f1xx_hal_cortex.c \
	$(STM32_SRC_PATH)/stm32f1xx_hal_pwr.c \
	$(STM32_SRC_PATH)/stm32f1xx_hal_flash.c \
	$(STM32_SRC_PATH)/stm32f1xx_hal_flash_ex.c \
	$(STM32_SRC_PATH)/stm32f1xx_hal_exti.c \
	$(STM32_SRC_PATH)/stm32f1xx_hal_uart.c \
	lib/dwt_delay/dwt_delay.c \
	lib/eeprom/eeprom.c \
	$(wildcard src/mtbbus/*.c) \
	$(wildcard src/*.c)

# ASM sources
ASM_SOURCES = startup_stm32f103xb.s

# ASM sources
ASMM_SOURCES =


PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

CPU = -mcpu=cortex-m3
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

AS_DEFS =
C_DEFS = \
	-DUSE_HAL_DRIVER \
	-DSTM32F103xB

AS_INCLUDES =
C_INCLUDES = \
	-I$(STM32_DRIVERS_PATH)/STM32F1xx_HAL_Driver/Inc \
	-I$(STM32_DRIVERS_PATH)/STM32F1xx_HAL_Driver/Inc/Legacy \
	-I$(STM32_DRIVERS_PATH)/CMSIS/Device/ST/STM32F1xx/Include \
	-I$(STM32_DRIVERS_PATH)/CMSIS/Include \
	-Ilib/dwt_delay \
	-Ilib/eeprom \
	-Isrc \
	-Isrc/mtbbus

ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -pedantic -std=c99 -fdata-sections -ffunction-sections
CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -pedantic -std=c99 -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

# link script
LDSCRIPT = STM32F103C8Tx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASMM_SOURCES:.S=.o)))
vpath %.S $(sort $(dir $(ASMM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@
$(BUILD_DIR)/%.o: %.S Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

clean:
	-rm -fR $(BUILD_DIR)

flash_stlink:
	st-flash --reset write $(BUILD_DIR)/$(TARGET).bin 0x08000000

-include $(wildcard $(BUILD_DIR)/*.d)

.PHONY: clean flash_stlink all
