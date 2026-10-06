##########################################################################################################################
# SmartESC STM32 V3 / M365
##########################################################################################################################

DO_NOT_USE_M365_BOOTLOADER = 0

######################################
# target
######################################

TARGET = EBiCS_Firmware2

######################################
# building variables
######################################

DEBUG = 1
OPT = -Og

#######################################
# paths
#######################################

BUILD_DIR = build

######################################
# source
######################################

C_SOURCES = \
Core/Src/main.c \
Core/Src/motor.c \
Core/Src/FOC.c \
Core/Src/eeprom.c \
Core/Src/decr_and_flash.c \
Core/Src/button_processing.c \
Core/Src/M365_Dashboard.c \
Core/Src/stm32f1xx_it.c \
Core/Src/print.c \
Core/Src/stm32f1xx_hal_msp.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c \
Core/Src/system_stm32f1xx.c

ASM_SOURCES = \
Core/Startup/startup_stm32f103c8tx.s

#######################################
# binaries
#######################################

PREFIX = arm-none-eabi-

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

#######################################
# CPU
#######################################

CPU = -mcpu=cortex-m3

MCU = $(CPU) -mthumb

#######################################
# defines
#######################################

AS_DEFS =

C_DEFS = \
-DUSE_HAL_DRIVER \
-DSTM32F103xB \
-DARM_MATH_CM3

#######################################
# includes
#######################################

AS_INCLUDES =

C_INCLUDES = \
-ICore/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
-IDrivers/CMSIS/Include

#######################################
# compiler flags
#######################################

ASFLAGS = \
$(MCU) \
$(AS_DEFS) \
$(AS_INCLUDES) \
$(OPT) \
-Wall \
-fdata-sections \
-ffunction-sections

CFLAGS = \
$(MCU) \
$(C_DEFS) \
$(C_INCLUDES) \
$(OPT) \
-Wall \
-fdata-sections \
-ffunction-sections

ifeq ($(DEBUG),1)
CFLAGS += -g -gdwarf-2
endif

ifeq ($(DO_NOT_USE_M365_BOOTLOADER),1)
CFLAGS += -DO_NOT_USE_M365_BOOTLOADER
endif

CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

#######################################
# linker
#######################################

LDSCRIPT = STM32F103C8Tx_FLASH-development.ld

LIBS = \
-lc \
-lm \
-lnosys \
-larm_cortexM3l_math

LIBDIR = -LDrivers/CMSIS

LDFLAGS = \
$(MCU) \
-specs=nano.specs \
-T$(LDSCRIPT) \
$(LIBDIR) \
$(LIBS) \
-Wl,-Map=$(BUILD_DIR)/$(TARGET).map \
-Wl,--cref \
-Wl,--gc-sections

#######################################
# objects
#######################################

OBJECTS = \
$(BUILD_DIR)/main.o \
$(BUILD_DIR)/motor.o \
$(BUILD_DIR)/FOC.o \
$(BUILD_DIR)/eeprom.o \
$(BUILD_DIR)/decr_and_flash.o \
$(BUILD_DIR)/button_processing.o \
$(BUILD_DIR)/M365_Dashboard.o \
$(BUILD_DIR)/stm32f1xx_it.o \
$(BUILD_DIR)/print.o \
$(BUILD_DIR)/stm32f1xx_hal_msp.o \
$(BUILD_DIR)/stm32f1xx_hal_gpio_ex.o \
$(BUILD_DIR)/stm32f1xx_hal_adc.o \
$(BUILD_DIR)/stm32f1xx_hal_adc_ex.o \
$(BUILD_DIR)/stm32f1xx_hal.o \
$(BUILD_DIR)/stm32f1xx_hal_rcc.o \
$(BUILD_DIR)/stm32f1xx_hal_rcc_ex.o \
$(BUILD_DIR)/stm32f1xx_hal_gpio.o \
$(BUILD_DIR)/stm32f1xx_hal_dma.o \
$(BUILD_DIR)/stm32f1xx_hal_cortex.o \
$(BUILD_DIR)/stm32f1xx_hal_pwr.o \
$(BUILD_DIR)/stm32f1xx_hal_flash.o \
$(BUILD_DIR)/stm32f1xx_hal_flash_ex.o \
$(BUILD_DIR)/stm32f1xx_hal_exti.o \
$(BUILD_DIR)/stm32f1xx_hal_tim.o \
$(BUILD_DIR)/stm32f1xx_hal_tim_ex.o \
$(BUILD_DIR)/stm32f1xx_hal_uart.o \
$(BUILD_DIR)/system_stm32f1xx.o \
$(BUILD_DIR)/startup_stm32f103c8tx.o

#######################################
# default target
#######################################

all: clean $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#######################################
# C compilation
#######################################

$(BUILD_DIR)/main.o: Core/Src/main.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/motor.o: Core/Src/motor.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/FOC.o: Core/Src/FOC.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/eeprom.o: Core/Src/eeprom.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/decr_and_flash.o: Core/Src/decr_and_flash.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/button_processing.o: Core/Src/button_processing.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/M365_Dashboard.o: Core/Src/M365_Dashboard.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32f1xx_it.o: Core/Src/stm32f1xx_it.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/print.o: Core/Src/print.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32f1xx_hal_msp.o: Core/Src/stm32f1xx_hal_msp.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32f1xx_hal_gpio_ex.o: Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32f1xx_hal_adc.o: Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32f1xx_hal_adc_ex.o: Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32f1xx_hal.o: Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32f1xx_hal_rcc.o: Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32f1xx_hal_rcc_ex.o: Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32f1xx_hal_gpio.o: Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32f1xx_hal_dma.o: Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32f1xx_hal_cortex.o: Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32f1xx_hal_pwr.o: Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32f1xx_hal_flash.o: Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32f1xx_hal_flash_ex.o: Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32f1xx_hal_exti.o: Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32f1xx_hal_tim.o: Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32f1xx_hal_tim_ex.o: Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/stm32f1xx_hal_uart.o: Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/system_stm32f1xx.o: Core/Src/system_stm32f1xx.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

#######################################
# ASM
#######################################

$(BUILD_DIR)/startup_stm32f103c8tx.o: Core/Startup/startup_stm32f103c8tx.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

#######################################
# link
#######################################

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

#######################################
# HEX
#######################################

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

#######################################
# BIN
#######################################

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

#######################################
# build directory
#######################################

$(BUILD_DIR):
	mkdir -p $@

#######################################
# clean
#######################################

clean:
	rm -rf $(BUILD_DIR)

#######################################
# dependencies
#######################################

-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
