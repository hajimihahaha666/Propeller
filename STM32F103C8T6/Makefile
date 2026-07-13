TARGET = pai_stm32f103c8t6
BUILD_DIR = build

PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size

MCU = -mcpu=cortex-m3 -mthumb

C_DEFS = -DSTM32F103xB -DUSE_HAL_DRIVER

C_INCLUDES = \
	-ICore/Inc \
	-IMy_Core \
	-IDrivers/STM32F1xx_HAL_Driver/Inc \
	-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
	-IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
	-IDrivers/CMSIS/Include

C_SOURCES = \
	Core/Src/main.c \
	Core/Src/gpio.c \
	Core/Src/dma.c \
	Core/Src/tim.c \
	Core/Src/usart.c \
	Core/Src/stm32f1xx_it.c \
	Core/Src/stm32f1xx_hal_msp.c \
	Core/Src/system_stm32f1xx.c \
	My_Core/App.c \
	My_Core/esc_spi.c \
	My_Core/spi_slave.c \
	My_Core/Ibus.c \
	My_Core/Servo.c \
	My_Core/Step_Motor.c \
	My_Core/UART.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c \
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
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
	Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c

ASM_SOURCES = \
	Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) -Og -g3 -Wall -fdata-sections -ffunction-sections
LDFLAGS = $(MCU) -specs=nano.specs -TSTM32F103C8Tx_FLASH.ld \
	-Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

LIBS = -lc -lm -lnosys

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).hex

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) $(LIBS) -o $@
	$(SZ) $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(CP) -O binary $< $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(CP) -O ihex $< $@

$(BUILD_DIR):
	mkdir -p $@

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean
