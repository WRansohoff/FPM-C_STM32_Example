TARGET = main

# Default target chip.
#MCU ?= STM32F030K6
#MCU ?= STM32F031K6
#MCU ?= STM32F103C8
#MCU ?= STM32L031K6
#MCU ?= STM32L052K8
#MCU ?= STM32L082KZ
MCU ?= STM32L432KC

ifeq ($(MCU), STM32F030K6)
	MCU_FILES = STM32F030K6T6
	ST_MCU_DEF = STM32F030x6
	MCU_CLASS = F0
	MCU_LINE = STM32F0
else ifeq ($(MCU), STM32F031K6)
	MCU_FILES = STM32F031K6T6
	ST_MCU_DEF = STM32F031x6
	MCU_CLASS = F0
	MCU_LINE = STM32F0
else ifeq ($(MCU), STM32F103C8)
	MCU_FILES  = STM32F103C8T6
	ST_MCU_DEF = STM32F103xB
	MCU_CLASS  = F1
	MCU_LINE = STM32F1
else ifeq ($(MCU), STM32L031K6)
	MCU_FILES = STM32L031K6T6
	ST_MCU_DEF = STM32L031xx
	MCU_CLASS = L0
	MCU_LINE = STM32L0
else ifeq ($(MCU), STM32L052K8)
	MCU_FILES = STM32L052K8T6
	ST_MCU_DEF = STM32L052xx
	MCU_CLASS = L0
	MCU_LINE = STM32L0
else ifeq ($(MCU), STM32L082KZ)
	MCU_FILES = STM32L082KZT6
	ST_MCU_DEF = STM32L082xx
	MCU_CLASS = L0
	MCU_LINE = STM32L0
else ifeq ($(MCU), STM32L432KC)
	MCU_FILES = STM32L432KCT6
	ST_MCU_DEF = STM32L432xx
	MCU_CLASS = L4
	MCU_LINE = STM32L4
endif

# Define the linker script location and chip architecture.
LD_SCRIPT = $(MCU_FILES).ld
ifeq ($(MCU_CLASS), F0)
	MCU_SPEC = cortex-m0
else ifeq ($(MCU_CLASS), F1)
	MCU_SPEC = cortex-m3
else ifeq ($(MCU_CLASS), L0)
	MCU_SPEC = cortex-m0plus
else ifeq ($(MCU_CLASS), L4)
	MCU_SPEC = cortex-m4
endif

# Toolchain definitions (ARM bare metal defaults)
TOOLCHAIN = /usr
CC = $(TOOLCHAIN)/bin/arm-none-eabi-gcc
AS = $(TOOLCHAIN)/bin/arm-none-eabi-as
LD = $(TOOLCHAIN)/bin/arm-none-eabi-ld
OC = $(TOOLCHAIN)/bin/arm-none-eabi-objcopy
OD = $(TOOLCHAIN)/bin/arm-none-eabi-objdump
OS = $(TOOLCHAIN)/bin/arm-none-eabi-size

# Assembly directives.
ASFLAGS += -c
ASFLAGS += -O0
ASFLAGS += -mcpu=$(MCU_SPEC)
ASFLAGS += -mthumb
ASFLAGS += -Wall
# (Set error messages to appear on a single line.)
ASFLAGS += -fmessage-length=0
ASFLAGS += -DVVC_$(MCU_CLASS)

# C compilation directives
CFLAGS += -mcpu=$(MCU_SPEC)
CFLAGS += -mthumb
ifeq ($(MCU_CLASS), L4)
	CFLAGS += -mhard-float
	CFLAGS += -mfloat-abi=hard
	CFLAGS += -mfpu=fpv4-sp-d16
else
	CFLAGS += -msoft-float
	CFLAGS += -mfloat-abi=soft
endif
CFLAGS += -Wall
CFLAGS += -g
# (Set error messages to appear on a single line.)
CFLAGS += -fmessage-length=0
# (Set system to ignore semihosted junk)
CFLAGS += --specs=nosys.specs
CFLAGS += -D$(ST_MCU_DEF)
CFLAGS += -D$(MCU_LINE)
CFLAGS += -DVVC_$(MCU_CLASS)

# Linker directives.
LSCRIPT = ./ld/$(LD_SCRIPT)
LFLAGS += -mcpu=$(MCU_SPEC)
LFLAGS += -mthumb
ifeq ($(MCU_CLASS), L4)
	LFLAGS += -mhard-float
	LFLAGS += -mfloat-abi=hard
	LFLAGS += -mfpu=fpv4-sp-d16
else
	LFLAGS += -msoft-float
	LFLAGS += -mfloat-abi=soft
endif
LFLAGS += -Wall
LFLAGS += --specs=nosys.specs
LFLAGS += -nostdlib
LFLAGS += -lm -lc -lgcc
LFLAGS += -T$(LSCRIPT)

AS_SRC   =  ./boot_code/$(MCU_FILES)_core.S
AS_SRC   += ./vector_tables/$(MCU_FILES)_vt.S
C_SRC    =  ./src/main.c
C_SRC    += ./src/util.c
C_SRC    += ./src/interrupts.c
C_SRC    += ./src/uart.c
C_SRC    += ./lib/fpm.c
# TODO: Other chip types.
C_SRC    += ./hal/L4/system_stm32l4xx.c
C_SRC    += ./hal/L4/Src/stm32l4xx_hal.c
C_SRC    += ./hal/L4/Src/stm32l4xx_hal_cortex.c
C_SRC    += ./hal/L4/Src/stm32l4xx_hal_gpio.c
C_SRC    += ./hal/L4/Src/stm32l4xx_hal_rcc.c
C_SRC    += ./hal/L4/Src/stm32l4xx_hal_pwr.c
C_SRC    += ./hal/L4/Src/stm32l4xx_hal_pwr_ex.c

INCLUDE  =  -I./
INCLUDE  += -I./device_headers
INCLUDE  += -I./lib
# TODO: Other chip types.
INCLUDE  += -I./hal/L4/Inc

OBJS  = $(AS_SRC:.S=.o)
OBJS += $(C_SRC:.c=.o)

.PHONY: all
all: $(TARGET).bin

%.o: %.S
	$(CC) -x assembler-with-cpp $(ASFLAGS) $< -o $@

%.o: %.c
	$(CC) -c $(CFLAGS) $(INCLUDE) $< -o $@

$(TARGET).elf: $(OBJS)
	$(CC) $^ $(LFLAGS) -o $@

$(TARGET).bin: $(TARGET).elf
	$(OC) -S -O binary $< $@
	$(OS) $<

.PHONY: clean
clean:
	rm -f $(OBJS)
	rm -f $(TARGET).elf
	rm -f $(TARGET).bin
