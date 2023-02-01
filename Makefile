# MAKEFILE FOR SURGE-FLIGHT
# IMPORTANT: have the stm32f4 plugged in when running target all, as running make all will try to upload to the board
#			 A lot of these paths are hard-coded, change them upon moving the project (or just update the file to do it dynamically)
# TODO: Add a check for the board being plugged in? 

# STORE THE PATH OF THE BOARD STARTUP FILES, CMSIS INCLUDES AND LINKER SCRIPT
ROOT_DIR := $(dir $(realpath $(lastword $(MAKEFILE_LIST))))

INCLUDE_CMSIS_CORE :=-I${ROOT_DIR}arm_essentials/cmsisCore
INCLUDE_STMLIB:=-I${ROOT_DIR}arm_essentials/standardLib
INCLUDE_STM32F4_BASE:=-I${ROOT_DIR}arm_essentials/STM32F411includes
ALL_INCLUDES=$(INCLUDE_CMSIS_CORE) $(INCLUDE_STM32F4_BASE)
LINKER_SCRIPT:=-T${ROOT_DIR}arm_essentials/linkerFile/STM32F411RE_FLASH.ld

# SET UP COMPILER FLAGS
TARGET_PROCESSOR=-DSTM32F411xE
OPTIMIZE_FOR_SIZE=-Os
GARBAGE_COLLECT_UNUSED_INPUT_SECTIONS=-Wl,--gc-sections
CFLAGS=-Wall -mcpu=cortex-m4 -mlittle-endian -mthumb -mfloat-abi=soft #-mfloat-abi=hard
CFLAGS+=$(OPTIMIZE_FOR_SIZE)
CFLAGS+=$(GARBAGE_COLLECT_UNUSED_INPUT_SECTIONS)
CLAGS+=$(TARGET_PROCESSOR)

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

# STORE THE PATH OF THE FILES FOR THE FLIGHT CONTROLLER
FOLDER_ARM_DRIVERS=arm_drivers/
FOLDER_ARM_ESSENTIALS=arm_essentials/
FOLDER_FLIGHT_CONTROLLER=flight_controller/
FOLDER_HARDWARE_DRIVERS=hardware_drivers/
FOLDER_TEST_UTILS=test_utils/

ARM_DRIVERS_SOURCES := $(wildcard $(FOLDER_ARM_DRIVERS)*.c)
ARM_DRIVERS_OFILES := $(patsubst %.c,%.o,$(ARM_DRIVERS_SOURCES))
FLIGHT_CONTROLLER_SOURCES := $(wildcard $(FOLDER_FLIGHT_CONTROLLER)*.c)
FLIGHT_CONTROLLER_OFILES := $(patsubst %.c,%.o,$(FLIGHT_CONTROLLER_SOURCES))
HARDWARE_DRIVERS_SOURCES := $(wildcard $(FOLDER_HARDWARE_DRIVERS)*.c)
HARDWARE_DRIVERS_OFILES := $(patsubst %.c,%.o,$(HARDWARE_DRIVERS_SOURCES))
TEST_UTILS_SOURCES := $(wildcard $(FOLDER_TEST_UTILS)*.c)
TEST_UTILS_OFILES := $(patsubst %.c,%.o,$(TEST_UTILS_SOURCES))


all: COMPILE LINK CREATE_HEX UPLOAD_FILE


# COMPILATION TARGETS: This is such a hacky solution hahaha but it works for now! 
COMPILE: $(ARM_DRIVERS_OFILES) $(FLIGHT_CONTROLLER_OFILES) $(HARDWARE_DRIVERS_OFILES) $(TEST_UTILS_OFILES) main startup_c startup_asm #syscalls
	mv *.o build/
	rm main.h.gch

PRINT:
	$(info  ROOT_DIR is  $(value ROOT_DIR) CMSIS code: $(value INCLUDE_CMSIS_CORE))

# NOTE: is not needed if you select the correct linker flags
# 		(use the linker flag --specs=rdimon.specs to avoid it)
syscalls: syscalls.c
	$(CC) $(CFLAGS) $(ALL_INCLUDES) -c syscalls.c -o syscalls.o

main: main.c main.h #$(SRC)
	$(CC) $(CFLAGS) $(ALL_INCLUDES) -c main.c main.h #-o main.o

startup_c: system_stm32f4xx.c #$(STARTUP_FILE_C)
	$(CC) $(CFLAGS) $(ALL_INCLUDES) -c system_stm32f4xx.c -o system_stm32f4xx.o

startup_asm: startup_stm32f411xe.s #$(STARTUP_FILE_ASM)
	$(CC) $(CFLAGS) $(ALL_INCLUDES) -c startup_stm32f411xe.s -o startup_stm32f411xe.o


# COMPILATION RULE FOR .o files
%.o: %.c %.h
	$(CC) $(CFLAGS) $(ALL_INCLUDES) -c $<


# LINK TARGETS: step requires all .o files in the /build folder 
LINK:
	# --specs=rdimon.specs : semihosting support
	# --specs=nosys.specs
	$(CC) --specs=rdimon.specs $(CFLAGS) $(LINKER_SCRIPT) $(wildcard build/*.o) -o build/main.elf


# .elf -> .hex (the stm32f411 board file format)
CREATE_HEX:
	arm-none-eabi-objcopy -Oihex build/main.elf build/main.hex


# USE OPENOCD TO PROGRAM THE .hex ONTO THE BOARD
UPLOAD_FILE:
	openocd -f /usr/share/openocd/scripts/board/stm32f4discovery.cfg -c "program build/main.hex verify reset" -c "shutdown"


# OTHER TARGETS
clean:
	rm build/*.o build/*.hex build/*.elf



