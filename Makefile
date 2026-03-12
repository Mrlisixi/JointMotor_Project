##########################################################################################################################
# Custom Makefile for JointMotor_Project
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
# ------------------------------------------------

######################################
# target
######################################
TARGET = JointMotor_Project


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES = \
AT32WorkBench/project/src/main.c \
AT32WorkBench/project/src/syscalls.c \
AT32WorkBench/project/src/sysmem.c \
AT32WorkBench/project/src/at32f403a_407_wk_config.c \
AT32WorkBench/project/src/at32f403a_407_int.c \
AT32WorkBench/project/src/wk_system.c \
AT32WorkBench/project/src/wk_debug.c \
AT32WorkBench/project/src/wk_tmr.c \
AT32WorkBench/project/src/wk_gpio.c \
AT32WorkBench/project/src/wk_acc.c \
AT32WorkBench/project/src/wk_adc.c \
AT32WorkBench/project/src/wk_spi.c \
AT32WorkBench/project/src/wk_usbfs.c \
AT32WorkBench/project/src/wk_dma.c \
AT32WorkBench/project/src/wk_can.c \
AT32WorkBench/project/src/usb_app.c \
AT32WorkBench/project/src/freertos_app.c \
AT32WorkBench/project/src/cdc_class.c \
AT32WorkBench/project/src/cdc_desc.c \
AT32WorkBench/libraries/cmsis/cm4/device_support/system_at32f403a_407.c \
AT32WorkBench/libraries/drivers/src/at32f403a_407_crm.c \
AT32WorkBench/libraries/drivers/src/at32f403a_407_tmr.c \
AT32WorkBench/libraries/drivers/src/at32f403a_407_gpio.c \
AT32WorkBench/libraries/drivers/src/at32f403a_407_pwc.c \
AT32WorkBench/libraries/drivers/src/at32f403a_407_adc.c \
AT32WorkBench/libraries/drivers/src/at32f403a_407_spi.c \
AT32WorkBench/libraries/drivers/src/at32f403a_407_dma.c \
AT32WorkBench/libraries/drivers/src/at32f403a_407_flash.c \
AT32WorkBench/libraries/drivers/src/at32f403a_407_exint.c \
AT32WorkBench/libraries/drivers/src/at32f403a_407_misc.c \
AT32WorkBench/libraries/drivers/src/at32f403a_407_usb.c \
AT32WorkBench/libraries/drivers/src/at32f403a_407_acc.c \
AT32WorkBench/libraries/drivers/src/at32f403a_407_can.c \
AT32WorkBench/libraries/drivers/src/at32f403a_407_debug.c \
Motor_Control/src/motor_control.c \
Motor_Control/src/wk_stubs.c \
AT32WorkBench/middlewares/usbd_drivers/src/usbd_core.c \
AT32WorkBench/middlewares/usbd_drivers/src/usbd_int.c \
AT32WorkBench/middlewares/usbd_drivers/src/usbd_sdr.c \
AT32WorkBench/middlewares/freertos/source/croutine.c \
AT32WorkBench/middlewares/freertos/source/event_groups.c \
AT32WorkBench/middlewares/freertos/source/list.c \
AT32WorkBench/middlewares/freertos/source/queue.c \
AT32WorkBench/middlewares/freertos/source/tasks.c \
AT32WorkBench/middlewares/freertos/source/timers.c \
AT32WorkBench/middlewares/freertos/source/portable/GCC/ARM_CM4F/port.c \
AT32WorkBench/middlewares/freertos/source/portable/memmang/heap_4.c \
AT32WorkBench/middlewares/freertos/source/stream_buffer.c \

# ASM sources
ASM_SOURCES =  \
AT32WorkBench/startup_at32f403a_407.s


#######################################
# binaries
#######################################
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
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_STDPERIPH_DRIVER \
-DAT32F403ACGT7


# AS includes
AS_INCLUDES =

# C includes
C_INCLUDES = \
-IAT32WorkBench/project/inc \
-IAT32WorkBench/libraries/drivers/inc \
-IAT32WorkBench/libraries/cmsis/cm4/core_support \
-IAT32WorkBench/libraries/cmsis/cm4/device_support \
-IAT32WorkBench/middlewares/usbd_drivers/inc \
-IMotor_Control/inc \
-IAT32WorkBench/middlewares/freertos/source/include \
-IAT32WorkBench/middlewares/freertos/source/portable/memmang \
-IAT32WorkBench/middlewares/freertos/source/portable/GCC/ARM_CM4F \


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = AT32WorkBench/AT32F403AxG_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
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
		
#######################################
# clean up
#######################################
clean:
	rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***