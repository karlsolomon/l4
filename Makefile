##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [4.4.0-B60] date: [Wed Oct 16 09:40:45 MST 2024]
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = l4


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
C_SOURCES =  \
Core/Src/main.c \
Core/Src/gpio.c \
Core/Src/freertos.c \
Core/Src/aes.c \
Core/Src/hash.c \
Core/Src/i2c.c \
Core/Src/iwdg.c \
Core/Src/lptim.c \
Core/Src/usart.c \
Core/Src/quadspi.c \
Core/Src/rng.c \
Core/Src/spi.c \
Core/Src/tim.c \
Core/Src/stm32l4xx_it.c \
Core/Src/stm32l4xx_hal_msp.c \
FATFS/App/fatfs.c \
FATFS/Target/user_diskio.c \
USB_DEVICE/App/usb_device.c \
USB_DEVICE/App/usbd_desc.c \
USB_DEVICE/App/usbd_storage_if.c \
USB_DEVICE/Target/usbd_conf.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_utils.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_exti.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_gpio.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd_ex.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cryp.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cryp_ex.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_hash.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_hash_ex.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_i2c.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_dma.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_lptim.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_rcc.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_lpuart.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usart.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_qspi.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_rng.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_spi.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_tim.c \
Core/Src/system_stm32l4xx.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FatFs/src/diskio.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FatFs/src/ff.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FatFs/src/ff_gen_drv.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FatFs/src/option/syscall.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/list.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/queue.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/timers.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_bot.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_data.c \
/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_scsi.c \
Core/Src/sysmem.c \
Core/Src/syscalls.c  

# ASM sources
ASM_SOURCES =  \
startup_stm32l4a6xx.s

# ASM sources
ASMM_SOURCES = 


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
-DUSE_FULL_LL_DRIVER \
-DUSE_HAL_DRIVER \
-DSTM32L4A6xx


# AS includes
AS_INCLUDES =  \
-ICore/Inc

# C includes
C_INCLUDES =  \
-ICore/Inc \
-IFATFS/Target \
-IFATFS/App \
-IUSB_DEVICE/App \
-IUSB_DEVICE/Target \
-I/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Inc \
-I/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy \
-I/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/include \
-I/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
-I/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-I/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FatFs/src \
-I/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/ST/STM32_USB_Device_Library/Core/Inc \
-I/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc \
-I/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include \
-I/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/CMSIS/Include


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
LDSCRIPT = STM32L4A6ZGTx_FLASH.ld

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

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
