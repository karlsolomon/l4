cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(CXX C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_FULL_LL_DRIVER 
	USE_HAL_DRIVER 
	STM32L4A6xx
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../Core/Inc
    ../../FATFS/Target
    ../../FATFS/App
    ../../USB_DEVICE/App
    ../../USB_DEVICE/Target
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Inc
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/include
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FatFs/src
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Inc
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/CMSIS/Device/ST/STM32L4xx/Include
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/CMSIS/Include
)

target_sources(stm32cubemx INTERFACE
    ../../Core/Src/main.c
    ../../Core/Src/gpio.c
    ../../Core/Src/freertos.c
    ../../Core/Src/aes.c
    ../../Core/Src/hash.c
    ../../Core/Src/i2c.c
    ../../Core/Src/iwdg.c
    ../../Core/Src/lptim.c
    ../../Core/Src/usart.c
    ../../Core/Src/quadspi.c
    ../../Core/Src/rng.c
    ../../Core/Src/spi.c
    ../../Core/Src/tim.c
    ../../Core/Src/stm32l4xx_it.c
    ../../Core/Src/stm32l4xx_hal_msp.c
    ../../FATFS/App/fatfs.c
    ../../FATFS/Target/user_diskio.c
    ../../USB_DEVICE/App/usb_device.c
    ../../USB_DEVICE/App/usbd_desc.c
    ../../USB_DEVICE/App/usbd_storage_if.c
    ../../USB_DEVICE/Target/usbd_conf.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_utils.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_exti.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_gpio.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pcd_ex.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usb.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_exti.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cryp.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cryp_ex.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_hash.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_hash_ex.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_i2c.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_dma.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_lptim.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_rcc.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_lpuart.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_usart.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_qspi.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_rng.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_spi.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_ll_tim.c
    ../../Core/Src/system_stm32l4xx.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FatFs/src/diskio.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FatFs/src/ff.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FatFs/src/ff_gen_drv.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FatFs/src/option/syscall.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/list.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/queue.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/timers.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_bot.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_data.c
    /home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1/Middlewares/ST/STM32_USB_Device_Library/Class/MSC/Src/usbd_msc_scsi.c
    ../../Core/Src/sysmem.c
    ../../Core/Src/syscalls.c
    ../../startup_stm32l4a6xx.s
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


