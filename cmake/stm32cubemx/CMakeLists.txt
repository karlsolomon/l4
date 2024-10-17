cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(CXX C ASM)

target_compile_definitions(
    stm32cubemx
    INTERFACE USE_FULL_LL_DRIVER
              USE_HAL_DRIVER
              STM32L4A6xx
              $<$<CONFIG:Debug>:DEBUG>
)

set(cube "/home/ksolomon/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.1")
set(cubeDrivers "${cube}/Drivers")
set(hal "${cubeDrivers}/STM32L4xx_HAL_Driver")
set(fatfs "${cube}/Middlewares/Third_Party/FatFs")
set(freeRTOS "${cube}/Middlewares/Third_Party/FreeRTOS")
set(app "../../Core/")
set(userDrivers "${app}/Src/drivers")
set(userTasks "${app}/Src/tasks")
set(usb "${cube}/Middlewares/ST/STM32_USB_Device_Library")

target_include_directories(
    stm32cubemx
    INTERFACE ${app}/Inc
              ../../FATFS/Target
              ../../FATFS/App
              ../../USB_DEVICE/App
              ../../USB_DEVICE/Target
              ${hal}/Inc
              ${hal}/Inc/Legacy
              ${freeRTOS}/Source/include
              ${freeRTOS}/Source/CMSIS_RTOS_V2
              ${freeRTOS}/Source/portable/GCC/ARM_CM4F
              ${fatfs}/src
              ${usb}/Core/Inc
              ${usb}/Class/MSC/Inc
              ${cubeDrivers}/CMSIS/Device/ST/STM32L4xx/Include
              ${cubeDrivers}/CMSIS/Include
)

target_sources(
    stm32cubemx
    INTERFACE ${app}/Src/main.cpp
              ${userDrivers}/com.cpp
              ${userDrivers}/gpio.c
              ${userTasks}/freertos.cpp
              ${userTasks}/heartbeat.cpp
              ${userDrivers}/aes.c
              ${userDrivers}/hash.c
              ${userDrivers}/i2c.c
              ${userDrivers}/iwdg.c
              ${userDrivers}/lptim.c
              ${userDrivers}/usart.c
              ${userDrivers}/quadspi.c
              ${userDrivers}/rng.c
              ${userDrivers}/spi.c
              ${userDrivers}/tim.c
              ${app}/Src/stm32l4xx_it.c
              ${app}/Src/stm32l4xx_hal_msp.c
              ../../FATFS/App/fatfs.c
              ../../FATFS/Target/user_diskio.c
              ../../USB_DEVICE/App/usb_device.c
              ../../USB_DEVICE/App/usbd_desc.c
              ../../USB_DEVICE/App/usbd_storage_if.c
              ../../USB_DEVICE/Target/usbd_conf.c
              ${hal}/Src/stm32l4xx_ll_utils.c
              ${hal}/Src/stm32l4xx_ll_exti.c
              ${hal}/Src/stm32l4xx_ll_gpio.c
              ${hal}/Src/stm32l4xx_hal_pcd.c
              ${hal}/Src/stm32l4xx_hal_pcd_ex.c
              ${hal}/Src/stm32l4xx_ll_usb.c
              ${hal}/Src/stm32l4xx_hal.c
              ${hal}/Src/stm32l4xx_hal_rcc.c
              ${hal}/Src/stm32l4xx_hal_rcc_ex.c
              ${hal}/Src/stm32l4xx_hal_flash.c
              ${hal}/Src/stm32l4xx_hal_flash_ex.c
              ${hal}/Src/stm32l4xx_hal_flash_ramfunc.c
              ${hal}/Src/stm32l4xx_hal_gpio.c
              ${hal}/Src/stm32l4xx_hal_i2c.c
              ${hal}/Src/stm32l4xx_hal_i2c_ex.c
              ${hal}/Src/stm32l4xx_hal_dma.c
              ${hal}/Src/stm32l4xx_hal_dma_ex.c
              ${hal}/Src/stm32l4xx_hal_pwr.c
              ${hal}/Src/stm32l4xx_hal_pwr_ex.c
              ${hal}/Src/stm32l4xx_hal_cortex.c
              ${hal}/Src/stm32l4xx_hal_exti.c
              ${hal}/Src/stm32l4xx_hal_cryp.c
              ${hal}/Src/stm32l4xx_hal_cryp_ex.c
              ${hal}/Src/stm32l4xx_hal_hash.c
              ${hal}/Src/stm32l4xx_hal_hash_ex.c
              ${hal}/Src/stm32l4xx_ll_i2c.c
              ${hal}/Src/stm32l4xx_ll_dma.c
              ${hal}/Src/stm32l4xx_ll_lptim.c
              ${hal}/Src/stm32l4xx_ll_rcc.c
              ${hal}/Src/stm32l4xx_ll_lpuart.c
              ${hal}/Src/stm32l4xx_ll_usart.c
              ${hal}/Src/stm32l4xx_hal_qspi.c
              ${hal}/Src/stm32l4xx_ll_rng.c
              ${hal}/Src/stm32l4xx_ll_spi.c
              ${hal}/Src/stm32l4xx_hal_tim.c
              ${hal}/Src/stm32l4xx_hal_tim_ex.c
              ${hal}/Src/stm32l4xx_ll_tim.c
              ${app}/Src/system_stm32l4xx.c
              ${fatfs}/src/diskio.c
              ${fatfs}/src/ff.c
              ${fatfs}/src/ff_gen_drv.c
              ${fatfs}/src/option/syscall.c
              ${freeRTOS}/Source/croutine.c
              ${freeRTOS}/Source/event_groups.c
              ${freeRTOS}/Source/list.c
              ${freeRTOS}/Source/queue.c
              ${freeRTOS}/Source/stream_buffer.c
              ${freeRTOS}/Source/tasks.c
              ${freeRTOS}/Source/timers.c
              ${freeRTOS}/Source/CMSIS_RTOS_V2/cmsis_os2.c
              ${freeRTOS}/Source/portable/MemMang/heap_4.c
              ${freeRTOS}/Source/portable/GCC/ARM_CM4F/port.c
              ${usb}/Core/Src/usbd_core.c
              ${usb}/Core/Src/usbd_ctlreq.c
              ${usb}/Core/Src/usbd_ioreq.c
              ${usb}/Class/MSC/Src/usbd_msc.c
              ${usb}/Class/MSC/Src/usbd_msc_bot.c
              ${usb}/Class/MSC/Src/usbd_msc_data.c
              ${usb}/Class/MSC/Src/usbd_msc_scsi.c
              ${app}/Src/sysmem.c
              ${app}/Src/syscalls.c
              ../../startup_stm32l4a6xx.s
)

target_link_directories(stm32cubemx INTERFACE)

target_link_libraries(stm32cubemx INTERFACE)

# Validate that STM32CubeMX code is compatible with C standard
if (CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif ()
