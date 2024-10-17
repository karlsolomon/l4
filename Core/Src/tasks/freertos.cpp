/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <ranges>
#include <utility>
#include <vector>

#include "cmsis_os.h"
#include "main.h"
#include "pins.h"
#include "stm32l4xx_hal_gpio.h"
#include "task.h"
#include "taskUtils.h"

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[128];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .cb_mem = &defaultTaskControlBlock,
    .cb_size = sizeof(defaultTaskControlBlock),
    .stack_mem = &defaultTaskBuffer[0],
    .stack_size = sizeof(defaultTaskBuffer),
    .priority = (osPriority_t)osPriorityNormal,
};

/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
uint8_t myQueue01Buffer[16 * sizeof(uint16_t)];
osStaticMessageQDef_t myQueue01ControlBlock;
const osMessageQueueAttr_t myQueue01_attributes = {.name = "myQueue01",
                                                   .cb_mem = &myQueue01ControlBlock,
                                                   .cb_size = sizeof(myQueue01ControlBlock),
                                                   .mq_mem = &myQueue01Buffer,
                                                   .mq_size = sizeof(myQueue01Buffer)};
/* Definitions for myQueue02 */
osMessageQueueId_t myQueue02Handle;
uint8_t myQueue02Buffer[16 * sizeof(uint16_t)];
osStaticMessageQDef_t myQueue02ControlBlock;
const osMessageQueueAttr_t myQueue02_attributes = {.name = "myQueue02",
                                                   .cb_mem = &myQueue02ControlBlock,
                                                   .cb_size = sizeof(myQueue02ControlBlock),
                                                   .mq_mem = &myQueue02Buffer,
                                                   .mq_size = sizeof(myQueue02Buffer)};
/* Definitions for myTimer01 */
osTimerId_t myTimer01Handle;
osStaticTimerDef_t myTimer01ControlBlock;
const osTimerAttr_t myTimer01_attributes = {
    .name = "myTimer01",
    .cb_mem = &myTimer01ControlBlock,
    .cb_size = sizeof(myTimer01ControlBlock),
};
/* Definitions for myTimer02 */
osTimerId_t myTimer02Handle;
osStaticTimerDef_t myTimer02ControlBlock;
const osTimerAttr_t myTimer02_attributes = {
    .name = "myTimer02",
    .cb_mem = &myTimer02ControlBlock,
    .cb_size = sizeof(myTimer02ControlBlock),
};
/* Definitions for myMutex01 */
osMutexId_t myMutex01Handle;
osStaticMutexDef_t myMutex01ControlBlock;
const osMutexAttr_t myMutex01_attributes = {
    .name = "myMutex01",
    .cb_mem = &myMutex01ControlBlock,
    .cb_size = sizeof(myMutex01ControlBlock),
};
/* Definitions for myMutex02 */
osMutexId_t myMutex02Handle;
osStaticMutexDef_t myMutex02ControlBlock;
const osMutexAttr_t myMutex02_attributes = {
    .name = "myMutex02",
    .cb_mem = &myMutex02ControlBlock,
    .cb_size = sizeof(myMutex02ControlBlock),
};
/* Definitions for myRecursiveMutex01 */
osMutexId_t myRecursiveMutex01Handle;
osStaticMutexDef_t myRecursiveMutex01ControlBlock;
const osMutexAttr_t myRecursiveMutex01_attributes = {
    .name = "myRecursiveMutex01",
    .attr_bits = osMutexRecursive,
    .cb_mem = &myRecursiveMutex01ControlBlock,
    .cb_size = sizeof(myRecursiveMutex01ControlBlock),
};
/* Definitions for myRecursiveMutex02 */
osMutexId_t myRecursiveMutex02Handle;
osStaticMutexDef_t myRecursiveMutex02ControlBlock;
const osMutexAttr_t myRecursiveMutex02_attributes = {
    .name = "myRecursiveMutex02",
    .attr_bits = osMutexRecursive,
    .cb_mem = &myRecursiveMutex02ControlBlock,
    .cb_size = sizeof(myRecursiveMutex02ControlBlock),
};
/* Definitions for myBinarySem01 */
osSemaphoreId_t myBinarySem01Handle;
osStaticSemaphoreDef_t myBinarySem01ControlBlock;
const osSemaphoreAttr_t myBinarySem01_attributes = {
    .name = "myBinarySem01",
    .cb_mem = &myBinarySem01ControlBlock,
    .cb_size = sizeof(myBinarySem01ControlBlock),
};
/* Definitions for myBinarySem02 */
osSemaphoreId_t myBinarySem02Handle;
osStaticSemaphoreDef_t myBinarySem02ControlBlock;
const osSemaphoreAttr_t myBinarySem02_attributes = {
    .name = "myBinarySem02",
    .cb_mem = &myBinarySem02ControlBlock,
    .cb_size = sizeof(myBinarySem02ControlBlock),
};
/* Definitions for myCountingSem01 */
osSemaphoreId_t myCountingSem01Handle;
osStaticSemaphoreDef_t myCountingSem01ControlBlock;
const osSemaphoreAttr_t myCountingSem01_attributes = {
    .name = "myCountingSem01",
    .cb_mem = &myCountingSem01ControlBlock,
    .cb_size = sizeof(myCountingSem01ControlBlock),
};
/* Definitions for myCountingSem02 */
osSemaphoreId_t myCountingSem02Handle;
osStaticSemaphoreDef_t myCountingSem02ControlBlock;
const osSemaphoreAttr_t myCountingSem02_attributes = {
    .name = "myCountingSem02",
    .cb_mem = &myCountingSem02ControlBlock,
    .cb_size = sizeof(myCountingSem02ControlBlock),
};
/* Definitions for myEvent01 */
osEventFlagsId_t myEvent01Handle;
osStaticEventGroupDef_t myEvent01ControlBlock;
const osEventFlagsAttr_t myEvent01_attributes = {
    .name = "myEvent01",
    .cb_mem = &myEvent01ControlBlock,
    .cb_size = sizeof(myEvent01ControlBlock),
};
/* Definitions for myEvent02 */
osEventFlagsId_t myEvent02Handle;
osStaticEventGroupDef_t myEvent02ControlBlock;
const osEventFlagsAttr_t myEvent02_attributes = {
    .name = "myEvent02",
    .cb_mem = &myEvent02ControlBlock,
    .cb_size = sizeof(myEvent02ControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Callback01(void *argument);
void Callback02(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationTickHook(void);
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);
void vApplicationDaemonTaskStartupHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void) {}

__weak unsigned long getRunTimeCounterValue(void) { return 0; }
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
void vApplicationIdleHook(void) {
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
    to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
    task. It is essential that code added to this hook function never attempts
    to block in any way (for example, call xQueueReceive() with a block time
    specified, or call vTaskDelay()). If the application makes use of the
    vTaskDelete() API function (as this demo application does) then it is also
    important that vApplicationIdleHook() is permitted to return to its calling
    function, because it is the responsibility of the idle task to clean up
    memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
void vApplicationTickHook(void) {
    /* This function will be called by each tick interrupt if
    configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
    added here, but the tick hook is called from an interrupt context, so
    code must not attempt to block, and only the interrupt safe FreeRTOS API
    functions can be used (those that end in FromISR()). */
}
/* USER CODE END 3 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName) {
    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
    called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void) {
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created. It is also called by various parts of the
    demo application. If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/* USER CODE BEGIN DAEMON_TASK_STARTUP_HOOK */
void vApplicationDaemonTaskStartupHook(void) {}
/* USER CODE END DAEMON_TASK_STARTUP_HOOK */

/* USER CODE BEGIN PREPOSTSLEEP */
__weak void PreSleepProcessing(uint32_t ulExpectedIdleTime) { /* place for user code */ }

__weak void PostSleepProcessing(uint32_t ulExpectedIdleTime) { /* place for user code */ }
/* USER CODE END PREPOSTSLEEP */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */
    /* Create the mutex(es) */
    /* creation of myMutex01 */
    myMutex01Handle = osMutexNew(&myMutex01_attributes);

    /* creation of myMutex02 */
    myMutex02Handle = osMutexNew(&myMutex02_attributes);

    /* Create the recursive mutex(es) */
    /* creation of myRecursiveMutex01 */
    myRecursiveMutex01Handle = osMutexNew(&myRecursiveMutex01_attributes);

    /* creation of myRecursiveMutex02 */
    myRecursiveMutex02Handle = osMutexNew(&myRecursiveMutex02_attributes);

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* Create the semaphores(s) */
    /* creation of myBinarySem01 */
    myBinarySem01Handle = osSemaphoreNew(1, 1, &myBinarySem01_attributes);

    /* creation of myBinarySem02 */
    myBinarySem02Handle = osSemaphoreNew(1, 1, &myBinarySem02_attributes);

    /* creation of myCountingSem01 */
    myCountingSem01Handle = osSemaphoreNew(2, 0, &myCountingSem01_attributes);

    /* creation of myCountingSem02 */
    myCountingSem02Handle = osSemaphoreNew(2, 0, &myCountingSem02_attributes);

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* Create the timer(s) */
    /* creation of myTimer01 */
    myTimer01Handle = osTimerNew(Callback01, osTimerPeriodic, NULL, &myTimer01_attributes);

    /* creation of myTimer02 */
    myTimer02Handle = osTimerNew(Callback02, osTimerPeriodic, NULL, &myTimer02_attributes);

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* creation of myQueue01 */
    myQueue01Handle = osMessageQueueNew(16, sizeof(uint16_t), &myQueue01_attributes);

    /* creation of myQueue02 */
    myQueue02Handle = osMessageQueueNew(16, sizeof(uint16_t), &myQueue02_attributes);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* creation of myEvent01 */
    myEvent01Handle = osEventFlagsNew(&myEvent01_attributes);

    /* creation of myEvent02 */
    myEvent02Handle = osEventFlagsNew(&myEvent02_attributes);

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
    /* init code for USB_DEVICE */
    /*MX_USB_DEVICE_Init();*/
    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}

/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
void HeartbeatTask(void *argument) {
    /* Add your application code here */
    LL_GPIO_InitTypeDef btn = {.Pin = LL_GPIO_PIN_13,
                               .Mode = LL_GPIO_MODE_INPUT,
                               .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
                               .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
                               .Pull = LL_GPIO_PULL_NO,
                               .Alternate = LL_GPIO_AF_0};
    LL_GPIO_InitTypeDef led1 = {.Pin = LD1_PIN,
                                .Mode = LL_GPIO_MODE_OUTPUT,
                                .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
                                .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
                                .Pull = LL_GPIO_PULL_NO,
                                .Alternate = LL_GPIO_AF_0};
    LL_GPIO_InitTypeDef led2 = {.Pin = LD2_PIN,
                                .Mode = LL_GPIO_MODE_OUTPUT,
                                .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
                                .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
                                .Pull = LL_GPIO_PULL_NO,
                                .Alternate = LL_GPIO_AF_0};
    LL_GPIO_InitTypeDef led3 = {.Pin = LD3_PIN,
                                .Mode = LL_GPIO_MODE_OUTPUT,
                                .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
                                .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
                                .Pull = LL_GPIO_PULL_NO,
                                .Alternate = LL_GPIO_AF_0};

    std::vector<std::pair<GPIO_TypeDef *, LL_GPIO_InitTypeDef>> pins = {
        {GPIOC, btn}, {LD1_PORT, led1}, {LD2_PORT, led2}, {LD3_PORT, led3}};

    std::vector<std::pair<GPIO_TypeDef *, LL_GPIO_InitTypeDef>> leds = {
        std::vector<std::pair<GPIO_TypeDef *, LL_GPIO_InitTypeDef>>(pins.begin() + 1, pins.end()),
    };

    // Initialize pins
    for (auto [port, pin] : pins) {
        LL_GPIO_Init(port, &pin);
    }

    for (auto [port, pin] : leds) {
        LL_GPIO_SetOutputPin(port, pin.Pin);
    }

    while (true) {
        for (auto [port, pin] : leds) {
            LL_GPIO_TogglePin(port, pin.Pin);
            LL_mDelay(50);
        }

        for (auto [port, pin] : std::ranges::reverse_view(leds)) {
            LL_GPIO_TogglePin(port, pin.Pin);
            LL_mDelay(50);
        }
    }
}

/* Callback01 function */
void Callback01(void *argument) {
    /* USER CODE BEGIN Callback01 */

    /* USER CODE END Callback01 */
}

/* Callback02 function */
void Callback02(void *argument) {
    /* USER CODE BEGIN Callback02 */

    /* USER CODE END Callback02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
