#include "FreeRTOS.h"
#include "stackmonitor_task.h"
#include "task.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define STACK_MONITOR_PERIOD_MS        1000u
#define STACK_MONITOR_SNAPSHOT_DIV     5u
#define STACK_WATERMARK_ALERT_WORDS    32u

static void reportTaskStackWatermark(const char* name, TaskHandle_t handle, uint8_t printSnapshot)
{
    UBaseType_t wm;

    if (name == NULL || handle == NULL)
    {
        return;
    }

    wm = uxTaskGetStackHighWaterMark(handle);
    if (printSnapshot != 0u)
    {
        printf("STACK,%s,%lu\r\n", name, (unsigned long)wm);
    }

    if (wm <= (UBaseType_t)STACK_WATERMARK_ALERT_WORDS)
    {
        printf("STACK,ALERT,%s,wm=%lu\r\n", name, (unsigned long)wm);
    }
}

void StackMonitor_Task(void* params)
{
    uint8_t snapshotDivider = 0u;

    (void)params;

    for (;;)
    {
        uint8_t printSnapshot = 0u;
        TaskHandle_t defaultTask = (TaskHandle_t)defaultTaskHandle;

        snapshotDivider++;
        if (snapshotDivider >= STACK_MONITOR_SNAPSHOT_DIV)
        {
            snapshotDivider = 0u;
            printSnapshot = 1u;
            printf("STACK,BEGIN\r\n");
        }

        reportTaskStackWatermark("gyroaccel", g_getGyroAccelTaskHandle, printSnapshot);
        reportTaskStackWatermark("mag", g_getMagTaskHandle, printSnapshot);
        reportTaskStackWatermark("pressure", g_getPressureTaskHandle, printSnapshot);
        reportTaskStackWatermark("uartdbg", g_uartDebugTaskHandle, printSnapshot);
        reportTaskStackWatermark("ppm", g_getPpmTaskHandle, printSnapshot);
        reportTaskStackWatermark("flightctl", g_flightControlTaskHandle, printSnapshot);
        reportTaskStackWatermark("stackmon", g_stackMonitorTaskHandle, printSnapshot);
        reportTaskStackWatermark("default", defaultTask, printSnapshot);

        if (printSnapshot != 0u)
        {
            printf("STACK,END\r\n");
        }

        vTaskDelay(pdMS_TO_TICKS(STACK_MONITOR_PERIOD_MS));
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    const char* name = (pcTaskName != NULL) ? pcTaskName : "unknown";
    (void)xTask;

    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\nSTACK,OVERFLOW,task=", 23u, 100u);
    HAL_UART_Transmit(&huart1, (uint8_t*)name, (uint16_t)strlen(name), 100u);
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2u, 100u);

    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}
