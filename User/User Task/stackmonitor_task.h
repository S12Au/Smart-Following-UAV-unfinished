#ifndef STACKMONITOR_TASK_H
#define STACKMONITOR_TASK_H

#include "task.h"
#include "cmsis_os.h"

extern TaskHandle_t g_getGyroAccelTaskHandle;
extern TaskHandle_t g_getMagTaskHandle;
extern TaskHandle_t g_getPressureTaskHandle;
extern TaskHandle_t g_uartDebugTaskHandle;
extern TaskHandle_t g_getPpmTaskHandle;
extern TaskHandle_t g_flightControlTaskHandle;
extern TaskHandle_t g_stackMonitorTaskHandle;

extern osThreadId_t defaultTaskHandle;

void StackMonitor_Task(void* params);

#endif
