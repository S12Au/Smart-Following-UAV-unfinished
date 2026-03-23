#include "freertos.h"
#include "queue.h"
#ifndef GETPPM_TASK_H
#define GETPPM_TASK_H

#define PPM_CHANNEL_COUNT 8u

extern QueueHandle_t QueuePPM;

struct PPM_Data{
	uint16_t ppmCh[PPM_CHANNEL_COUNT];
};
extern void getPPM_Task();

#endif
