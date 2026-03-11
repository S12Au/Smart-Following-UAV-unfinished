#include "flightcontrol_task.h"

/* FlightControl.c exports the actual control-loop implementation. */
void FlightControl_TaskImpl(void* params);

void FlightControl_Task(void* params)
{
    FlightControl_TaskImpl(params);
}
