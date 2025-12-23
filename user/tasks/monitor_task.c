#include "string.h"

#include "boards.h"
#include "util.h"
#include "adc.h"

#include "parameters.h"
#include "trace.h"

#include "monitor_task.h"
#include "sensors_task.h"


int monitor_task(void)
{
    static uint32_t monitor_ticks = 0;


    float  temp_f = 0.0f;

    if(IS_PRE_MINUS_MID_OVER_POST(sys_time_ms_get(), monitor_ticks, 10))   //间隔 10ms
    {
        monitor_ticks = sys_time_ms_get();

    }

    return 0;
}









