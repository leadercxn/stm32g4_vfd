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

        // 硬件启动
        if(gpio_input_get(DSP_X1_STARTUP_PORT, DSP_X1_STARTUP_PIN) == 0)
        {
            SET_BIT64(g_app_param.evt_code, EVT_STARTUP_HW);
        }
        else
        {
            CLR_BIT64(g_app_param.evt_code, EVT_STARTUP_HW);
        }

        // 硬件复位
        if(gpio_input_get(DSP_X2_RST_PORT, DSP_X2_RST_PIN) == 0)
        {
            SET_BIT64(g_app_param.evt_code, EVT_RESET_HW);
        }
        else
        {
            CLR_BIT64(g_app_param.evt_code, EVT_RESET_HW);
        }

        // IGBT FLT 硬件反馈
        if(gpio_input_get(DSP_IGBT_FLT_PORT, DSP_IGBT_FLT_PIN))
        {
            SET_BIT64(g_app_param.evt_code, EVT_IGBT_FLT_HW);
        }
        else
        {
            CLR_BIT64(g_app_param.evt_code, EVT_IGBT_FLT_HW);
        }

        // WU相 硬件反馈
        if(gpio_input_get(DSP_EB_WU_ERR_PORT, DSP_EB_WU_ERR_PIN))
        {
            SET_BIT64(g_app_param.evt_code, EVT_EB_WU_ERR_HW);
        }
        else
        {
            CLR_BIT64(g_app_param.evt_code, EVT_EB_WU_ERR_HW);
        }

        // VU相 硬件反馈
        if(gpio_input_get(DSP_EA_VU_ERR_PORT, DSP_EA_VU_ERR_PIN))
        {
            SET_BIT64(g_app_param.evt_code, EVT_EA_VU_ERR_HW);
        }
        else
        {
            CLR_BIT64(g_app_param.evt_code, EVT_EA_VU_ERR_HW);
        }

        // UVW缺相 硬件反馈
        if(gpio_input_get(DSP_UVW_PHASE_LOSS_PORT, DSP_UVW_PHASE_LOSS_PIN))
        {
            SET_BIT64(g_app_param.evt_code, EVT_UVW_PHASE_LOSS_HW);
        }
        else
        {
            CLR_BIT64(g_app_param.evt_code, EVT_UVW_PHASE_LOSS_HW);
        }
    }

    return 0;
}









