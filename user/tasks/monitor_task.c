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

    static uint8_t u_i_over_curr_cnt = 0;
    static uint8_t v_i_over_curr_cnt = 0;
    static uint8_t w_i_over_curr_cnt = 0;

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

        // 三相电流过流检测
        if(adc_sample_physical_value_get(ADC_CH_U_I) > 2.0f)
        {
            u_i_over_curr_cnt++;
            if(u_i_over_curr_cnt >= 5)    // 连续3次过流
            {
                u_i_over_curr_cnt = 5;
                SET_BIT64(g_app_param.evt_code, EVT_U_OVER_CURR);
            }
        }
        else
        {
            u_i_over_curr_cnt = 0;

            CLR_BIT64(g_app_param.evt_code, EVT_U_OVER_CURR);
        }

        if(adc_sample_physical_value_get(ADC_CH_V_I) > 2.0f)
        {
            v_i_over_curr_cnt++;
            if(v_i_over_curr_cnt >= 5)    // 连续3次过流
            {
                v_i_over_curr_cnt = 5;
                SET_BIT64(g_app_param.evt_code, EVT_V_OVER_CURR);
            }
        }
        else
        {
            v_i_over_curr_cnt = 0;

            CLR_BIT64(g_app_param.evt_code, EVT_V_OVER_CURR);
        }

        if(adc_sample_physical_value_get(ADC_CH_W_I) > 2.0f)
        {
            w_i_over_curr_cnt++;
            if(w_i_over_curr_cnt >= 5)    // 连续3次过流
            {
                w_i_over_curr_cnt = 5;
                SET_BIT64(g_app_param.evt_code, EVT_W_OVER_CURR);
            }
        }
        else
        {
            w_i_over_curr_cnt = 0;

            CLR_BIT64(g_app_param.evt_code, EVT_W_OVER_CURR);
        }



        if( IS_SET64(g_app_param.evt_code, EVT_IGBT_FLT_HW) || \
            IS_SET64(g_app_param.evt_code, EVT_U_OVER_CURR) || \
            IS_SET64(g_app_param.evt_code, EVT_V_OVER_CURR) || \
            IS_SET64(g_app_param.evt_code, EVT_W_OVER_CURR) )
        {
            g_app_param.motor_sta = MOTOR_STA_ERROR;
        }
    }

    return 0;
}









