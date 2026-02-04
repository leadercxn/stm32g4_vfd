#include "string.h"

#include "boards.h"
#include "util.h"
#include "adc.h"

#include "parameters.h"
#include "trace.h"

#include "monitor_task.h"
#include "sensors_task.h"

/**
 * 电流阈值故障检测
 * 
 * 放在 HAL_ADCEx_InjectedConvCpltCallback 回调函数中使用， 100us 的执行频率
 */
void uvw_current_check_handle(float u_curr, float v_curr, float w_curr)
{
    static uint8_t m_u_step_cnt = 0;            // 阶跃电流计数
    static uint8_t m_v_step_cnt = 0;
    static uint8_t m_w_step_cnt = 0;
    static uint16_t m_u_over_curr_cnt = 0;      // 过流计数
    static uint16_t m_v_over_curr_cnt = 0;
    static uint16_t m_w_over_curr_cnt = 0;
    static uint16_t m_u_limit_cnt = 0;          // 限流计数
    static uint16_t m_v_limit_cnt = 0;
    static uint16_t m_w_limit_cnt = 0;

    float abs_u_curr = fabsf(u_curr);
    float abs_v_curr = fabsf(v_curr);
    float abs_w_curr = fabsf(w_curr);

    // U相电流
    if(abs_u_curr > g_app_param.step_curr_th)   // 阶跃故障判断
    {
        m_u_step_cnt++;
        if(m_u_step_cnt >= 3)                   // 连续阶跃电流
        {
            phase_pwm_stop();                   // 立即刹车
            hmi_event_set(ERR_U_STEP_CURR);     // 标记故障
        }
    }
    else
    {
        m_u_step_cnt = 0;
        // hmi_event_clear(ERR_U_STEP_CURR);    // 阶跃故障清除，放在电流恢复正常后一段时间再清除

        if(hmi_event_get(ERR_U_STEP_CURR))      // 已发生故障，直接退出函数
        {
            return ;
        }
        else                                    // 未发生故障，继续判断过流
        {
            // 电流过载判断
        }
    }
}


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
            hmi_event_set(EVT_STARTUP_HW);
        }
        else
        {
            hmi_event_clear(EVT_STARTUP_HW);
        }

        // 硬件复位
        if(gpio_input_get(DSP_X2_RST_PORT, DSP_X2_RST_PIN) == 0)
        {
            hmi_event_set(EVT_RESET_HW);
        }
        else
        {
            hmi_event_clear(EVT_RESET_HW);
        }

        // IGBT FLT 硬件反馈
        if(gpio_input_get(DSP_IGBT_FLT_PORT, DSP_IGBT_FLT_PIN))
        {
            hmi_event_set(ERR_IGBT_FLT_HW);
        }
        else
        {
            hmi_event_clear(ERR_IGBT_FLT_HW);
        }

        // WU相 硬件反馈
        if(gpio_input_get(DSP_EB_WU_ERR_PORT, DSP_EB_WU_ERR_PIN))
        {
            hmi_event_set(WARN_EB_WU_HW);
        }
        else
        {
            hmi_event_clear(WARN_EB_WU_HW);
        }

        // VU相 硬件反馈
        if(gpio_input_get(DSP_EA_VU_ERR_PORT, DSP_EA_VU_ERR_PIN))
        {
            hmi_event_set(WARN_EA_VU_HW);
        }
        else
        {
            hmi_event_clear(WARN_EA_VU_HW);
        }

        // UVW缺相 硬件反馈
        if(gpio_input_get(DSP_UVW_PHASE_LOSS_PORT, DSP_UVW_PHASE_LOSS_PIN))
        {
            hmi_event_set(WARN_EA_VU_HW);
        }
        else
        {
            hmi_event_clear(WARN_EA_VU_HW);
        }


        if( hmi_event_get(ERR_IGBT_FLT_HW) ||           \
            hmi_event_get(ERR_UVW_IN_PHASE_LOSS_HW) ||  \
            hmi_event_get(ERR_U_OVER_CURR) ||           \
            hmi_event_get(ERR_V_OVER_CURR) ||           \
            hmi_event_get(ERR_W_OVER_CURR) )
        {
            g_app_param.motor_sta = MOTOR_STA_ERROR;
        }
    }

    return 0;
}









