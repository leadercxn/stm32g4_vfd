#include "string.h"

#include "boards.h"
#include "util.h"
#include "adc.h"

#include "parameters.h"
#include "trace.h"
#include "mb_slaver_task.h"

#include "monitor_task.h"
#include "sensors_task.h"

/**
 * @brief 阶跃电流、过流、限流检查处理函数
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
            m_u_step_cnt = 3;
            phase_pwm_stop();                   // 立即刹车
            hmi_event_set(ERR_U_STEP_CURR);     // 标记故障
            hmi_event_clear(ERR_U_OVER_CURR);   // 清除过流故障
            hmi_event_clear(WARN_U_CURR_LIMIT); // 清除限流警告
        }
    }
    else
    {
        m_u_step_cnt = 0;
        if(hmi_event_get(ERR_U_STEP_CURR))      // 已发生故障，直接退出函数
        {
            return ;
        }
        else                                    // 未发生故障，继续判断过流
        {
            // 电流过载判断 -- 均方根
            if(rms_curr_get(ADC_CH_U_I) > g_app_param.over_curr_th)
            {
                m_u_over_curr_cnt++;
                if(m_u_over_curr_cnt >= 50)             // 50 次过流
                {
                    m_u_over_curr_cnt = 50;
                    phase_pwm_stop();                   // 立即刹车
                    hmi_event_set(ERR_U_OVER_CURR);
                    hmi_event_clear(WARN_U_CURR_LIMIT); // 清除限流警告
                }
            }
            else
            {
                m_u_over_curr_cnt = 0;
                if(hmi_event_get(ERR_U_OVER_CURR))      // 已发生故障，直接退出函数
                {
                    return ;
                }
                else                                    // 未发生过流，继续判断限流
                {
                    if(rms_curr_get(ADC_CH_U_I) > g_app_param.limit_curr_th)
                    {
                        m_u_limit_cnt++;
                        if(m_u_limit_cnt >= 50)       // 100 次限流
                        {
                            m_u_limit_cnt = 50;
                            hmi_event_set(WARN_U_CURR_LIMIT);
                        }
                    }
                    else
                    {
                        m_u_limit_cnt = 0;
                        if(hmi_event_get(WARN_U_CURR_LIMIT))    // 已发生限流， 得要电流下降到一定程度会才可以消除
                        {
                            if(rms_curr_get(ADC_CH_U_I) < (g_app_param.limit_curr_th - 3.0f))   // 电流限流预警值回差
                            {
                                hmi_event_clear(WARN_U_CURR_LIMIT);
                            }
                        }
                    }
                }
            }
        }
    }

    // V相电流
    if(abs_v_curr > g_app_param.step_curr_th)   // 阶跃故障判断
    {
        m_v_step_cnt++;
        if(m_v_step_cnt >= 3)                   // 连续阶跃电流
        {
            m_v_step_cnt = 3;
            phase_pwm_stop();                   // 立即刹车
            hmi_event_set(ERR_V_STEP_CURR);     // 标记故障
            hmi_event_clear(ERR_V_OVER_CURR);   // 清除过流故障
            hmi_event_clear(WARN_V_CURR_LIMIT); // 清除限流警告
        }
    }
    else
    {
        m_v_step_cnt = 0;
        if(hmi_event_get(ERR_V_STEP_CURR))      // 已发生故障，直接退出函数
        {
            return ;
        }
        else                                    // 未发生故障，继续判断过流
        {
            // 电流过载判断 -- 均方根
            if(rms_curr_get(ADC_CH_V_I) > g_app_param.over_curr_th)
            {
                m_v_over_curr_cnt++;
                if(m_v_over_curr_cnt >= 50)             // 50 次过流
                {
                    m_v_over_curr_cnt = 50;
                    phase_pwm_stop();                   // 立即刹车
                    hmi_event_set(ERR_V_OVER_CURR);
                    hmi_event_clear(WARN_V_CURR_LIMIT); // 清除限流警告
                }
            }
            else
            {
                m_v_over_curr_cnt = 0;
                if(hmi_event_get(ERR_V_OVER_CURR))      // 已发生故障，直接退出函数
                {
                    return ;
                }
                else                                    // 未发生过流，继续判断限流
                {
                    if(rms_curr_get(ADC_CH_V_I) > g_app_param.limit_curr_th)
                    {
                        m_v_limit_cnt++;
                        if(m_v_limit_cnt >= 50)       // 100 次限流
                        {
                            m_v_limit_cnt = 50;
                            hmi_event_set(WARN_V_CURR_LIMIT);
                        }
                    }
                    else
                    {
                        m_v_limit_cnt = 0;
                        if(hmi_event_get(WARN_V_CURR_LIMIT))    // 已发生限流， 得要电流下降到一定程度会才可以消除
                        {
                            if(rms_curr_get(ADC_CH_V_I) < (g_app_param.limit_curr_th - 3.0f))   // 电流限流预警值回差
                            {
                                hmi_event_clear(WARN_V_CURR_LIMIT);
                            }
                        }
                    }
                }
            }
        }
    }

    // W相电流
    if(abs_w_curr > g_app_param.step_curr_th)   // 阶跃故障判断
    {
        m_w_step_cnt++;
        if(m_w_step_cnt >= 3)                   // 连续阶跃电流
        {
            m_w_step_cnt = 3;
            phase_pwm_stop();                   // 立即刹车
            hmi_event_set(ERR_W_STEP_CURR);     // 标记故障
            hmi_event_clear(ERR_W_OVER_CURR);   // 清除过流故障
            hmi_event_clear(WARN_W_CURR_LIMIT); // 清除限流警告
        }
    }
    else
    {
        m_w_step_cnt = 0;
        if(hmi_event_get(ERR_W_STEP_CURR))      // 已发生故障，直接退出函数
        {
            return ;
        }
        else                                    // 未发生故障，继续判断过流
        {
            // 电流过载判断 -- 均方根
            if(rms_curr_get(ADC_CH_W_I) > g_app_param.over_curr_th)
            {
                m_w_over_curr_cnt++;
                if(m_w_over_curr_cnt >= 50)             // 50 次过流
                {
                    m_w_over_curr_cnt = 50;
                    phase_pwm_stop();                   // 立即刹车
                    hmi_event_set(ERR_W_OVER_CURR);
                    hmi_event_clear(WARN_W_CURR_LIMIT); // 清除限流警告
                }
            }
            else
            {
                m_w_over_curr_cnt = 0;
                if(hmi_event_get(ERR_W_OVER_CURR))      // 已发生故障，直接退出函数
                {
                    return ;
                }
                else                                    // 未发生过流，继续判断限流
                {
                    if(rms_curr_get(ADC_CH_W_I) > g_app_param.limit_curr_th)
                    {
                        m_w_limit_cnt++;
                        if(m_w_limit_cnt >= 50)       // 100 次限流
                        {
                            m_w_limit_cnt = 50;
                            hmi_event_set(WARN_W_CURR_LIMIT);
                        }
                    }
                    else
                    {
                        m_w_limit_cnt = 0;
                        if(hmi_event_get(WARN_W_CURR_LIMIT))    // 已发生限流， 得要电流下降到一定程度会才可以消除
                        {
                            if(rms_curr_get(ADC_CH_W_I) < (g_app_param.limit_curr_th - 3.0f))   // 电流限流预警值回差
                            {
                                hmi_event_clear(WARN_W_CURR_LIMIT);
                            }
                        }
                    }
                }
            }
        }
    }
}

/**
 * @brief
 *  PIM-IGBT 模块温度过温，预警检测;  
 *  散热器温度过温、预警检测; 
 *  控制板温度过温检测处理函数
 */
static void temp_sens_check_handle(void)
{
    static uint16_t pim_igbt_over_t_cnt = 0;      // PIM-IGBT模块过温计数
    static uint16_t heat_rad_over_t_cnt = 0;      // 散热器过温计数
    static uint16_t ctrl_board_over_t_cnt = 0;    // 控制板过温计数

    static uint16_t pim_igbt_warn_t_cnt = 0;      // PIM-IGBT模块预警计数
    static uint16_t heat_rad_warn_t_cnt = 0;      // 散热器预警计数

    // PIM-IGBT模块温度过温、预警检测
    if(adc_sample_physical_value_get(ADC_CH_PIM_IGBT_T) > g_app_param.pim_igbt_over_t_th)  // PIM-IGBT模块过温判断
    {
        pim_igbt_over_t_cnt++;
        if(pim_igbt_over_t_cnt >= 50)                   // 数次过温
        {
            pim_igbt_over_t_cnt = 50;
            hmi_event_set(ERR_PIM_IGBT_T_OVER_TH);     // 设置故障
            hmi_event_clear(WARN_PIM_IGBT_T_LIMIT);    // 清除预警
        }
    }
    else
    {
        pim_igbt_over_t_cnt = 0;
        if(hmi_event_get(ERR_PIM_IGBT_T_OVER_TH))      // 已发生过温故障
        {
            if(adc_sample_physical_value_get(ADC_CH_PIM_IGBT_T) < (g_app_param.pim_igbt_over_t_th - 10.0f))   // 温度过温故障回差
            {
                hmi_event_clear(ERR_PIM_IGBT_T_OVER_TH);
            }
        }
        else                                           // 未发生过温故障，继续判断预警
        {
            if(adc_sample_physical_value_get(ADC_CH_PIM_IGBT_T) > g_app_param.pim_igbt_limit_t_th)
            {
                pim_igbt_warn_t_cnt++;
                if(pim_igbt_warn_t_cnt >= 50)          // 数次预警
                {
                    pim_igbt_warn_t_cnt = 50;
                    hmi_event_set(WARN_PIM_IGBT_T_LIMIT);
                }
            }
            else
            {
                pim_igbt_warn_t_cnt = 0;
                if(hmi_event_get(WARN_PIM_IGBT_T_LIMIT))    // 已发生预警， 得要温度下降到一定程度会才可以消除
                {
                    if(adc_sample_physical_value_get(ADC_CH_PIM_IGBT_T) < (g_app_param.pim_igbt_limit_t_th - 5.0f))   // 温度预警回差
                    {
                        hmi_event_clear(WARN_PIM_IGBT_T_LIMIT);
                    }
                }
            }
        }
    }

#if 0       // 目前还只是预留
    // 散热器温度过温、预警检测 -- 预留
    if(adc_sample_physical_value_get(ADC_CH_RAD_T) > g_app_param.rad_over_t_th)  // 散热器过温判断
    {
        heat_rad_over_t_cnt++;
        if(heat_rad_over_t_cnt >= 50)                   // 数次过温
        {
            heat_rad_over_t_cnt = 50;
            hmi_event_set(ERR_RAD_T_OVER_TH);     // 设置故障
            hmi_event_clear(WARN_RAD_T_LIMIT);    // 清除预警
        }
    }
    else
    {
        heat_rad_over_t_cnt = 0;
        if(hmi_event_get(ERR_RAD_T_OVER_TH))      // 已发生过温故障
        {
            if(adc_sample_physical_value_get(ADC_CH_RAD_T) < (g_app_param.rad_over_t_th - 10.0f))   // 温度过温故障回差
            {
                hmi_event_clear(ERR_RAD_T_OVER_TH);
            }
        }
        else                                           // 未发生过温故障，继续判断预警
        {
            if(adc_sample_physical_value_get(ADC_CH_RAD_T) > g_app_param.rad_limit_t_th)
            {
                heat_rad_warn_t_cnt++;
                if(heat_rad_warn_t_cnt >= 50)          // 数次预警
                {
                    heat_rad_warn_t_cnt = 50;
                    hmi_event_set(WARN_RAD_T_LIMIT);
                }
            }
            else
            {
                heat_rad_warn_t_cnt = 0;
                if(hmi_event_get(WARN_RAD_T_LIMIT))    // 已发生预警， 得要温度下降到一定程度会才可以消除
                {
                    if(adc_sample_physical_value_get(ADC_CH_RAD_T) < (g_app_param.rad_limit_t_th - 5.0f))   // 温度预警回差
                    {
                        hmi_event_clear(WARN_RAD_T_LIMIT);
                    }
                }
            }
        }
    }
#endif

    // 控制板温度预警检测处理函数
    if(adc_sample_physical_value_get(ADC_CH_CTL_BSP_T) > g_app_param.ctrl_bsp_warn_t_th)
    {
        ctrl_board_over_t_cnt++;
        if(ctrl_board_over_t_cnt >= 50)                   // 数次过温
        {
            ctrl_board_over_t_cnt = 50;
            hmi_event_set(WARN_CTRL_BSP_T);       // 设置故障
        }
    }
    else
    {
        ctrl_board_over_t_cnt = 0;
        if(hmi_event_get(WARN_CTRL_BSP_T))        // 已发生过温故障
        {
            if(adc_sample_physical_value_get(ADC_CH_CTL_BSP_T) < (g_app_param.ctrl_bsp_warn_t_th - 10.0f))   // 温度过温故障回差
            {
                hmi_event_clear(WARN_CTRL_BSP_T);
            }
        }
    }

}

/**
 * @brief
 * 直流母线电压过压、欠压检测处理函数
 * 变压器输出电压过压、欠压检测处理函数
 * 基准电压检测异常处理函数
 */
static void volt_sens_check_handle(void)
{
    static uint16_t ubus_over_volt_cnt = 0;        // 直流母线过压计数
    static uint16_t ubus_under_volt_cnt = 0;       // 直流母线欠压计数
    static uint16_t tran_out_err_volt_cnt = 0;     // 变压器输出异常计数
    static uint16_t base_volt_err_cnt = 0;         // 基准电压异常计数

    // 直流母线电压过压检测
    if(adc_sample_physical_value_get(ADC_CH_UBUS_VOLT) > g_app_param.ubus_over_volt_th)
    {
        ubus_over_volt_cnt++;
        if(ubus_over_volt_cnt >= 50)                   // 数次过压
        {
            ubus_over_volt_cnt = 50;
            hmi_event_set(ERR_UBUS_OVER_VOLT);        // 设置故障
        }
    }
    else
    {
        ubus_over_volt_cnt = 0;
        if(hmi_event_get(ERR_UBUS_OVER_VOLT))         // 已发生过压故障
        {
            if(adc_sample_physical_value_get(ADC_CH_UBUS_VOLT) < (g_app_param.ubus_over_volt_th - 10))   // 过压故障回差
            {
                hmi_event_clear(ERR_UBUS_OVER_VOLT);
            }
        }
    }

    // 直流母线电压欠压检测
    if(adc_sample_physical_value_get(ADC_CH_UBUS_VOLT) < g_app_param.ubus_under_volt_th)
    {
        ubus_under_volt_cnt++;
        if(ubus_under_volt_cnt >= 50)                  // 数次欠压
        {
            ubus_under_volt_cnt = 50;
            hmi_event_set(ERR_UBUS_UNDER_VOLT);       // 设置故障
        }
    }
    else
    {
        ubus_under_volt_cnt = 0;
        if(hmi_event_get(ERR_UBUS_UNDER_VOLT))        // 已发生欠压故障
        {
            if(adc_sample_physical_value_get(ADC_CH_UBUS_VOLT) > (g_app_param.ubus_under_volt_th + 10))   // 欠压故障回差
            {
                hmi_event_clear(ERR_UBUS_UNDER_VOLT);
            }
        }
    }

    // 变压器输出电压异常检测
    if( (adc_sample_physical_value_get(ADC_CH_VCC_VOLT) > 7.0f) || \
        (adc_sample_physical_value_get(ADC_CH_VCC_VOLT) < 5.0) )
    {
        tran_out_err_volt_cnt++;
        if(tran_out_err_volt_cnt >= 50)                  // 数次异常
        {
            tran_out_err_volt_cnt = 50;
            hmi_event_set(ERR_TRAN_OUT_ABNORMAL);        // 设置故障
        }
    }
    else
    {
        tran_out_err_volt_cnt = 0;
        if(hmi_event_get(ERR_TRAN_OUT_ABNORMAL))         // 已发生异常故障
        {
            if( (adc_sample_physical_value_get(ADC_CH_VCC_VOLT) < 6.8f) && \
                (adc_sample_physical_value_get(ADC_CH_VCC_VOLT) > 5.2f) )
            {
                hmi_event_clear(ERR_TRAN_OUT_ABNORMAL);
            }
        }
    }

    // 基准电压异常检测
    if( (adc_sample_physical_value_get(ADC_CH_BASE_VOLT) > 2.3f) || \
        (adc_sample_physical_value_get(ADC_CH_BASE_VOLT) < 1.7f) )
    {
        base_volt_err_cnt++;
        if(base_volt_err_cnt >= 50)                      // 数次异常
        {
            base_volt_err_cnt = 50;
            hmi_event_set(WARN_BASE_VOLT);      // 设置故障
        }
    }
    else
    {
        base_volt_err_cnt = 0;
        if(hmi_event_get(WARN_BASE_VOLT))       // 已发生异常故障
        {
            if( (adc_sample_physical_value_get(ADC_CH_BASE_VOLT) < 2.2f) && \
                (adc_sample_physical_value_get(ADC_CH_BASE_VOLT) > 1.8f) )
            {
                hmi_event_clear(WARN_BASE_VOLT);
            }
        }
    }

}


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

        temp_sens_check_handle();     // 温度传感器检查处理
        volt_sens_check_handle();     // 电压传感器检查处理

        // 发生 ERR 事件， 切换到故障状态
        if( hmi_event_get(ERR_IGBT_FLT_HW) ||           \
            hmi_event_get(ERR_UVW_IN_PHASE_LOSS_HW) ||  \
            hmi_event_get(ERR_U_OVER_CURR) ||           \
            hmi_event_get(ERR_V_OVER_CURR) ||           \
            hmi_event_get(ERR_W_OVER_CURR) ||           \
            hmi_event_get(ERR_PIM_IGBT_T_OVER_TH) ||    \
            hmi_event_get(ERR_TRAN_OUT_ABNORMAL))
        {
            g_app_param.motor_sta = MOTOR_STA_ERROR;
        }
    }

    return 0;
}









