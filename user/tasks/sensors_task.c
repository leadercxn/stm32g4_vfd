#include <math.h>
#include "string.h"

#include "boards.h"
#include "adc.h"
#include "gpio.h"
#include "sys.h"
#include "util.h"

#include "parameters.h"
#include "sensors_task.h"
#include "motor_ctrl_task.h"
#include "monitor_task.h"
#include "trace.h"

#define ADC_I_OFFSET_SAMP_TIMES     50                          // 静态电流采样次数

static uint16_t m_adc_average_data[ADC_CH_MAX] = {0};           // adc通道 采样平均数据
static float    m_adc_physical_value[ADC_CH_MAX] = {0};         // adc采样物理量数据， 电压单位V, 电流单位A, 温度单位℃
static uint32_t m_adc_i_offset_origin_data[3] = {0};            // 电流偏置adc采样原始数据 u,v,w
static uint32_t m_adc_inj_origin_data[3] = {0};                 // 注入通道采样原始数据 u,v,w

static bool m_adc_i_offset_cal_done = false;                    // 电流偏置校准完成标志

static uint16_t m_rms_curr_req_samp_cnt = 0;                    // 均方根要采样的次数
static float m_u_rms_curr = 0.0f;                               // U相RMS均方根 -- 每一周期值
static float m_v_rms_curr = 0.0f;
static float m_w_rms_curr = 0.0f;
static float m_u_rms_curr_aver = 0.0f;                          // U相RMS均方根 -- 平均值 数个周期平均值
static float m_v_rms_curr_aver = 0.0f;
static float m_w_rms_curr_aver = 0.0f;

/**
 * @brief 计算输出UVW三相的 RMS 均方根
 * 
 * 放在 HAL_ADCEx_InjectedConvCpltCallback 回调函数中使用， 100us 的执行频率
 */
static void adc_uvw_rms_curr_cal(void)
{
    static float u_rms_curr_total = 0.0f;             // 统计平方和
    static float v_rms_curr_total = 0.0f;
    static float w_rms_curr_total = 0.0f;
    
    static uint16_t old_rms_curr_req_samp_cnt = 0;    //上次设置的要采样的次数
    static uint16_t rms_curr_samp_cnt = 0;            //已经采样的次数

    static float u_rms_curr_aver_total = 0.0f;        // 均方根平均值统计
    static float v_rms_curr_aver_total = 0.0f;
    static float w_rms_curr_aver_total = 0.0f;
    static uint16_t rms_curr_aver_cnt = 0;            // 当前均方根平均值统计次数

    if((m_rms_curr_req_samp_cnt == old_rms_curr_req_samp_cnt) && (m_rms_curr_req_samp_cnt > 0))  //频率没有变化，继续统计 且 采样要求次数大于0
    {
        // U
        u_rms_curr_total += (m_adc_physical_value[ADC_CH_U_I] * m_adc_physical_value[ADC_CH_U_I]);
        // V
        v_rms_curr_total += (m_adc_physical_value[ADC_CH_V_I] * m_adc_physical_value[ADC_CH_V_I]);
        // W
        w_rms_curr_total += (m_adc_physical_value[ADC_CH_W_I] * m_adc_physical_value[ADC_CH_W_I]);

        rms_curr_samp_cnt++;
        if(rms_curr_samp_cnt >= m_rms_curr_req_samp_cnt)    //均方根采样完成
        {
            float rms_value = sqrtf(u_rms_curr_total / rms_curr_samp_cnt);
            m_u_rms_curr = rms_value;

            rms_value = sqrtf(v_rms_curr_total / rms_curr_samp_cnt);
            m_v_rms_curr = rms_value;

            rms_value = sqrtf(w_rms_curr_total / rms_curr_samp_cnt);
            m_w_rms_curr = rms_value;

            // 重置统计数据
            u_rms_curr_total = 0.0f;
            v_rms_curr_total = 0.0f;
            w_rms_curr_total = 0.0f;
            rms_curr_samp_cnt = 0;

            u_rms_curr_aver_total += m_u_rms_curr;
            v_rms_curr_aver_total += m_v_rms_curr;
            w_rms_curr_aver_total += m_w_rms_curr;
            rms_curr_aver_cnt++;
            if(rms_curr_aver_cnt >= 10)    //均方根平均值统计完成
            {
                // 计算均方根平均值
                m_u_rms_curr_aver = u_rms_curr_aver_total / rms_curr_aver_cnt;
                m_v_rms_curr_aver = v_rms_curr_aver_total / rms_curr_aver_cnt;
                m_w_rms_curr_aver = w_rms_curr_aver_total / rms_curr_aver_cnt;

                // 重置均方根平均值统计数据
                u_rms_curr_aver_total = 0.0f;
                v_rms_curr_aver_total = 0.0f;
                w_rms_curr_aver_total = 0.0f;
                rms_curr_aver_cnt = 0;
            }
        }
    }
    else                                                        //频率发生变化，重新清0
    {
        // 重置统计数据
        u_rms_curr_total  = 0.0f;
        v_rms_curr_total  = 0.0f;
        w_rms_curr_total  = 0.0f;
        rms_curr_samp_cnt = 0;

// 暂时不用重置平均值的计算
//        u_rms_curr_aver_total = 0.0f;
//        v_rms_curr_aver_total = 0.0f;
//        w_rms_curr_aver_total = 0.0f;
//        rms_curr_aver_cnt = 0;

        old_rms_curr_req_samp_cnt = m_rms_curr_req_samp_cnt;
    }
}

float adc_sample_physical_value_get(adc_channel_e ch)
{
    if(ch >= ADC_CH_MAX)
    {
        return 0.0f;                            //错误通道
    }

    return m_adc_physical_value[ch];
}

/**
 * @brief 获取每一周期的均方根电流值
 */
float rms_curr_get(adc_channel_e ch)
{
    if(ch == ADC_CH_U_I)
    {
        return m_u_rms_curr;
    }
    else if(ch == ADC_CH_V_I)
    {
        return m_v_rms_curr;
    }
    else if(ch == ADC_CH_W_I)
    {
        return m_w_rms_curr;
    }
    
    return 0.0f;
}

/**
 * @brief 获取均方根电流平均值
 */
float rms_curr_aver_get(adc_channel_e ch)
{ 
    if(ch == ADC_CH_U_I)
    {
        return m_u_rms_curr_aver;
    }
    else if(ch == ADC_CH_V_I)
    {
        return m_v_rms_curr_aver;
    }
    else if(ch == ADC_CH_W_I)
    {
        return m_w_rms_curr_aver;
    }
    
    return 0.0f;
}

/**
 * @brief       adc 规则通道原始数据 转化为 对应的物理量
 */
static void adc_reg_origin_data_to_phy_value(void)
{
    int     temp;
    float   result = 0.0f;

    /**
     * 机箱体 电压关系 DSP_ADCB4 = box_t 传感器输出，直连
     */

    /**
     * 基准电压 电压关系 DSP_ADCA0 = base_volt , 直连
     * 
     * 公式统一处理 adc * 3.30f / 4095.0f = adc * 0.00080586f
     */
    m_adc_physical_value[ADC_CH_BASE_VOLT] = m_adc_average_data[ADC_CH_BASE_VOLT] * 0.00080586f;

    /**
     * 电阻分压式母线电压 电压关系 DSP_ADCA7 = 1/2 UBUS
     * 驱动板电压关系 P750V_VOLT * [75 / (75 + 360 + 6000)] = P750V_VOLT * 75 / 6435  = UBUS
     * 
     * 公式统一处理 adc * 3.30f / 4095.0f * 2.0f * 6435.0f / 75.0f = adc * 0.138286f
     */
    m_adc_physical_value[ADC_CH_UBUS_VOLT] = m_adc_average_data[ADC_CH_UBUS_VOLT] * 0.138286f;

    /**
     * 变压器母线电压（隔离式母线电压） 电压关系 DSP_ADCB2 = 1/2 VCC
     * 驱动板关系 VCC * 10 / (10 + 10 + 1) = adc / 4095 * 3.30f
     * 
     * 公式统一处理 adc * 3.30f / 4095.0f * 21.0f / 10.0f= adc * 0.001692f
     */
    m_adc_physical_value[ADC_CH_VCC_VOLT] = m_adc_average_data[ADC_CH_VCC_VOLT] * 0.001692f;

    /**
     * IGBT温度 电压关系 DSP_ADCB0 = 1/2 PIM-T
     *
     */

    /**
     * 散热片温度 电压关系 DSP_ADCB1 =  3/5 JX1 (预留)
     *
     */

} 


/**
 * @brief       adc 注入通道原始数据 转化为 对应的物理量
 */
static void adc_inj_data_to_physical_value(void)
{
    int temp;
    float   result = 0.0f;

    /**
     * 经过 自研控制板的 电流采样 自研电机驱动板
     *
     * 电压关系： DSP_ADCA2 = 3/4 * IU
     */
    // U_I
    temp = m_adc_inj_origin_data[0] - m_adc_i_offset_origin_data[0];
    // 公式统一处理  ( adc * 3.30f / 4095.0f * 4.0f / 3.0f - 2.50f ) / 0.02f = (adc * 0.001074 - 2.50f) / 0.02f = adc * 0.053724 - 125.0f
    result = temp * 0.053724f;
    // m_adc_physical_value[ADC_CH_U_I] = result;   //直接取值
    m_adc_physical_value[ADC_CH_U_I] =  result * 0.1 + m_adc_physical_value[ADC_CH_U_I] * 0.9;   //一阶滤波处理

    // V_I
    temp = m_adc_inj_origin_data[1] - m_adc_i_offset_origin_data[1];
    result = temp * 0.053724f;
    //m_adc_physical_value[ADC_CH_V_I] = result;
    m_adc_physical_value[ADC_CH_V_I] = result * 0.1 + m_adc_physical_value[ADC_CH_V_I] * 0.9;   //一阶滤波处理

    // W_I
    temp = m_adc_inj_origin_data[2] - m_adc_i_offset_origin_data[2];
    result = temp * 0.053724f;
    //m_adc_physical_value[ADC_CH_W_I] = result;
    m_adc_physical_value[ADC_CH_W_I] = result * 0.1 + m_adc_physical_value[ADC_CH_W_I] * 0.9;   //一阶滤波处理
}

static uint16_t m_test_ticks = 0;

/**
 * 传感器逻辑任务
 */
int sensors_task(void)
{
    static uint32_t offset_i_cal_ticks = 0;
    static uint8_t  ofset_i_samp_cnt = 0;           //静态电流采样次数
    static uint32_t offset_i_adc_total[3] = {0};    //静态电流平均值U,V,W

    //静态电流采样
    if(IS_PRE_MINUS_MID_OVER_POST(sys_time_ms_get(), offset_i_cal_ticks, 50))   //间隔 50ms
    {
        offset_i_cal_ticks = sys_time_ms_get();

        if(g_app_param.motor_sta == MOTOR_STA_STOP)     //电机处于停止状态
        {
            offset_i_adc_total[0] += m_adc_inj_origin_data[0];
            offset_i_adc_total[1] += m_adc_inj_origin_data[1];
            offset_i_adc_total[2] += m_adc_inj_origin_data[2];

            ofset_i_samp_cnt++;
            if(ofset_i_samp_cnt >= ADC_I_OFFSET_SAMP_TIMES)               //每50次统计一次静态值
            {
                ofset_i_samp_cnt = 0;                                     //重置采样索引

                m_adc_i_offset_origin_data[0] = offset_i_adc_total[0] / ADC_I_OFFSET_SAMP_TIMES;
                m_adc_i_offset_origin_data[1] = offset_i_adc_total[1] / ADC_I_OFFSET_SAMP_TIMES;
                m_adc_i_offset_origin_data[2] = offset_i_adc_total[2] / ADC_I_OFFSET_SAMP_TIMES;

                offset_i_adc_total[0] = 0;
                offset_i_adc_total[1] = 0;
                offset_i_adc_total[2] = 0;

                m_adc_i_offset_cal_done = true;    //电流偏置采样完成标志

//              trace_debug("u ofset %lu, v ofset %lu, w ofset %lu \r\n", m_adc_i_offset_origin_data[0], m_adc_i_offset_origin_data[1], m_adc_i_offset_origin_data[2]);
            }
        }
    }


    // 常规采样
    static uint32_t sens_collect_ticks = 0;

    static uint32_t pit_t_total = 0;
    static uint32_t rad_t_total = 0;
    static uint32_t vcc_volt_total = 0;
    static uint32_t box_t_total = 0;
    static uint32_t base_volt_total = 0;
    static uint32_t ubus_volt_total = 0;

    static uint8_t  adc_collect_cnt = 0;

    if(IS_PRE_MINUS_MID_OVER_POST(sys_time_ms_get(), sens_collect_ticks, 100))   //间隔 100 ms
    {
        sens_collect_ticks = sys_time_ms_get();

        pit_t_total       += adc1_ch6_val_get();
        rad_t_total       += adc1_ch7_val_get();
        vcc_volt_total    += adc1_ch8_val_get();
        box_t_total       += adc1_ch9_val_get();
        base_volt_total   += adc3_ch7_val_get();
        ubus_volt_total   += adc3_ch11_val_get();

        adc_collect_cnt++;

        if(adc_collect_cnt >= 10)    //10次采样取平均
        {
            adc_collect_cnt = 0;

            m_adc_average_data[ADC_CH_PIM_IGBT_T]  = pit_t_total / 10;
            m_adc_average_data[ADC_CH_RAD_T]       = rad_t_total / 10;
            m_adc_average_data[ADC_CH_VCC_VOLT]    = vcc_volt_total / 10;
            m_adc_average_data[ADC_CH_CTL_BSP_T]   = box_t_total / 10;
            m_adc_average_data[ADC_CH_BASE_VOLT]   = base_volt_total / 10;
            m_adc_average_data[ADC_CH_UBUS_VOLT]   = ubus_volt_total / 10;

            pit_t_total       = 0;
            rad_t_total       = 0;
            vcc_volt_total    = 0;
            box_t_total       = 0;
            base_volt_total   = 0;
            ubus_volt_total   = 0;

            adc_reg_origin_data_to_phy_value();     //采样数据转换物理数据

#if 0
            trace_debug("1_ch6 PIM_T %d, 1_ch7 RAD_T %d, 1_ch8 VCC_VOLT %d, 1_ch9 BOX_T %d, 3_ch7 BASE_VOLT %d, 3_ch11 UBUS_VOLT %d, time %ld \r\n",
                m_adc_average_data[ADC_CH_PIM_IGBT_T],
                m_adc_average_data[ADC_CH_RAD_T],
                m_adc_average_data[ADC_CH_VCC_VOLT],
                m_adc_average_data[ADC_CH_CTL_BSP_T],
                m_adc_average_data[ADC_CH_BASE_VOLT],
                m_adc_average_data[ADC_CH_UBUS_VOLT],
                sys_time_ms_get() );
#endif

#if 0
            trace_debug("PIM_T %.2f, RAD_T %.2f, VCC_VOLT %.2fV, BOX_T %.2f, BASE_VOLT %.2fV, UBUS_VOLT %.2fV, I-U %.2f, I-V %.2f, I-W %.2f\r\n",
                m_adc_physical_value[ADC_CH_PIM_IGBT_T],
                m_adc_physical_value[ADC_CH_RAD_T],
                m_adc_physical_value[ADC_CH_VCC_VOLT],
                m_adc_physical_value[ADC_CH_CTL_BSP_T],
                m_adc_physical_value[ADC_CH_BASE_VOLT],
                m_adc_physical_value[ADC_CH_UBUS_VOLT],
                m_adc_physical_value[ADC_CH_U_I],
                m_adc_physical_value[ADC_CH_V_I],
                m_adc_physical_value[ADC_CH_W_I] );
#endif

        }
    }

    

// 中断计时验证
#if 0
    if(m_test_ticks >= 1000)    // 理论上对应 100ms
    {
        m_test_ticks = 0;

        trace_debug("sys time ms %lu\r\n", sys_time_ms_get());
    }
#endif

    return 0;
}


/**
 * @brief       规则通道ADC转换完成的回调函数
 * @param       无
 * @retval      无
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) 
    {

    }
    else if(hadc->Instance == ADC3)
    {

    }
}


/**
 * @brief       注入通道ADC转换完成的回调函数， 参考 adc3_inj_start 执行频率，目前应该是 10K
 * @param       无
 * @retval      无
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC3)
    {
        m_adc_inj_origin_data[0] = HAL_ADCEx_InjectedGetValue(&g_adc3_handle, ADC_INJECTED_RANK_1); //U电流
        m_adc_inj_origin_data[1] = HAL_ADCEx_InjectedGetValue(&g_adc3_handle, ADC_INJECTED_RANK_2); //V电流
        m_adc_inj_origin_data[2] = HAL_ADCEx_InjectedGetValue(&g_adc3_handle, ADC_INJECTED_RANK_3); //W电流

        if(m_adc_i_offset_cal_done)             // 静态电流采集完成flag
        {
            adc_inj_data_to_physical_value();   // 转为物理量

            if(g_app_param.curr_speed_ring_s != 0)  // 避免除0错误， 且能确认到电机在转动
            {
                m_rms_curr_req_samp_cnt = (uint16_t)(FOC_FREQ / g_app_param.curr_speed_ring_s);   // 计算均方根电流采样次数
                adc_uvw_rms_curr_cal();     // 要运动起来后才有周期，才能计算UVW三相均方根电流
            }

            //检测阶跃电流，和均方根电流
            uvw_current_check_handle(m_adc_physical_value[ADC_CH_U_I], m_adc_physical_value[ADC_CH_V_I], m_adc_physical_value[ADC_CH_W_I]);

            //电机在非停机状态下都要运行
            if((g_app_param.motor_sta > MOTOR_STA_STOP) && (g_app_param.motor_sta < MOTOR_STA_ERROR))    
            {
                g_app_param.vf_curr_theta += g_app_param.vf_step_rad;
                g_app_param.vf_curr_theta = radian_normalize(g_app_param.vf_curr_theta);
                motor_vf_run();
            }
        }
//        m_test_ticks++;
    }
}
