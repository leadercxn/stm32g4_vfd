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
#include "trace.h"

#define ADC_I_OFFSET_SAMP_TIMES     50   //静态电流采样次数

static uint16_t m_adc_average_data[ADC_CH_MAX] = {0};          //adc通道 采样平均数据

static float    m_adc_physical_value[ADC_CH_MAX] = {0};        //adc采样物理量数据， 电压单位V, 电流单位A, 温度单位℃

static uint32_t m_adc_i_offset_origin_data[3] = {0};  // 电流偏置adc采样原始数据 u,v,w
static uint32_t m_adc_inj_origin_data[3] = {0};       // 注入通道采样原始数据 u,v,w


float adc_sample_physical_value_get(adc_channel_e ch)
{
    if(ch >= ADC_CH_MAX)
    {
        return 0.0f;                            //错误通道
    }

    return m_adc_physical_value[ch];
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
     */

    /**
     * 电阻分压式母线电压 电压关系 DSP_ADCA7 = 1/2 UBUS
     *
     */

    /**
     * 变压器母线电压（隔离式母线电压） 电压关系 DSP_ADCB2 = 1/2 VCC
     *
     */

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
     * 经过自研开发板的电流采样电路
     *
     * 电压关系： DSP_ADCA2 = 3/4 * IU
     */
    // U_I
    temp = m_adc_inj_origin_data[0] - m_adc_i_offset_origin_data[0];
    // 公式统一处理  3.30f / 4095.0f * 4.0f / 3.0f / 0.12f = 0.008954
    result = temp * 0.008954f;

    m_adc_physical_value[ADC_CH_U_I] = result;

    // V_I
    temp = m_adc_inj_origin_data[1] - m_adc_i_offset_origin_data[1];
    result = temp * 0.008954f;
    m_adc_physical_value[ADC_CH_V_I] = result;

    // W_I
    temp = m_adc_inj_origin_data[2] - m_adc_i_offset_origin_data[2];
    result = temp * 0.008954f;
    m_adc_physical_value[ADC_CH_W_I] = result;
}

static uint16_t m_test_ticks = 0;

/**
 * 传感器逻辑任务
 */
int sensors_task(void)
{
    static uint32_t offset_i_cal_ticks = 0;

    static uint32_t offset_i_adc_buff[3][ADC_I_OFFSET_SAMP_TIMES] = {0};     //
    static uint8_t  offset_i_samp_index = 0;                                 //静态电流采样索引

    //静态电流采样
    if(IS_PRE_MINUS_MID_OVER_POST(sys_time_ms_get(), offset_i_cal_ticks, 50))   //间隔 50ms
    {
        offset_i_cal_ticks = sys_time_ms_get();

        if(g_app_param.motor_sta == MOTOR_STA_STOP)     //电机处于停止状态
        {
            uint32_t offset_i_adc_total[3] = {0};      //静态电流平均值U,V,W
            
            offset_i_adc_buff[0][offset_i_samp_index] = m_adc_inj_origin_data[0];
            offset_i_adc_buff[1][offset_i_samp_index] = m_adc_inj_origin_data[1];
            offset_i_adc_buff[2][offset_i_samp_index] = m_adc_inj_origin_data[2];

            offset_i_samp_index++;
            if(offset_i_samp_index > ADC_I_OFFSET_SAMP_TIMES)                //每50次统计一次静态值 10 * 1000
            {
                offset_i_samp_index = 0;                                     //重置采样索引

                for(uint8_t i = 0; i < 3; i++)
                {
                    offset_i_adc_total[i] = 0;

                    for(uint8_t j = 0; j < ADC_I_OFFSET_SAMP_TIMES; j++)
                    {
                        offset_i_adc_total[i] += offset_i_adc_buff[i][j];
                    }

                    offset_i_adc_total[i] /= ADC_I_OFFSET_SAMP_TIMES;       //计算平均值
                    m_adc_i_offset_origin_data[i] = offset_i_adc_total[i];    //保存静态电流偏移数据
                }

                trace_debug("u ofset %lu, v ofset %lu, w ofset %lu \r\n", m_adc_i_offset_origin_data[0], m_adc_i_offset_origin_data[1], m_adc_i_offset_origin_data[2]);
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

#if 1
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

            m_adc_average_data[ADC_CH_PIM_T]       = pit_t_total / 10;
            m_adc_average_data[ADC_CH_RAD_T]       = rad_t_total / 10;
            m_adc_average_data[ADC_CH_VCC_VOLT]    = vcc_volt_total / 10;
            m_adc_average_data[ADC_CH_BOX_T]       = box_t_total / 10;
            m_adc_average_data[ADC_CH_BASE_VOLT]   = base_volt_total / 10;
            m_adc_average_data[ADC_CH_UBUS_VOLT]   = ubus_volt_total / 10;

            pit_t_total       = 0;
            rad_t_total       = 0;
            vcc_volt_total    = 0;
            box_t_total       = 0;
            base_volt_total   = 0;
            ubus_volt_total   = 0;

#if 0
            trace_debug("1_ch6 %d, 1_ch7 %d, 1_ch8 %d, 1_ch9 %d, 3_ch7 %d, 3_ch11 %d, time %ld \r\n",
                m_adc_average_data[ADC_CH_PIM_T],
                m_adc_average_data[ADC_CH_RAD_T],
                m_adc_average_data[ADC_CH_VCC_VOLT],
                m_adc_average_data[ADC_CH_BOX_T],
                m_adc_average_data[ADC_CH_BASE_VOLT],
                m_adc_average_data[ADC_CH_UBUS_VOLT],
                sys_time_ms_get() );
#endif

#if 0
            trace_debug("3_ch8 U %ld, 3_ch9 V %ld, 3_ch10 %ld \r\n",
                m_adc_inj_origin_data[0],
                m_adc_inj_origin_data[1],
                m_adc_inj_origin_data[2] );

//            float iu_volt = m_adc_inj_origin_data[0] * (float)(3.30f / 4.095f * 4.0f / 3.0f) * 0.001f;
//            float iv_volt = m_adc_inj_origin_data[1] * (float)(3.30f / 4.095f * 4.0f / 3.0f) * 0.001f;
//            float iw_volt = m_adc_inj_origin_data[2] * (float)(3.30f / 4.095f * 4.0f / 3.0f) * 0.001f;
            float iu_volt = m_adc_inj_origin_data[0] * (float)(3.30f / 4.095f) * 0.001f;
            float iv_volt = m_adc_inj_origin_data[1] * (float)(3.30f / 4.095f) * 0.001f;
            float iw_volt = m_adc_inj_origin_data[2] * (float)(3.30f / 4.095f) * 0.001f;

            trace_debug("3_ch8 U %.3fV, 3_ch9 V %.3fV, 3_ch10 %.3fV \r\n",
                iu_volt, iv_volt, iw_volt );
#endif
        }

        adc_reg_origin_data_to_phy_value();     //采样数据转换物理数据
    }
#endif

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
        gpio_output_set(DSP_LED_ERR_PORT, DSP_LED_ERR_PIN, 1);

        m_adc_inj_origin_data[0] = HAL_ADCEx_InjectedGetValue(&g_adc3_handle, ADC_INJECTED_RANK_1); //U电流
        m_adc_inj_origin_data[1] = HAL_ADCEx_InjectedGetValue(&g_adc3_handle, ADC_INJECTED_RANK_2); //V电流
        m_adc_inj_origin_data[2] = HAL_ADCEx_InjectedGetValue(&g_adc3_handle, ADC_INJECTED_RANK_3); //W电流
        adc_inj_data_to_physical_value();

        m_test_ticks++;

        //电机在非停机状态下都要运行
        if((g_app_param.motor_sta > MOTOR_STA_STOP) && (g_app_param.motor_sta < MOTOR_STA_ERROR))    
        {
            g_app_param.vf_curr_theta += g_app_param.vf_step_rad;
            g_app_param.vf_curr_theta = radian_normalize(g_app_param.vf_curr_theta);
            motor_vf_run();
        }

        gpio_output_set(DSP_LED_ERR_PORT, DSP_LED_ERR_PIN, 0);
        gpio_output_set(DSP_DRIVE_IGBT_PORT, DSP_DRIVE_IGBT_PIN, 1);
    }
}
