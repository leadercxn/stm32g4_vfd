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

} 

static void adc_inj_data_to_physical_value(void)
{
    int temp;
    float   result = 0.0f;

    // U_I
    temp = m_adc_inj_origin_data[0] - m_adc_i_offset_origin_data[0];

    result = temp * (float)(3.3f / 4.0960f / 0.12f);
    result *= 0.001f;
    m_adc_physical_value[ADC_CH_U_I] = result;

    // V_I
    temp = m_adc_inj_origin_data[1] - m_adc_i_offset_origin_data[1];
    result = temp * (float)(3.3f / 4.0960f / 0.12f);
    result *= 0.001f;
    m_adc_physical_value[ADC_CH_V_I] = result;

    // W_I
    temp = m_adc_inj_origin_data[2] - m_adc_i_offset_origin_data[2];
    result = temp * (float)(3.3f / 4.0960f / 0.12f);
    result *= 0.001f;
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
#endif
        }

        adc_reg_origin_data_to_phy_value();     //采样数据转换物理数据
    }

#if 0
    if(m_test_ticks >= 1000)
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
//        gpio_output_set(DSP_RELAY_IGBT_PORT, DSP_RELAY_IGBT_PIN, 1);

        m_adc_inj_origin_data[0] = HAL_ADCEx_InjectedGetValue(&g_adc3_handle, ADC_INJECTED_RANK_1); //U电流
        m_adc_inj_origin_data[1] = HAL_ADCEx_InjectedGetValue(&g_adc3_handle, ADC_INJECTED_RANK_2); //V电流
        m_adc_inj_origin_data[2] = HAL_ADCEx_InjectedGetValue(&g_adc3_handle, ADC_INJECTED_RANK_3); //W电流

        adc_inj_data_to_physical_value();

        m_test_ticks++;

//        if((g_app_param.motor_sta == MOTOR_STA_STARTING) || (g_app_param.motor_sta == MOTOR_STA_RUNNING))     //电机在非停机状态下都要运行
        {

//EKF
#if 0
            //motor_run();
#endif

//VF
#if 1
            g_app_param.curr_theta += g_app_param.target_step_angle;
            g_app_param.curr_theta = radian_normalize(g_app_param.curr_theta);
            motor_vf_run();
#endif

//IF
#if 0
            motor_if_run();
#endif

        }

//        gpio_output_set(DSP_RELAY_IGBT_PORT, DSP_RELAY_IGBT_PIN, 0);
//        gpio_output_set(DSP_DRIVE_IGBT_PORT, DSP_DRIVE_IGBT_PIN, 0);
    }
}

