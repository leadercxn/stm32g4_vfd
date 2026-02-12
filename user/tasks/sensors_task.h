#ifndef SENSORS_TASK_H__
#define SENSORS_TASK_H__

typedef enum
{
    ADC_CH_PIM_IGBT_T,      // IGBT 模块自带的温度传感器
    ADC_CH_RAD_T,           // 另外需要安装在铝块散热片上的温度传感器, 预留
    ADC_CH_VCC_VOLT,        // 变压器输出电压
    ADC_CH_CTL_BSP_T,       // 控制板载温度传感器

    ADC_CH_BASE_VOLT,
    ADC_CH_U_I,
    ADC_CH_V_I,
    ADC_CH_W_I,
    ADC_CH_UBUS_VOLT,

    ADC_CH_MAX,
} adc_channel_e;

float adc_sample_physical_value_get(adc_channel_e ch);
float rms_curr_get(adc_channel_e ch);
float rms_curr_aver_get(adc_channel_e ch);

int sensors_task(void);

#endif
