#ifndef ADC_H__
#define ADC_H__

typedef enum
{
    ADC_CH_PIM_T,
    ADC_CH_RAD_T,
    ADC_CH_VCC_VOLT,
    ADC_CH_BOX_T,

    ADC_CH_BASE_VOLT,
    ADC_CH_U_I,
    ADC_CH_V_I,
    ADC_CH_W_I,
    ADC_CH_UBUS_VOLT,

    ADC_CH_MAX,
} adc_channel_e;

extern ADC_HandleTypeDef g_adc3_handle;

int adc1_init(void);
int adc3_init(void);

uint16_t adc1_ch6_val_get(void);
uint16_t adc1_ch7_val_get(void);
uint16_t adc1_ch8_val_get(void);
uint16_t adc1_ch9_val_get(void);
uint16_t adc3_ch7_val_get(void);
uint16_t adc3_ch11_val_get(void);

void adc3_inj_start(void);
void adc3_inj_stop(void);

#endif