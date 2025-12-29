#ifndef ADC_H__
#define ADC_H__

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