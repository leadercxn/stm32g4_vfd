#include "string.h"
#include "boards.h"
#include "adc.h"
#include "math.h"
#include "parameters.h"

#include "trace.h"

extern void Error_Handler(void);

static ADC_HandleTypeDef m_adc1_handle;
ADC_HandleTypeDef g_adc3_handle;

/**
 * adc初始化函数
 */
int adc1_init(void)
{
    ADC_ChannelConfTypeDef      channle_cfg = {0};  //规则通道
    int     err_code = 0;

    m_adc1_handle.Instance                      = ADC1;
    m_adc1_handle.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;
    m_adc1_handle.Init.Resolution               = ADC_RESOLUTION_12B;
    m_adc1_handle.Init.DataAlign                = ADC_DATAALIGN_RIGHT;
    m_adc1_handle.Init.GainCompensation         = 0;
    m_adc1_handle.Init.ScanConvMode             = ADC_SCAN_DISABLE;

    m_adc1_handle.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
    m_adc1_handle.Init.LowPowerAutoWait         = DISABLE;
    m_adc1_handle.Init.ContinuousConvMode       = DISABLE;
    m_adc1_handle.Init.NbrOfConversion          = 1;
    m_adc1_handle.Init.DiscontinuousConvMode    = DISABLE;
    m_adc1_handle.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
    m_adc1_handle.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
    m_adc1_handle.Init.DMAContinuousRequests    = DISABLE;
    m_adc1_handle.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
    m_adc1_handle.Init.OversamplingMode         = DISABLE;

    if (HAL_ADC_Init(&m_adc1_handle) != HAL_OK)
    {
        Error_Handler();
    }


    //规则通道
    channle_cfg.Channel      = ADC_CHANNEL_6;
    channle_cfg.Rank         = ADC_REGULAR_RANK_1;
    channle_cfg.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    channle_cfg.SingleDiff   = ADC_SINGLE_ENDED;
    channle_cfg.OffsetNumber = ADC_OFFSET_NONE;
    channle_cfg.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&m_adc1_handle, &channle_cfg) != HAL_OK)
    {
        Error_Handler();
    }

#if 0
    channle_cfg.Channel = ADC_CHANNEL_7;
    channle_cfg.Rank    = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&m_adc1_handle, &channle_cfg) != HAL_OK)
    {
        Error_Handler();
    }

    channle_cfg.Channel = ADC_CHANNEL_8;
    channle_cfg.Rank    = ADC_REGULAR_RANK_3;
    if (HAL_ADC_ConfigChannel(&m_adc1_handle, &channle_cfg) != HAL_OK)
    {
        Error_Handler();
    }

    channle_cfg.Channel = ADC_CHANNEL_9;
    channle_cfg.Rank    = ADC_REGULAR_RANK_4;
    if (HAL_ADC_ConfigChannel(&m_adc1_handle, &channle_cfg) != HAL_OK)
    {
        Error_Handler();
    }
#endif

    err_code = HAL_ADCEx_Calibration_Start(&m_adc1_handle, ADC_SINGLE_ENDED);   //ADC 启动前自校准

//    HAL_ADC_Start(&m_adc1_handle);

    return err_code;
}

int adc3_init(void)
{
    ADC_ChannelConfTypeDef      channle_cfg = {0};  //规则通道
    int     err_code = 0;

    g_adc3_handle.Instance                      = ADC3;
    g_adc3_handle.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;
    g_adc3_handle.Init.Resolution               = ADC_RESOLUTION_12B;
    g_adc3_handle.Init.DataAlign                = ADC_DATAALIGN_RIGHT;
    g_adc3_handle.Init.GainCompensation         = 0;
    g_adc3_handle.Init.ScanConvMode             = ADC_SCAN_ENABLE;
    g_adc3_handle.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
    g_adc3_handle.Init.LowPowerAutoWait         = DISABLE;
    g_adc3_handle.Init.ContinuousConvMode       = DISABLE;
    g_adc3_handle.Init.NbrOfConversion          = 1;
    g_adc3_handle.Init.DiscontinuousConvMode    = DISABLE;
    g_adc3_handle.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
    g_adc3_handle.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
    g_adc3_handle.Init.DMAContinuousRequests    = DISABLE;
    g_adc3_handle.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
    g_adc3_handle.Init.OversamplingMode         = DISABLE;

    if (HAL_ADC_Init(&g_adc3_handle) != HAL_OK)
    {
        Error_Handler();
    }

    //规则通道
    channle_cfg.Channel      = ADC_CHANNEL_7;
    channle_cfg.Rank         = ADC_REGULAR_RANK_1;
    channle_cfg.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    channle_cfg.SingleDiff   = ADC_SINGLE_ENDED;
    channle_cfg.OffsetNumber = ADC_OFFSET_NONE;
    channle_cfg.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&g_adc3_handle, &channle_cfg) != HAL_OK)
    {
        Error_Handler();
    }

#if 0
    channle_cfg.Channel = ADC_CHANNEL_11;
    channle_cfg.Rank    = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&g_adc3_handle, &channle_cfg) != HAL_OK)
    {
        Error_Handler();
    }
#endif


    // 配置注入通道
    ADC_InjectionConfTypeDef injection_ch_cfg = {0};

    injection_ch_cfg.InjectedChannel                = ADC_CHANNEL_8;
    injection_ch_cfg.InjectedRank                   = ADC_INJECTED_RANK_1;
    injection_ch_cfg.InjectedSamplingTime           = ADC_SAMPLETIME_2CYCLES_5;
    injection_ch_cfg.InjectedSingleDiff             = ADC_SINGLE_ENDED;
    injection_ch_cfg.InjectedOffsetNumber           = ADC_OFFSET_NONE;
    injection_ch_cfg.InjectedOffset                 = 0;
    injection_ch_cfg.InjectedNbrOfConversion        = 3;
    injection_ch_cfg.InjectedDiscontinuousConvMode  = DISABLE;
    injection_ch_cfg.AutoInjectedConv               = DISABLE;
    injection_ch_cfg.QueueInjectedContext           = DISABLE;
    injection_ch_cfg.ExternalTrigInjecConv          = ADC_INJECTED_SOFTWARE_START;
    injection_ch_cfg.ExternalTrigInjecConvEdge      = ADC_EXTERNALTRIGINJECCONV_EDGE_NONE;

//    injection_ch_cfg.ExternalTrigInjecConv          = ADC_EXTERNALTRIGINJEC_T1_CC4;          // 不知为何，T1 CH4 总是触发不了 ADC 注入通道的启动
//    injection_ch_cfg.ExternalTrigInjecConvEdge      = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    injection_ch_cfg.InjecOversamplingMode          = DISABLE;
    if (HAL_ADCEx_InjectedConfigChannel(&g_adc3_handle, &injection_ch_cfg) != HAL_OK)
    {
        Error_Handler();
    }

    injection_ch_cfg.InjectedChannel  = ADC_CHANNEL_9;
    injection_ch_cfg.InjectedRank     = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&g_adc3_handle, &injection_ch_cfg) != HAL_OK)
    {
        trace_error("ADC3 Inj Rank2 Config Error\r\n");

        Error_Handler();
    }

    injection_ch_cfg.InjectedChannel  = ADC_CHANNEL_10;
    injection_ch_cfg.InjectedRank     = ADC_INJECTED_RANK_3;
    if (HAL_ADCEx_InjectedConfigChannel(&g_adc3_handle, &injection_ch_cfg) != HAL_OK)
    {
        trace_error("ADC3 Inj Rank3 Config Error\r\n");

        Error_Handler();
    }

    err_code = HAL_ADCEx_Calibration_Start(&g_adc3_handle, ADC_SINGLE_ENDED);   //ADC 启动前自校准

//    HAL_ADC_Start(&g_adc3_handle);

    return err_code;
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{
  GPIO_InitTypeDef          gpio_init_struct = {0};
  RCC_PeriphCLKInitTypeDef  periph_clk_init = {0};

  if(adcHandle->Instance == ADC1)
  {
    periph_clk_init.PeriphClockSelection    = RCC_PERIPHCLK_ADC12;
    periph_clk_init.Adc12ClockSelection     = RCC_ADC12CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&periph_clk_init) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_ADC12_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio_init_struct.Pin = DSP_ADCB0_PIM_T_PIN | DSP_ADCB1_RAD_T_PIN | DSP_ADCB2_VCC_VOLT_PIN | DSP_ADCB4_BOX_T_PIN;
    gpio_init_struct.Mode = GPIO_MODE_ANALOG;
    gpio_init_struct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);
  }
  else if(adcHandle->Instance == ADC3)
  {
    periph_clk_init.PeriphClockSelection    = RCC_PERIPHCLK_ADC345;
    periph_clk_init.Adc12ClockSelection     = RCC_ADC345CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&periph_clk_init) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_ADC345_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();

    gpio_init_struct.Pin = DSP_ADCA0_BASE_VOLT_PIN | DSP_ADCA2_IU_PIN | DSP_ADCA4_IV_PIN | DSP_ADCA6_IW_PIN | DSP_ADCA7_UBUS_VOLT_PIN;
    gpio_init_struct.Mode = GPIO_MODE_ANALOG;
    gpio_init_struct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);

    HAL_NVIC_SetPriority(ADC3_IRQn, 2, 1);
    HAL_NVIC_EnableIRQ(ADC3_IRQn);
  }
}

/**
 * @brief       ADC 采集中断服务函数
 * @param       无
 * @retval      无
 */
void ADC1_2_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&m_adc1_handle);
}

void ADC3_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&g_adc3_handle);
}

uint16_t adc1_ch6_val_get(void)
{
    uint16_t val = 0;
    ADC_ChannelConfTypeDef      channle_cfg = {0};  //规则通道

	channle_cfg.Channel      = ADC_CHANNEL_6;
    channle_cfg.Rank         = ADC_REGULAR_RANK_1;
    channle_cfg.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
    if (HAL_ADC_ConfigChannel(&m_adc1_handle, &channle_cfg) != HAL_OK)
    {
        Error_Handler();
    }

	HAL_ADC_Start(&m_adc1_handle);
	HAL_ADC_PollForConversion(&m_adc1_handle, 100);
	val = HAL_ADC_GetValue(&m_adc1_handle);

    return val;
}

uint16_t adc1_ch7_val_get(void)
{
    uint16_t val = 0;
    ADC_ChannelConfTypeDef      channle_cfg = {0};  //规则通道

	channle_cfg.Channel      = ADC_CHANNEL_7;
    channle_cfg.Rank         = ADC_REGULAR_RANK_1;
    channle_cfg.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
    if (HAL_ADC_ConfigChannel(&m_adc1_handle, &channle_cfg) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADC_Start(&m_adc1_handle);
	HAL_ADC_PollForConversion(&m_adc1_handle, 100);
	val = HAL_ADC_GetValue(&m_adc1_handle);

    return val;
}

uint16_t adc1_ch8_val_get(void)
{
    uint16_t val = 0;
    ADC_ChannelConfTypeDef      channle_cfg = {0};  //规则通道

	channle_cfg.Channel      = ADC_CHANNEL_8;
    channle_cfg.Rank         = ADC_REGULAR_RANK_1;
    channle_cfg.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
    if (HAL_ADC_ConfigChannel(&m_adc1_handle, &channle_cfg) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADC_Start(&m_adc1_handle);
	HAL_ADC_PollForConversion(&m_adc1_handle, 100);
	val = HAL_ADC_GetValue(&m_adc1_handle);

    return val;
}

uint16_t adc1_ch9_val_get(void)
{
    uint16_t val = 0;
    ADC_ChannelConfTypeDef      channle_cfg = {0};  //规则通道

	channle_cfg.Channel      = ADC_CHANNEL_9;
    channle_cfg.Rank         = ADC_REGULAR_RANK_1;
    channle_cfg.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
    if (HAL_ADC_ConfigChannel(&m_adc1_handle, &channle_cfg) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADC_Start(&m_adc1_handle);
	HAL_ADC_PollForConversion(&m_adc1_handle, 100);
	val = HAL_ADC_GetValue(&m_adc1_handle);

    return val;
}

uint16_t adc3_ch7_val_get(void)
{
    uint16_t val = 0;
    ADC_ChannelConfTypeDef      channle_cfg = {0};  //规则通道

	channle_cfg.Channel      = ADC_CHANNEL_7;
    channle_cfg.Rank         = ADC_REGULAR_RANK_1;
    channle_cfg.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
    if (HAL_ADC_ConfigChannel(&g_adc3_handle, &channle_cfg) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADC_Start(&g_adc3_handle);
	HAL_ADC_PollForConversion(&g_adc3_handle, 100);
	val = HAL_ADC_GetValue(&g_adc3_handle);

    return val;
}

uint16_t adc3_ch11_val_get(void)
{
    uint16_t val = 0;
    ADC_ChannelConfTypeDef      channle_cfg = {0};  //规则通道

	channle_cfg.Channel      = ADC_CHANNEL_11;
    channle_cfg.Rank         = ADC_REGULAR_RANK_1;
    channle_cfg.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
    if (HAL_ADC_ConfigChannel(&g_adc3_handle, &channle_cfg) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADC_Start(&g_adc3_handle);
	HAL_ADC_PollForConversion(&g_adc3_handle, 100);
	val = HAL_ADC_GetValue(&g_adc3_handle);

    return val;
}


void adc3_inj_start(void)
{
    HAL_ADCEx_InjectedStart_IT(&g_adc3_handle);     //启动ADC1注入通道转换
}

void adc3_inj_stop(void)
{
    HAL_ADCEx_InjectedStop_IT(&g_adc3_handle);
}







