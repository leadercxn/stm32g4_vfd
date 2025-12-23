#include "gpio.h"

#include "boards.h"


int bsp_gpio_init(void)
{
    GPIO_InitTypeDef gpio_init_struct = { 0 };

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    gpio_init_struct.Pin     = TEST0_IO_PIN | TEST1_IO_PIN ;
    gpio_init_struct.Mode    = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull    = GPIO_NOPULL;
    gpio_init_struct.Speed   = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TEST0_IO_PORT, &gpio_init_struct);

    //input
    gpio_init_struct.Pin     = DSP_X1_STARTUP_PIN | DSP_X2_RST_PIN | DSP_IGBT_FLT_PIN | DSP_EB_WU_ERR_PIN | DSP_EA_VU_ERR_PIN | DSP_UVW_PHASE_LOSS_PIN;
    gpio_init_struct.Mode    = GPIO_MODE_INPUT;
    gpio_init_struct.Pull    = GPIO_NOPULL;
    gpio_init_struct.Speed   = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);

    //output
    gpio_init_struct.Pin     = DSP_ERR_RELAY_PIN | DSP_LED_GREEN_PIN | DSP_LED_ERR_PIN ;
    gpio_init_struct.Mode    = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull    = GPIO_NOPULL;
    gpio_init_struct.Speed   = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);

    gpio_init_struct.Pin     = DSP_RELAY_IGBT_PIN | DSP_DRIVE_IGBT_PIN ;
    gpio_init_struct.Mode    = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull    = GPIO_NOPULL;
    gpio_init_struct.Speed   = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);

    gpio_init_struct.Pin     = FLASH_CSI_PIN ;
    gpio_init_struct.Mode    = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull    = GPIO_NOPULL;
    gpio_init_struct.Speed   = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    gpio_output_set(DSP_LED_ERR_PORT, DSP_LED_ERR_PIN, 1);  //关闭 ERR LED

    return 0;
}

void gpio_output_set(uint32_t gpio_port, uint32_t gpio_pin, uint8_t value)
{
    if(value)
    {
        HAL_GPIO_WritePin((GPIO_TypeDef *)gpio_port, gpio_pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin((GPIO_TypeDef *)gpio_port, gpio_pin, GPIO_PIN_RESET);
    }
}

int gpio_input_get(uint32_t gpio_port, uint32_t gpio_pin)
{
    return HAL_GPIO_ReadPin((GPIO_TypeDef *)gpio_port, gpio_pin);
}
