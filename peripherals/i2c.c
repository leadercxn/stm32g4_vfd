#include "string.h"
#include "boards.h"
#include "i2c.h"
#include "parameters.h"

#include "trace.h"

extern void Error_Handler(void);

I2C_HandleTypeDef g_i2c3_handle;

int i2c3_init(void)
{
  g_i2c3_handle.Instance = I2C3;
  g_i2c3_handle.Init.Timing = 0x2000090E;
  g_i2c3_handle.Init.OwnAddress1 = 0;
  g_i2c3_handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  g_i2c3_handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  g_i2c3_handle.Init.OwnAddress2 = 0;
  g_i2c3_handle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  g_i2c3_handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  g_i2c3_handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&g_i2c3_handle) != HAL_OK)
  {
    Error_Handler();
  }

#if 0
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&g_i2c3_handle, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&g_i2c3_handle, 0) != HAL_OK)
  {
    Error_Handler();
  }
#endif
  return 0;
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef gpio_init_struct = {0};

  if(i2cHandle->Instance == I2C3)
  {
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /**
     * I2C3 GPIO Configuration
        PC8     ------> I2C3_SCL
        PC9     ------> I2C3_SDA
    */

    gpio_init_struct.Pin = DSP_I2C_SCLA_PIN | DSP_I2C_SDAA_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_OD;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init_struct.Alternate = GPIO_AF8_I2C3;
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);

    __HAL_RCC_I2C3_CLK_ENABLE();
  }
}

uint8_t i2c3_at24cxx_one_byte_read(uint16_t reg)
{
    uint8_t rxdata = 0;

    HAL_I2C_Mem_Read(&g_i2c3_handle, AT24CXX_DEV_ADDR, reg, I2C_MEMADD_SIZE_16BIT, &rxdata, 1, 100);

    return rxdata;
}

void i2c3_at24cxx_one_byte_write(uint16_t reg, uint8_t txdata)
{
    HAL_I2C_Mem_Write(&g_i2c3_handle, AT24CXX_DEV_ADDR, reg, I2C_MEMADD_SIZE_16BIT, &txdata, 1, 100);
}

void i2c3_at24cxx_read(uint16_t reg, uint8_t *p_rx_data, uint16_t len)
{
    while(len)
	{
		*p_rx_data++ = i2c3_at24cxx_one_byte_read(reg++);
		len--;
	}
}

void i2c3_at24cxx_write(uint16_t reg, uint8_t *p_tx_data, uint16_t len)
{
    while(len--)
	{
		i2c3_at24cxx_one_byte_write(reg, *p_tx_data);
		reg++;
		p_tx_data++;
	}
}
