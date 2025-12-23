#include "string.h"
#include "boards.h"
#include "virt_i2c.h"
#include "parameters.h"
#include "delay.h"

#include "trace.h"

static void virt_i2c_delay(void)
{
    delay_us(5);
}

void virt_i2c_start(void)
{
    gpio_output_set(DSP_I2C_SDAA_PORT, DSP_I2C_SDAA_PIN, 1);
    gpio_output_set(DSP_I2C_SCLA_PORT, DSP_I2C_SCLA_PIN, 1);
    virt_i2c_delay();
    gpio_output_set(DSP_I2C_SDAA_PORT, DSP_I2C_SDAA_PIN, 0);
    virt_i2c_delay();
    gpio_output_set(DSP_I2C_SDAA_PORT, DSP_I2C_SCLA_PIN, 0);
    virt_i2c_delay();
}

void virt_i2c_stop(void)
{
    gpio_output_set(DSP_I2C_SDAA_PORT, DSP_I2C_SDAA_PIN, 0);
    virt_i2c_delay();
    gpio_output_set(DSP_I2C_SCLA_PORT, DSP_I2C_SCLA_PIN, 1);
    virt_i2c_delay();
    gpio_output_set(DSP_I2C_SDAA_PORT, DSP_I2C_SDAA_PIN, 1);
    virt_i2c_delay();
}

void virt_i2c_wait_ack(void)
{
    uint8_t waittime = 0;
    uint8_t rack = 0;

    gpio_output_set(DSP_I2C_SDAA_PORT, DSP_I2C_SDAA_PIN, 1);
    virt_i2c_delay();
    gpio_output_set(DSP_I2C_SCLA_PORT, DSP_I2C_SCLA_PIN, 1);
    virt_i2c_delay();
    
    //sda 转 input
    while( HAL_GPIO_ReadPin(DSP_I2C_SDAA_PORT, DSP_I2C_SDAA_PIN) )    /* 等待应答 */
    {
        waittime++;

        if (waittime > 250)
        {
            virt_i2c_stop();
            rack = 1;
            break;
        }
    }

    gpio_output_set(DSP_I2C_SCLA_PORT, DSP_I2C_SCLA_PIN, 0);
    virt_i2c_delay();

    return rack;
}

void virt_i2c_ack(void)
{
    gpio_output_set(DSP_I2C_SDAA_PORT, DSP_I2C_SDAA_PIN, 0);
    virt_i2c_delay();
    gpio_output_set(DSP_I2C_SCLA_PORT, DSP_I2C_SCLA_PIN, 1);
    virt_i2c_delay();

    gpio_output_set(DSP_I2C_SCLA_PORT, DSP_I2C_SCLA_PIN, 0);
    virt_i2c_delay();
    gpio_output_set(DSP_I2C_SDAA_PORT, DSP_I2C_SDAA_PIN, 1);
    virt_i2c_delay();
}

void virt_i2c_nack(void)
{
    gpio_output_set(DSP_I2C_SDAA_PORT, DSP_I2C_SDAA_PIN, 1);
    virt_i2c_delay();
    gpio_output_set(DSP_I2C_SCLA_PORT, DSP_I2C_SCLA_PIN, 1);
    virt_i2c_delay();

    gpio_output_set(DSP_I2C_SCLA_PORT, DSP_I2C_SCLA_PIN, 0);
    virt_i2c_delay();
}

void virt_i2c_send_byte(uint8_t txdata)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        gpio_output_set(DSP_I2C_SDAA_PORT, DSP_I2C_SDAA_PIN, ((txdata & 0x80) >> 7));
        virt_i2c_delay();
        gpio_output_set(DSP_I2C_SCLA_PORT, DSP_I2C_SCLA_PIN, 1);
        virt_i2c_delay();
        gpio_output_set(DSP_I2C_SCLA_PORT, DSP_I2C_SCLA_PIN, 0);

        txdata <<= 1;
    }

    gpio_output_set(DSP_I2C_SDAA_PORT, DSP_I2C_SDAA_PIN, 1);
}

uint8_t virt_i2c_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0;

    for (i = 0; i < 8; i++ )    /* 接收1个字节数据 */
    {
        receive <<= 1;  /* 高位先输出,所以先收到的数据位要左移 */
        gpio_output_set(DSP_I2C_SCLA_PORT, DSP_I2C_SCLA_PIN, 1);
        virt_i2c_delay();

        if( HAL_GPIO_ReadPin(DSP_I2C_SDAA_PORT, DSP_I2C_SDAA_PIN) )
        {
            receive++;
        }
        
        gpio_output_set(DSP_I2C_SCLA_PORT, DSP_I2C_SCLA_PIN, 0);
        virt_i2c_delay();
    }

    if (!ack)
    {
        virt_i2c_ack();     /* 发送nACK */
    }
    else
    {
        virt_i2c_nack();      /* 发送ACK */
    }

    return receive;
}

void virt_i2c_io_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    __HAL_RCC_GPIOC_CLK_ENABLE();


    gpio_init_struct.Pin = DSP_I2C_SCLA_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;        /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* 快速 */
    HAL_GPIO_Init(DSP_I2C_SCLA_PORT, &gpio_init_struct);/* SCL */

    gpio_init_struct.Pin = DSP_I2C_SDAA_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;        /* 开漏输出 */
    HAL_GPIO_Init(DSP_I2C_SDAA_PORT, &gpio_init_struct);/* SDA */
    /* SDA引脚模式设置,开漏输出,上拉, 这样就不用再设置IO方向了, 开漏输出的时候(=1), 也可以读取外部信号的高低电平 */

    iic_stop();     /* 停止总线上所有设备 */
}





