#include "string.h"
#include "boards.h"
#include "spi.h"
#include "parameters.h"

#include "trace.h"

extern void Error_Handler(void);

SPI_HandleTypeDef g_spi1_handle;

int spi1_init(void)
{
    g_spi1_handle.Instance              = SPI1;                       /* SPI1 */
    g_spi1_handle.Init.Mode             = SPI_MODE_MASTER;            /* 设置SPI工作模式，设置为主模式 */
    g_spi1_handle.Init.Direction        = SPI_DIRECTION_2LINES;       /* 设置SPI单向或者双向的数据模式:SPI设置为双线模式 */
    g_spi1_handle.Init.DataSize         = SPI_DATASIZE_8BIT;          /* 设置SPI的数据大小:SPI发送接收8位帧结构 */
    g_spi1_handle.Init.CLKPolarity      = SPI_POLARITY_LOW;          /* 串行同步时钟的空闲状态为低电平 */
    g_spi1_handle.Init.CLKPhase         = SPI_PHASE_1EDGE;            /* 串行同步时钟的第二个跳变沿（上升或下降）数据被采样 */
    g_spi1_handle.Init.NSS              = SPI_NSS_SOFT;                 /* NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制 */
    g_spi1_handle.Init.BaudRatePrescaler    = SPI_BAUDRATEPRESCALER_8;   /* 定义波特率预分频的值:波特率预分频值为4 */
    g_spi1_handle.Init.FirstBit         = SPI_FIRSTBIT_MSB;           /* 指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始 */
    g_spi1_handle.Init.TIMode           = SPI_TIMODE_DISABLE;         /* 关闭TI模式 */
    g_spi1_handle.Init.CRCCalculation   = SPI_CRCCALCULATION_DISABLE; /* 关闭硬件CRC校验 */
    g_spi1_handle.Init.CRCPolynomial    = 10;                         /* CRC值计算的多项式 */

    if (HAL_SPI_Init(&g_spi1_handle) != HAL_OK)
    {
        Error_Handler();
    }

//    __HAL_SPI_ENABLE(&g_spi1_handle);                                 /* 使能SPI1 */
//    spi1_one_byte_wr(0xFF);     /* 启动传输, 实际上就是产生8个时钟脉冲, 达到清空DR的作用, 非必需 */

    return 0;
}

/**
 * @brief       SPI底层驱动，时钟使能，引脚配置
 * @note        此函数会被HAL_SPI_Init()调用
 * @param       hspi:SPI句柄
 * @retval      无
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    if (hspi->Instance == SPI1)
    {
        __HAL_RCC_SPI1_CLK_ENABLE();

        /* 使能 IO 时钟 */
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /**/
        gpio_init_struct.Pin    = FLASH_CLK_PIN | FLASH_MOSI_PIN | FLASH_MISO_PIN;
        gpio_init_struct.Mode   = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull   = GPIO_NOPULL;
        gpio_init_struct.Speed  = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);
    }
}

/**
 * @brief       SPI1读写一个字节数据
 * @param       txdata  : 要发送的数据(1字节)
 * @retval      接收到的数据(1字节)
 */
uint8_t spi1_one_byte_wr(uint8_t txdata)
{
    uint8_t rxdata;

    HAL_SPI_TransmitReceive(&g_spi1_handle, &txdata, &rxdata, 1, 100);

    return rxdata; /* 返回收到的数据 */
}

int spi1_bytes_wr(uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t len)
{
    HAL_SPI_TransmitReceive(&g_spi1_handle, p_tx_data, p_rx_data, len, 100);

    return 0;
}

void w25nxx_select(void)
{
    HAL_GPIO_WritePin(FLASH_CSI_PORT, FLASH_CSI_PIN, GPIO_PIN_RESET);
}

void w25nxx_disselect(void)
{
    HAL_GPIO_WritePin(FLASH_CSI_PORT, FLASH_CSI_PIN, GPIO_PIN_SET);
}