#ifndef BSP_STM32G474RE_GEN1_V1_H__
#define BSP_STM32G474RE_GEN1_V1_H__

#include "stdint.h"
#include "stdbool.h"
#include "stm32g4xx_hal.h"

#define _RAM_FUNC                   __attribute__ ((section (".ramfunc")))

#define MACHINE_NAME                "STM32G474_VFD_GEN1"

//辅助IO，用来监控adc电流采样的时间点
#define TEST0_IO_PORT               GPIOA
#define TEST0_IO_PIN                GPIO_PIN_0
#define TEST1_IO_PORT               GPIOA
#define TEST1_IO_PIN                GPIO_PIN_1

//Flash SPI 接口定义
#define FLASH_CSI_PORT              GPIOA
#define FLASH_CSI_PIN               GPIO_PIN_4
#define FLASH_CLK_PORT              GPIOA
#define FLASH_CLK_PIN               GPIO_PIN_5
#define FLASH_MISO_PORT             GPIOA
#define FLASH_MISO_PIN              GPIO_PIN_6
#define FLASH_MOSI_PORT             GPIOA
#define FLASH_MOSI_PIN              GPIO_PIN_7

//I2C
#define DSP_I2C_SCLA_PORT           GPIOC
#define DSP_I2C_SCLA_PIN            GPIO_PIN_8
#define DSP_I2C_SDAA_PORT           GPIOC
#define DSP_I2C_SDAA_PIN            GPIO_PIN_9

//UART
#define USART1_TX0_232_PORT         GPIOA
#define USART1_TX0_232_PIN          GPIO_PIN_9
#define USART1_RX0_232_PORT         GPIOA
#define USART1_RX0_232_PIN          GPIO_PIN_10
#define USART1_PIN_AF               GPIO_AF7_USART1

#define USART2_SCITX_485_PORT       GPIOD
#define USART2_SCITX_485_PIN        GPIO_PIN_5
#define USART2_SCIRX_485_PORT       GPIOD
#define USART2_SCIRX_485_PIN        GPIO_PIN_6
#define USART2_PIN_AF               GPIO_AF7_USART2

#define USART2_RD_TX_DIR_PORT       GPIOD
#define USART2_RD_TX_DIR_PIN        GPIO_PIN_4

#define USART1_BAUDRATE             576000          // 921000太快了，232通信不稳定
#define USART2_BAUDRATE             115200

//INPUT
#define DSP_X1_STARTUP_PORT         GPIOB
#define DSP_X1_STARTUP_PIN          GPIO_PIN_10    //启动信号输入
#define DSP_X2_RST_PORT             GPIOB
#define DSP_X2_RST_PIN              GPIO_PIN_11    //复位信号输入
#define DSP_IGBT_FLT_PORT           GPIOB
#define DSP_IGBT_FLT_PIN            GPIO_PIN_12    //IGBT模块故障输入
#define DSP_EB_WU_ERR_PORT          GPIOB
#define DSP_EB_WU_ERR_PIN           GPIO_PIN_13    //WU相故障反馈输入
#define DSP_EA_VU_ERR_PORT          GPIOB
#define DSP_EA_VU_ERR_PIN           GPIO_PIN_14    //VU相故障反馈输入
#define DSP_UVW_PHASE_LOSS_PORT     GPIOB
#define DSP_UVW_PHASE_LOSS_PIN      GPIO_PIN_15    //UVW缺相故障输入

//OUTPUT
#define DSP_ERR_RELAY_PORT          GPIOB
#define DSP_ERR_RELAY_PIN           GPIO_PIN_5      //对外故障继电器输出

#define DSP_LED_GREEN_PORT          GPIOB
#define DSP_LED_GREEN_PIN           GPIO_PIN_6      //绿色LED运行指示灯
#define DSP_LED_ERR_PORT            GPIOB
#define DSP_LED_ERR_PIN             GPIO_PIN_7      //LED故障指示灯

#define DSP_RELAY_IGBT_PORT         GPIOE
#define DSP_RELAY_IGBT_PIN          GPIO_PIN_4      //IGBT板上继电器控制输出 -- 13
#define DSP_DRIVE_IGBT_PORT         GPIOE
#define DSP_DRIVE_IGBT_PIN          GPIO_PIN_5      //IGBT驱动光耦电源信号, 硬件输出电平取反 -- 29

//ADC
#define DSP_ADCB0_PIM_T_PORT        GPIOC
#define DSP_ADCB0_PIM_T_PIN         GPIO_PIN_0      //IGBT 温度检测  adc12-IN6 -- 32
#define DSP_ADCB1_RAD_T_PORT        GPIOC
#define DSP_ADCB1_RAD_T_PIN         GPIO_PIN_1      //散热片温度检测  adc12-IN7 -- JX1
#define DSP_ADCB2_VCC_VOLT_PORT     GPIOC
#define DSP_ADCB2_VCC_VOLT_PIN      GPIO_PIN_2      //变压器母线电压检测 - 5V电压 adc12-IN8  -- 31
#define DSP_ADCB4_BOX_T_PORT        GPIOC
#define DSP_ADCB4_BOX_T_PIN         GPIO_PIN_3      //机箱温度检测 adc12-IN9   -- 内部

#define DSP_ADCA0_BASE_VOLT_PORT    GPIOD
#define DSP_ADCA0_BASE_VOLT_PIN     GPIO_PIN_10     //基准电压检测 adc345-IN7   -- 内部
#define DSP_ADCA2_IU_PORT           GPIOD
#define DSP_ADCA2_IU_PIN            GPIO_PIN_11     //U相电流检测  adc345-IN8   -- 1
#define DSP_ADCA4_IV_PORT           GPIOD
#define DSP_ADCA4_IV_PIN            GPIO_PIN_12     //V相电流检测  adc345-IN9   -- 2
#define DSP_ADCA6_IW_PORT           GPIOD
#define DSP_ADCA6_IW_PIN            GPIO_PIN_13     //W相电流检测  adc345-IN10  -- 3
#define DSP_ADCA7_UBUS_VOLT_PORT    GPIOD
#define DSP_ADCA7_UBUS_VOLT_PIN     GPIO_PIN_14     //电阻分压式母线电压检测, 高压母线电压 -- adc345-IN11 -- 30

//PWM
#define PWM_UH_PORT                 GPIOE
#define PWM_UH_PIN                  GPIO_PIN_9   //UH - TIM1_CH1
#define PWM_UL_PORT                 GPIOE
#define PWM_UL_PIN                  GPIO_PIN_8   //UL - TIM1_CH1N

#define PWM_VH_PORT                 GPIOE
#define PWM_VH_PIN                  GPIO_PIN_11  //VH - TIM1_CH2
#define PWM_VL_PORT                 GPIOE
#define PWM_VL_PIN                  GPIO_PIN_10  //VL - TIM1_CH2N

#define PWM_WH_PORT                 GPIOE
#define PWM_WH_PIN                  GPIO_PIN_13  //WH - TIM1_CH3
#define PWM_WL_PORT                 GPIOE
#define PWM_WL_PIN                  GPIO_PIN_12  //WL - TIM1_CH3N

#endif

