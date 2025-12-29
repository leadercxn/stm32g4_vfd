#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "math.h"

#include "boards.h"
#include "sys.h"
#include "delay.h"
#include "uart.h"
#include "gpio.h"
#include "timer.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "parameters.h"
#include "trace.h"
#include "app_timer.h"
#include "mid_timer.h"

#include "sensors_task.h"
#include "motor_ctrl_task.h"
#include "mb_slaver_task.h"
#include "monitor_task.h"

#include "at24cxx.h"

#include "foc.h"
//#include "ekf.h"

static void param_init(void)
{
    g_foc_input.tpwm  = PWM_TIM_PULSE_TPWM;

    g_mb_ctrl_param.speed_pid_p     = SPEED_PI_P;
    g_mb_ctrl_param.speed_pid_i     = SPEED_PI_I;
    g_mb_ctrl_param.speed_pid_kb    = SPEED_PI_KB;
    g_mb_ctrl_param.speed_pid_limit = SPEED_PI_UP_LIMIT;

    g_mb_ctrl_param.i_pid_p      = Q_PI_P;
    g_mb_ctrl_param.i_pid_i      = Q_PI_I;
    g_mb_ctrl_param.i_pid_kb     = Q_PI_KB;
    g_mb_ctrl_param.i_pid_limit  = Q_PI_UP_LIMIT;

    g_mb_ctrl_param.phase_rs     = MOTOR_PHASE_RES;   //相电阻
    g_mb_ctrl_param.phase_ls     = MOTOR_PHASE_LS;    //相电感
    g_mb_ctrl_param.flux_link    = MOTOR_FLUXLINK;    //磁链

    g_mb_ctrl_param.speed_max    = MOTOR_SPEED_MAX_RPM;
    g_mb_ctrl_param.speed_min    = MOTOR_SPEED_MIN_RPM;

    g_mb_ctrl_param.i_err_th          = 10.0f;            //过流阈值
    g_mb_ctrl_param.v_err_th          = VBUS_VLOT * 1.2f; //过压阈值

    g_mb_ctrl_param.pll_p = 600.0f;
    g_mb_ctrl_param.pll_i = 1500.0f;

    g_mb_ctrl_param.motor_pole_pairs  = MOTOR_POLE_PAIRS;

    foc_algorithm_init();
}

/**
 * 测试sinf, arm_sin计算的速度
 */
void sin_cal_speed_compare(void)
{
    trace_debug("sinf %.4f, arm_sin f %.4f, cosf %.4f, arm_cos f %.4f\r\n", \
    sinf(2.22f), arm_sin_f32(2.22f), cosf(1.56f), arm_cos_f32(1.56f));

    float temp = 0;
    uint32_t ticks = 0;
    uint32_t ticks_delta = 0;

    ticks = sys_time_ms_get();
    for(uint32_t i = 0; i < 100000; i++)
    {
      temp = sinf(2.22f);     //对比好像这个运行更快
    }
    ticks_delta = sys_time_ms_get() - ticks;
    trace_debug("sinf %lu\r\n", ticks_delta);

    ticks = sys_time_ms_get();
    for(uint32_t i = 0; i < 100000; i++)
    {
      temp = arm_sin_f32(2.22f);
    }
    ticks_delta = sys_time_ms_get() - ticks;
    trace_debug("arm_sin_f32 %lu\r\n", ticks_delta);
}

int main(void)
{
  int err_code = 0;

  bool led_stat = false;
  uint32_t test_inter_ticks = 0;

  uint32_t w25n_id = 0;

  HAL_Init();
  sys_stm32_clock_init(85, 2, 2, 4, 8);       /* 设置时钟,170Mhz */
  delay_init(170);                            /* 延时初始化 */

  //外设初始化
  bsp_gpio_init();  //普通型IO初始化
  usart1_init();    //usart1 初始化, 用于串口打印调试信息
  usart2_init();    //usart2 初始化, 用于 modbus 数据交互

  timer1_init();    //用于生成PWM

  if(adc1_init() != HAL_OK)
  {
      trace_error("ADC1 Init Error\r\n");
  }

  if(adc3_init() != HAL_OK)
  {
      trace_error("ADC3 Init Error\r\n");
  }

  i2c3_init();    //I2C3 初始化
  spi1_init();    //SPI1 初始化

//w25q flash 测试

  w25nxx_reset(&g_w25nxx_dev);

  w25nxx_jedec_id_read(&g_w25nxx_dev, &w25n_id);
  trace_debug("W25N ID %#X\r\n", w25n_id);

  w25nxx_reg_write(&g_w25nxx_dev, PROT_REG_SR1, 0);
  w25nxx_reg_write(&g_w25nxx_dev, CFG_REG_SR2, 0x18);

// spi flash 测试
#if 0
  uint8_t crc_data[32] = {0};
  uint8_t p_data[4] = {0x33, 0x44, 0x55, 0x77};

  err_code = w25nxx_fast_read_with_BUF(&g_w25nxx_dev, SECTOR0_COL_ADDRESS, 0, crc_data, 4);
  trace_debug("W25N Read api return %d , crc :\r\n", err_code);
  trace_dump(crc_data, 4);

  if( (crc_data[0] == 0x33) && (crc_data[1] == 0x44) && (crc_data[2] == 0x55) && (crc_data[3] == 0x77) )
  {
      trace_debug("W25N CRC Data OK\r\n");
  }
  else
  {
      trace_debug("W25N CRC Data Error\r\n");
      if( (crc_data[0] != 0xFF) || (crc_data[1] != 0xFF) || (crc_data[2] != 0xFF) || (crc_data[3] != 0xFF) )
      {
          trace_debug("W25N CRC Data no origin\r\n"); //非原始数据
          w25nxx_block_128k_erase(&g_w25nxx_dev, 0);
      }

      err_code = w25nxx_data_write(&g_w25nxx_dev, SECTOR0_COL_ADDRESS, 0, p_data, 4);
      trace_debug("W25N data Write api return %d\r\n", err_code);

      w25nxx_fast_read_with_BUF(&g_w25nxx_dev, SECTOR0_COL_ADDRESS, 0, &crc_data[4], 4);
      trace_debug("W25N Read api return %d , crc :\r\n", err_code);
      trace_dump(&crc_data[4], 4);
  }
  
#endif

//i2c eeprom 测试
#if 0
  uint32_t ee_crc_data = 0;
  uint32_t ee_pData[1] = {0x12345678};

  trace_debug("AT24CXX connect %d\r\n", at24_isConnected());

  at24_read(0x00, (uint8_t *)&ee_crc_data, 4, 100);
  if(ee_crc_data == 0x12345678)
  {
    trace_debug("EE CRC Data OK\r\n");
  }
  else
  {
    trace_debug("EE CRC Data Error %#X\r\n", ee_crc_data);
    err_code = at24_write(0x00, (uint8_t *)ee_pData, 4, 100);
    trace_debug("EE Write api return %d\r\n", err_code);
  }
#endif

  TIMER_INIT();   // 调度定时器初始化，用于简单的ms级定时器调度

  trace_info("\r\n STM32G474 FOC Test Start \r\n\r\n")

  param_init();   //参数初始化

  phase_pwm_start();

//  apt_ekf_init();

  while (1)
  {
      if(sys_time_ms_get() - test_inter_ticks >= 1000)
      {
        test_inter_ticks = sys_time_ms_get();

        if(led_stat)
        {
            led_stat = false;
        }
        else
        {
            led_stat = true;
        }

        gpio_output_set(DSP_LED_GREEN_PORT, DSP_LED_GREEN_PIN, led_stat);

#ifdef DEBUG_SVPWM      // 测试 SVPWM 波形
        gt_vdq.vd = 0;                                         // D轴赋值
        gt_vdq.vq = 4.0f;                                      // Q轴赋值

        gt_theta += PI / 3;

        gt_theta = radian_normalize(gt_theta);

        trace_debug("t_theta %.3f, sector %d, CCR1 %lu, CCR2 %lu, CCR3 %lu\r\n", \
          gt_theta, gt_sector, TIM1->CCR1, TIM1->CCR2, TIM1->CCR3);
#endif

//        sin_cal_speed_compare();
        trace_debug("evt code %#llx, time %ld s\r\n", g_app_param.evt_code, test_inter_ticks/1000);
      }

      sensors_task();         //传感器任务

      motor_ctrl_task_r();      //电机控制任务

      mb_slaver_task();       //modbus 从机任务

      monitor_task();         //监控任务

      mid_timer_loop_task();  //调度定时器的循环执行
  }
}



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
