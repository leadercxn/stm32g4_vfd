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
#include "record_task.h"

#include "at24cxx.h"
#include "lfs_api.h"

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

static void test_task(void)
{
    bool run_once = true;
    uint32_t boot_count = 0;
    int32_t  offset = 0;
    int32_t  file_size = 0;

    int err_code = 0;

    if(run_once)
    {
      run_once = false;

		  lfs_file_open(&g_lfs, &g_boot_cnt_file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
		  lfs_file_read(&g_lfs, &g_boot_cnt_file, &boot_count, sizeof(boot_count));

      // update boot count
      boot_count += 1;
      lfs_file_rewind(&g_lfs, &g_boot_cnt_file);  // seek the file to begin
      lfs_file_write(&g_lfs, &g_boot_cnt_file, &boot_count, sizeof(boot_count));

      offset = lfs_file_seek(&g_lfs, &g_boot_cnt_file, 0, LFS_SEEK_CUR);

      file_size = lfs_file_size(&g_lfs, &g_boot_cnt_file);

		  lfs_file_close(&g_lfs, &g_boot_cnt_file);

//		lfs_unmount(&lfs);

		  // print the boot count
		  trace_debug("boot_count: %d, offset = %d, file size %d\n", boot_count, offset, file_size);

      err_code = lfs_remove(&g_lfs, "boot_count");
      trace_debug("file remove err_code %d\r\n", err_code);

//验证 littlefs 储存数据是否有问题使用
#if 1
      err_code = lfs_remove(&g_lfs, RUNNING_LOG_FILE);
      trace_debug("%s file remove err_code %d\r\n", RUNNING_LOG_FILE, err_code);
#endif
    }


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

  trace_info("\r\n STM32G474 FOC Test Start \r\n\r\n");

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

  // 挂载 lfs
  if(w25n_id == W25N01GV)
  {
    w25nxx_reg_write(&g_w25nxx_dev, PROT_REG_SR1, 0);
    w25nxx_reg_write(&g_w25nxx_dev, CFG_REG_SR2, 0x18);

    trace_debug("try to mount lfs\r\n");
    err_code = lfs_mount(&g_lfs, &lfs_cfg);

    if(err_code)
    {
      trace_debug("lfs mount err_code %d, try to erase w25n, wait ...\r\n", err_code);
      w25nxx_chip_erase(&g_w25nxx_dev);   // 全片擦除
      trace_debug("erase w25n done, retry to mount lfs again.. \r\n");

      lfs_format(&g_lfs, &lfs_cfg);
      err_code = lfs_mount(&g_lfs, &lfs_cfg);
      trace_debug("lfs again mount err_code %d\r\n", err_code);

      if(err_code)
      {
        trace_debug("lfs mount fail.\r\n");

        hmi_event_set(WARN_SPIFLASH_ABNOR);   //设置SPI flash异常事件
      }
      else
      {
        trace_debug("lfs mount success.\r\n");
      }
    }
    else
    {
      trace_debug("lfs mount success.\r\n");
    }
  }
  else
  {
    hmi_event_set(WARN_SPIFLASH_ABNOR);   //设置SPI flash异常事件
  }

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

// float 类型绝对值测试
#if 0
  float test_iu = 2.3456f;
  float test_iv = -1.2345f;

  float fabs_value = 0.0f;
  fabs_value = fabsf(test_iu);
  trace_debug("test_iu %.4f, fabs %.4f\r\n", test_iu, fabs_value);
  fabs_value = fabsf(test_iv);
  trace_debug("test_iv %.4f, fabs %.4f\r\n", test_iv, fabs_value);
#endif

  param_init();   //参数初始化

  phase_pwm_start();

  gpio_output_set(DSP_LED_ERR_PORT, DSP_LED_ERR_PIN, 1);
  gpio_output_set(DSP_RELAY_IGBT_PORT, DSP_RELAY_IGBT_PIN, 0);  // 先断开 主回路继电器

  test_task();   //测试任务

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

        gpio_output_set(DSP_RELAY_IGBT_PORT, DSP_RELAY_IGBT_PIN, 1);  // 闭合主回路继电器

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

#if 0
        trace_debug("IN: startup-%d, rst-%d, igbt-flt-%d, eb-wu-%d, ea-vu-%d, uvw-%d\r\n", 
            gpio_input_get(DSP_X1_STARTUP_PORT, DSP_X1_STARTUP_PIN),
            gpio_input_get(DSP_X2_RST_PORT, DSP_X2_RST_PIN),
            gpio_input_get(DSP_IGBT_FLT_PORT, DSP_IGBT_FLT_PIN),
            gpio_input_get(DSP_EB_WU_ERR_PORT, DSP_EB_WU_ERR_PIN),
            gpio_input_get(DSP_EA_VU_ERR_PORT, DSP_EA_VU_ERR_PIN),
            gpio_input_get(DSP_UVW_PHASE_LOSS_PORT, DSP_UVW_PHASE_LOSS_PIN) );
#endif
      }

      sensors_task();         //传感器任务

      motor_ctrl_task();      //电机控制任务

      mb_slaver_task();       //modbus 从机任务

      monitor_task();         //监控任务

      record_task();          //记录任务

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
