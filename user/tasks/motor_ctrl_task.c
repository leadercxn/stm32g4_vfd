#include <math.h>
#include "string.h"

#include "boards.h"
#include "adc.h"
#include "sys.h"
#include "util.h"
#include "gpio.h"
#include "parameters.h"
#include "motor_ctrl_task.h"
#include "uart.h"
#include "timer.h"
#include "foc.h"
#include "vofa.h"
#include "sensors_task.h"
//#include "ekf.h"

#include "app_timer.h"
#include "trace.h"

TIMER_DEF(m_speed_pid_timer);           //速度环定时器

typedef enum
{
    CMD_SW = 1,             //开关
    CMD_TARGET_SPEED,       //目标速度
    CMD_TARGET_IQ,          //目标d轴电流
    CMD_TARGET_UQ,          //目标q轴电压
    CMD_VF_STEP_RAD_S,      //目标步进幅度
    CMD_DIR,                //方向
} uart_cmd_e;

typedef union
{
  	float       fdate;
	uint32_t    udata;
} float_uint32_u;

typedef struct
{
    uart_cmd_e      cmd;
    float_uint32_u  data;
} __attribute__((__packed__ )) uart_cmd_t;

static float m_short_target_speed_ring_s = 0.0f;    //短暂的目标速度，用来做EKF的加减速目标速度

/**
 * timer1 CCH4 中断回调函数 10KHz的执行频率
 */
static void timer1_irq_cb_handler(void)
{
    gpio_output_set(DSP_DRIVE_IGBT_PORT, DSP_DRIVE_IGBT_PIN, 0);

    adc3_inj_start();        //每一次中断触发一次电流采集 10K 的执行频率
}

/**
 * 电机vf运行, 100us 的执行频率
 */
void motor_vf_run(void)
{
    static uint8_t  pwm_start_cnt = 0;
    static uint16_t vf_start_cnt = 0;
    static uint16_t ekf_acc_dec_step_cnt = 0;

#ifdef DEBUG_SVPWM      // 测试 SVPWM
    foc_algorithm_step_r();

    TIM1->CCR1 = (uint16_t)(g_foc_output.tcmp1);     
	TIM1->CCR2 = (uint16_t)(g_foc_output.tcmp2);
	TIM1->CCR3 = (uint16_t)(g_foc_output.tcmp3);
#else
    if(g_app_param.motor_sta == MOTOR_STA_VF_START)             // vf启动
    {
        pwm_start_cnt = 0;      //在每次启动的时候，都清0一次

        if(g_app_param.vf_curr_uq < g_app_param.vf_target_uq)
        {
            g_app_param.motor_sta = MOTOR_STA_VF_ACC;
        }
        else
        {
            g_app_param.motor_sta = MOTOR_STA_VF_DEC;
        }
    }

    if(g_app_param.motor_sta == MOTOR_STA_VF_ACC)               //vf 加速
    {
        g_app_param.vf_curr_uq += 0.001f;  //步进
        if(g_app_param.vf_curr_uq > g_app_param.vf_target_uq)
        {
            g_app_param.vf_curr_uq = g_app_param.vf_target_uq;
            g_app_param.motor_sta = MOTOR_STA_VF_CONST;
        }
    }
    else if(g_app_param.motor_sta == MOTOR_STA_VF_DEC)          //vf 减速
    {
        g_app_param.vf_curr_uq -= 0.001f;  //步
        if(g_app_param.vf_curr_uq < g_app_param.vf_target_uq)
        {
            g_app_param.vf_curr_uq = g_app_param.vf_target_uq;
            g_app_param.motor_sta = MOTOR_STA_VF_CONST;
        }
    }

    // 处于 VF 阶段
    if(g_app_param.motor_sta < MOTOR_STA_EKF_START)
    {
        g_foc_input.theta = g_app_param.vf_curr_theta;
        g_foc_input.iq_ref = g_app_param.vf_curr_uq;
        //检测速度是否达标速度闭环
        if( (g_foc_output.ekf[2] > 40.0f) || (g_foc_output.ekf[2] < -40.0f) )
        {
            vf_start_cnt++;
            if(vf_start_cnt > 100)                              //速度环达标超 0.1 * 100 ms 后，转到速度闭环
            {
                vf_start_cnt = 0;
                TIMER_START(m_speed_pid_timer, 1);              //1K的执行频率, 速度环PID
                g_app_param.motor_sta = MOTOR_STA_EKF_START;
                m_short_target_speed_ring_s = 10.0f;            //给定 ekf 初始的目标速度,避免过大，启动的时候要顺畅
            }
        }
        else
        {
            vf_start_cnt = 0;
        }
    }

    // 处于 EKF 阶段
    if(g_app_param.motor_sta > MOTOR_STA_VF_CONST)
    {
        // 使用卡尔曼
        g_foc_input.theta            = g_foc_output.ekf[3];          //使用卡尔曼估算角度
        g_foc_input.iq_ref           = g_speed_pid_out;              //使用速度环的输出值作为目标Iq

        g_app_param.curr_speed_ring_s = g_foc_output.ekf[2] / DOUBLE_PI;    //使用卡尔曼估算的角速度， 单位：圈/秒

        if(g_app_param.curr_speed_ring_s < (g_app_param.target_speed_ring_s - 1))
        {
            g_app_param.motor_sta =  MOTOR_STA_EKF_ACC;
        }
        else if(g_app_param.curr_speed_ring_s > (g_app_param.target_speed_ring_s + 1))
        {
            g_app_param.motor_sta =  MOTOR_STA_EKF_DEC;
        }
        else
        {
            g_app_param.motor_sta =  MOTOR_STA_EKF_CONST;

            ekf_acc_dec_step_cnt = 0;
        }

        if(g_app_param.motor_sta ==  MOTOR_STA_EKF_ACC)         //ekf 加速
        {
            ekf_acc_dec_step_cnt++;
            if(ekf_acc_dec_step_cnt >= 5000)        //每 500ms 步进一次
            {
                ekf_acc_dec_step_cnt = 0;

                m_short_target_speed_ring_s += g_app_param.ekf_step_ring_s;
                if(m_short_target_speed_ring_s > g_app_param.target_speed_ring_s)
                {
                    m_short_target_speed_ring_s = g_app_param.target_speed_ring_s;
                }
            }
        }
        else if(g_app_param.motor_sta ==  MOTOR_STA_EKF_DEC)    //ekf 减速
        {
            ekf_acc_dec_step_cnt++;
            if(ekf_acc_dec_step_cnt >= 5000)        //每 500ms 步进一次
            {
                ekf_acc_dec_step_cnt = 0;

                m_short_target_speed_ring_s -= g_app_param.ekf_step_ring_s;
                if(m_short_target_speed_ring_s < g_app_param.target_speed_ring_s)
                {
                    m_short_target_speed_ring_s = g_app_param.target_speed_ring_s;
                }
            }
        }
    }

    g_foc_input.udc     = 24.0f;
    g_foc_input.ia      = adc_sample_physical_value_get(ADC_CH_U_I);
    g_foc_input.ib      = adc_sample_physical_value_get(ADC_CH_V_I);
    g_foc_input.ic      = adc_sample_physical_value_get(ADC_CH_W_I);
    g_foc_input.id_ref  = 0.0f;

    //计算好后赋值到PWM_CCRX比较寄存器通道
   	foc_algorithm_step();
    TIM1->CCR1 = (uint16_t)(g_foc_output.tcmp1);     
	TIM1->CCR2 = (uint16_t)(g_foc_output.tcmp2);
	TIM1->CCR3 = (uint16_t)(g_foc_output.tcmp3);

    pwm_start_cnt++;
    if(pwm_start_cnt > 2)     // 等待3个PWM周期后，闭合IGBT，避免反相电路输出上电就是高电平，同时导通上下半桥
    {
        pwm_start_cnt = 0;
        gpio_output_set(DSP_RELAY_IGBT_PORT, DSP_RELAY_IGBT_PIN, 1);
    }
#endif  // DEBUG_SVPWM

}

/**
 *发送串口数据到vofa显示
 */
static void vofa_send(void)
{
#if 0
    justfloat_update(g_foc_output.ekf[3], 0);       //卡尔曼估算角度 -- 0
    justfloat_update(g_foc_output.ekf[2], 0);       //卡尔曼估算速度 -- 1
    justfloat_update(g_pll.theta, 0);               //SMO估算角度   -- 2
    justfloat_update(g_pll.we,    0);               //SMO角速度     -- 3
    justfloat_update(g_current_dq.iq,    0);        //当前Iq        -- 4
    justfloat_update(g_foc_input.iq_ref,    0);     //目标Iq        -- 5
    justfloat_update(g_voltage_dq.vq,    0);        //实际的Vq      -- 6
    justfloat_update(g_app_param.target_speed_ring_s,    0);    //目标speed     -- 7
    justfloat_update(g_app_param.vf_curr_theta,  1);            //强拖的角度     -- 8
#endif

    justfloat_update(g_foc_output.ekf[3], 0);       //卡尔曼估算角度 -- 0
    justfloat_update(g_foc_output.ekf[2], 0);       //卡尔曼估算速度 -- 1
    justfloat_update(g_foc_input.ia,    0);         //U相电流       -- 2
    justfloat_update(g_foc_input.ib,    0);         //V相电流       -- 3
    justfloat_update(g_foc_input.ic,    0);         //W相电流       -- 4
    justfloat_update(g_current_dq.iq,    0);        //当前Iq        -- 5
    justfloat_update(g_foc_input.iq_ref,    0);     //目标Iq        -- 6
    justfloat_update(g_voltage_dq.vq,    0);        //实际的Vq      -- 7
    justfloat_update(g_app_param.vf_curr_theta,  1);   //强拖的角度     -- 9
}

/**
 * 速度环回调函数
 */
static void speed_pid_timer_handler_r(void *p_data)
{
    //速度环执行
    speed_pid_cal(m_short_target_speed_ring_s, g_foc_output.ekf[2], &g_speed_pid_out, &g_speed_pid);
}

static void usart_ctrl_cmd_handler(void)
{
    //串口控制命令处理
    uart_cmd_t  usart1_rx_data;
    uint8_t     usart1_rx_len = 0;

    usart1_rx_len = usart1_rx( (uint8_t *)&usart1_rx_data );
    if(usart1_rx_len > 0)
    {
        trace_debug("u1 rx %d data:\r\n", usart1_rx_len);
        trace_dump((uint8_t *)&usart1_rx_data, usart1_rx_len);

        if(usart1_rx_len == 5)      //目前VOFA个人设置只发送5字节数据
        {
            switch (usart1_rx_data.cmd)
            {
                case CMD_SW:
                    if(usart1_rx_data.data.udata == 0x0)                //关机控件
                    {
                        g_app_param.motor_cmd   = MOTOR_CMD_STOP;
                        trace_debug("motor stop\r\n");
                    }
                    else if(usart1_rx_data.data.udata == 0x3F800000)    //开机控件
                    {
                        g_app_param.motor_cmd   = MOTOR_CMD_STARTUP;
                        trace_debug("motor start\r\n");
                    }
                    break;

                case CMD_TARGET_SPEED:
                        if(g_app_param.motor_dir == MOTOR_DIR_CCW)  //逆
                        {
                            if(usart1_rx_data.data.fdate < 0.0f)
                            {
                                usart1_rx_data.data.fdate = -usart1_rx_data.data.fdate;
                            }
                        }
                        else                                        //顺
                        {
                            if(usart1_rx_data.data.fdate > 0.0f)
                            {
                                usart1_rx_data.data.fdate = -usart1_rx_data.data.fdate;
                            }
                        }

                        g_app_param.target_speed_ring_s = usart1_rx_data.data.fdate;

                        trace_debug("target speed %.4f\r\n", usart1_rx_data.data.fdate);
                    break;

                case CMD_TARGET_IQ:
                        if(g_app_param.motor_dir == MOTOR_DIR_CCW)  //逆
                        {
                            if(usart1_rx_data.data.fdate < 0.0f)
                            {
                                usart1_rx_data.data.fdate = -usart1_rx_data.data.fdate;
                            }
                        }
                        else                                        //顺
                        {
                            if(usart1_rx_data.data.fdate > 0.0f)
                            {
                                usart1_rx_data.data.fdate = -usart1_rx_data.data.fdate;
                            }
                        }

                        g_app_param.target_iq = usart1_rx_data.data.fdate;
                        if((g_app_param.target_iq > 6.0f) || (g_app_param.target_iq < -6.0f))
                        {
                            g_app_param.target_iq = 0.0f;
                        }

                        trace_debug("target Iq %.4f\r\n", usart1_rx_data.data.fdate);
                    break;

                case CMD_TARGET_UQ:
                        g_app_param.vf_target_uq  = usart1_rx_data.data.fdate;

                        trace_debug("target Uq %.4f\r\n", usart1_rx_data.data.fdate);
                    break;

                case CMD_VF_STEP_RAD_S:
                        if(g_app_param.motor_dir == MOTOR_DIR_CCW)  //逆
                        {
                            if(usart1_rx_data.data.fdate < 0.0f)
                            {
                                usart1_rx_data.data.fdate = -usart1_rx_data.data.fdate;
                            }
                        }
                        else                                        //顺
                        {
                            if(usart1_rx_data.data.fdate > 0.0f)
                            {
                                usart1_rx_data.data.fdate = -usart1_rx_data.data.fdate;
                            }
                        }

                        g_app_param.vf_step_rad = usart1_rx_data.data.fdate;

                        trace_debug("target step angle %.4f\r\n", usart1_rx_data.data.fdate);
                    break;

                case CMD_DIR:
                    if(usart1_rx_data.data.udata == 0x0)                //控件数据
                    {
                        g_app_param.motor_dir = MOTOR_DIR_CCW;
                        trace_debug("dir cw\r\n");

                        //改变了方向，数据的正负极性也要修改
                        if(g_app_param.target_speed_ring_s < 0.0f)
                        {
                            g_app_param.target_speed_ring_s = -g_app_param.target_speed_ring_s;
                        }

                        if(g_app_param.target_iq < 0.0f)
                        {
                            g_app_param.target_iq = -g_app_param.target_iq;
                        }

                        if(g_app_param.vf_step_rad < 0.0f)
                        {
                            g_app_param.vf_step_rad = -g_app_param.vf_step_rad;
                        }
                    }
                    else if(usart1_rx_data.data.udata == 0x3F800000)    //控件数据
                    {
                        g_app_param.motor_dir = MOTOR_DIR_CW;
                        trace_debug("dir ccw\r\n");

                        //改变了方向，数据的正负极性也要修改
                        if(g_app_param.target_speed_ring_s > 0.0f)
                        {
                            g_app_param.target_speed_ring_s = -g_app_param.target_speed_ring_s;
                        }

                        if(g_app_param.target_iq > 0.0f)
                        {
                            g_app_param.target_iq = -g_app_param.target_iq;
                        }

                        if(g_app_param.vf_step_rad > 0.0f)
                        {
                            g_app_param.vf_step_rad = -g_app_param.vf_step_rad;
                        }
                    }
                    break;
                
                default:
                    break;
            }
        }
    }
}

/**
 * 电机控制逻辑任务
 */
int motor_ctrl_task(void)
{
    static bool init_done = false;

    if(!init_done)
    {
        init_done = true;
        timer1_irq_cb_register(timer1_irq_cb_handler);      //回调函数注册到 timer8 的中断函数里面

        TIMER_CREATE(&m_speed_pid_timer, false, true, speed_pid_timer_handler_r);     //循环定时器，立马执行
    }

    usart_ctrl_cmd_handler();    //串口控制命令处理

    if(g_app_param.motor_cmd != g_app_param.old_motor_cmd)
    {
        if(g_app_param.motor_cmd == MOTOR_CMD_STARTUP)
        {
            if(g_app_param.motor_sta == MOTOR_STA_STOP)         // 在关机状态下才能开机
            {
                g_app_param.motor_sta   = MOTOR_STA_VF_START;   // 进入VF启动状态
            }
        }
        else if(g_app_param.motor_cmd == MOTOR_CMD_STOP)
        {
            /**
             * 收到关机命令，就关机，不做递减到最小速度再来关机的逻辑
             * 漫长的关机过程，在某些新能源的应用场景不适用
             */
            g_app_param.motor_sta = MOTOR_STA_STOP;
        }

        g_app_param.old_motor_cmd = g_app_param.motor_cmd;
    }

    // 电机状态机
    switch(g_app_param.motor_sta)
    {
        case MOTOR_STA_STOP:
            gpio_output_set(DSP_RELAY_IGBT_PORT, DSP_RELAY_IGBT_PIN, 0);

            if(g_app_param.motor_sta != g_app_param.pre_motor_sta)  //开始停机
            {
                phase_pwm_stop();

                g_app_param.curr_iq = 0.0f;
                g_app_param.vf_curr_uq = 0.0f;
                g_app_param.vf_curr_theta = 0.0f;

                foc_algorithm_init();                               //FOC 算法参数初始化

                TIMER_STOP(m_speed_pid_timer);
            }
            break;

        case MOTOR_STA_VF_START:
            if(g_app_param.motor_sta != g_app_param.pre_motor_sta)  //每一次启动都要foc参数初始化
            {
                if_start_param_init();                              //IF启动参数初始化

                foc_algorithm_init();                               //FOC 算法参数初始化

                phase_pwm_start();
            }
            break;

        case MOTOR_STA_ERROR:
            gpio_output_set(DSP_RELAY_IGBT_PORT, DSP_RELAY_IGBT_PIN, 0);
            break;
    }

#ifndef TRACE_ENABLE
    vofa_send();    //vofa 显示
#endif

    if(g_app_param.motor_sta != g_app_param.pre_motor_sta)
    {
        g_app_param.pre_motor_sta = g_app_param.motor_sta;
    }

    return 0;
}

