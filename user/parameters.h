#ifndef PARAMETERS_H__
#define PARAMETERS_H__

#include <stdint.h>

#include "sys.h"
#include "boards.h"
#include "foc.h"
#include "pid.h"
#include "lfs.h"
#include "lfs_util.h"

#include "speed_pid.h"
#include "foc_algorithm.h"
#include "smo_pll.h"
#include "iir_lpf.h"
#include "arm_math.h"
#include "if_start.h"
#include "delay.h"
#include "spi.h"
#include "gpio.h"
#include "timer.h"
#include "lfs_api.h"


#include "w25n01gvxxig.h"

#define SYS_CLK_FREQ    170000000
#define PWM_FREQ        20000       //20K
#define PWM_PERIOD      8500        //(SYS_CLK_FREQ / PWM_FREQ)
#define MAX_PWM_DUTY    ((PWM_PERIOD - 1) * 0.96)  //最大占空比

#define PWM_TIM_PULSE_TPWM  (SYS_CLK_FREQ / (PWM_FREQ  / 2) )   //因为中心对齐

//电机参数
#define MOTOR_POLE_PAIRS    2           //电机极对数
#define MOTOR_PHASE_RES     0.2f        //电机相电阻，单位欧姆
#define MOTOR_PHASE_LS      0.0004f     //电机相电感，单位亨利
#define MOTOR_FLUXLINK      0.0090969f  //电机磁链常数

//程序设定参数
#define VBUS_VLOT               560.0f  //母线电压，单位V
#define MOTOR_SPEED_RING_S_MAX  100.0f  //电机最高转速, ring/s
#define MOTOR_SPEED_RING_S_MIN  2.0f    //电机最小速度

//FOC参数
#define FOC_PERIOD                  0.0001f     //FOC运行的时间间隔 s
#define FOC_FREQ                    10000       // (1.0f / FOC_PERIOD) FOC运行的频率 Hz
#define SPEED_LOOP_CLOSE_RAD_S      20.0f       //速度环切入闭环的速度  单位: rad/s
#define ONE_DIV_TWO_PI_FOC_PERIOD   1591.5494f  // 1 / (TWO_PI * FOC_PERIOD),设置为常数，计算更快
#define VF_RATIO_MIN                1.20f      //V/F 最小 VF 比率
#define VF_RATIO_MAX                3.50f      //V/F 最大 VF 比率

#define RING_PER_S_2_RAD(rings)      (float)((rings) * TWO_PI * FOC_PERIOD)         //圈/秒 -> 弧度/周期
#define RAD_2_RING_PER_S(rads)       (float)((rads) * ONE_DIV_TWO_PI_FOC_PERIOD)    //弧度/周期 -> 圈/秒

/**
 * 算法参数
 */
// 速度环默认参数
#define SPEED_PI_P          0.003f
#define SPEED_PI_I          5.0f
#define SPEED_PI_KB         0.015f
#define SPEED_PI_LOW_LIMIT  -6.0f
#define SPEED_PI_UP_LIMIT   6.0f

// Q轴电流环默认参数
#if 0
#define Q_PI_P              3.199f
#define Q_PI_I              2282.8f
#define Q_PI_KB             15.0f
#define Q_PI_LOW_LIMIT      -10.0f
#define Q_PI_UP_LIMIT       10.0f
#endif
#define Q_PI_P              1.199f
#define Q_PI_I              1000.0f
#define Q_PI_KB             5.0f
#define Q_PI_LOW_LIMIT      -10.0f
#define Q_PI_UP_LIMIT       10.0f

/**
 * 板载硬件配置
 */
#define AT24CXX_DEV_ADDR        0xA0        //AT24CXX 器件地址

#define IGBT_STEP_CURR_TH       30.0f       //IGBT 阶跃电流阈值, 单位A      -- 瞬间电流
#define IGBT_OVERCURR_TH        17.0f       //IGBT 过流保护电流阈值, 单位A  -- 均方根电流
#define IGBT_LIMIT_CURR_TH      10.0f       //IGBT 限流保护电流阈值, 单位A  -- 均方根电流

#define IGBT_TEMP_TH            90.0f      //IGBT 过温保护阈值, 单位摄氏度  -- igbt 停机
#define IGBT_TEMP_LIMIT_TH      75.0f       //IGBT 限温保护阈值, 单位摄氏度  -- igbt 降额运行

#define MB_VOLT_OVER_TH         700         // 母线电压过压保护阈值
#define MB_VOLT_UNDER_TH        320         // 母线电压欠压保护阈值

// 电机状态
typedef enum
{
    MOTOR_STA_STOP,         //停止

    MOTOR_STA_VF_START,     //vf启动中
    MOTOR_STA_VF_ACC,       //vf加速中
    MOTOR_STA_VF_DEC,       //vf减速中
    MOTOR_STA_VF_CONST,     //vf恒速中

    MOTOR_STA_EKF_START,    //ekf启动中
    MOTOR_STA_EKF_ACC,      //ekf加速中
    MOTOR_STA_EKF_DEC,      //ekf减速中
    MOTOR_STA_EKF_CONST,    //ekf恒速中

    MOTOR_STA_ERROR,        //故障状态,必停机
} motor_sta_e;

typedef enum
{
    MOTOR_DIR_CW,      //顺时针
    MOTOR_DIR_CCW,     //逆时针
} motor_dir_e;

typedef enum
{
    MOTOR_CMD_NONE,      //无命令
    MOTOR_CMD_STARTUP,   //启动命令
    MOTOR_CMD_STOP,      //停止命令
    MOTOR_CMD_RESET,     //复位命令
    MOTOR_CMD_STOP_NIW,  //紧急停止命令
} motor_cmd_e;


/*********** MODBUS **************/
typedef enum
{
    REG_SW = 0x01,          //开关机
    REG_DIR,                //电机方向

    REG_TARGET_SPEED_L16,   //目标速度 低16位
    REG_TARGET_SPEED_H16,   //目标速度 高16位

//电机调试参数
    REG_SPEED_PID_P_L16,    //速度环P参数 低16位
    REG_SPEED_PID_P_H16,    //速度环P参数 高16位
    REG_SPEED_PID_I_L16,    //速度环I参数 低16位
    REG_SPEED_PID_I_H16,    //速度环I参数 高16位
    REG_SPEED_PID_KB_L16,   //速度环Kb参数 低16位
    REG_SPEED_PID_KB_H16,   //速度环Kb参数 高16位
    REG_SPEED_PID_LIMIT_L16,   //速度环幅值 低16位
    REG_SPEED_PID_LIMIT_H16,   //速度环幅值 高16位

    REG_I_PID_P_L16,        //电流环P参数 低16位
    REG_I_PID_P_H16,        //电流环P参数 高16位
    REG_I_PID_I_L16,        //电流环I参数 低16位
    REG_I_PID_I_H16,        //电流环I参数 高16位
    REG_I_PID_KB_L16,       //电流环Kb参数 低16位
    REG_I_PID_KB_H16,       //电流环Kb参数 高16位
    REG_I_PID_LIMIT_L16,    //电流环幅值 低16位
    REG_I_PID_LIMIT_H16,    //电流环幅值 高16位

    REG_PHASE_RS_L16,       //相电阻 低16位
    REG_PHASE_RS_H16,       //相电阻 高16位
    REG_PHASE_LS_L16,       //相电感 低16位
    REG_PHASE_LS_H16,       //相电感 高16位
    REG_FLUX_LINK_L16,      //磁链 低16位
    REG_FLUX_LINK_H16,      //磁链 高16位

    REG_SPEED_MAX_L16,      //最大速度 低16位
    REG_SPEED_MAX_H16,      //最大速度 高16位
    REG_SPEED_MIN_L16,      //最小速度 低16位
    REG_SPEED_MIN_H16,      //最小速度 高16位

    REG_I_ERR_TH_L16,       //过流阈值 低16位
    REG_I_ERR_TH_H16,       //过流阈值 高16位
    REG_V_ERR_TH_L16,       //过压阈值 低16位
    REG_V_ERR_TH_H16,       //过压阈值 高16位

    REG_PLL_P_L16,          //PLL p参数 低16位
    REG_PLL_P_H16,          //PLL p参数 高16位
    REG_PLL_I_L16,          //PLL i参数 低16位
    REG_PLL_I_H16,          //PLL i参数 高16位

    REG_POLE_PAIRS,         //电机极对数
    REG_MB_ADDR,            //modbus地址

// 运行状态参数
    REG_VBUS_VOLT,     //母线电压
    REG_BSP_TEMP,      //板载温度
    REG_U_VOLT,        //U相电压
    REG_V_VOLT,        //V相电压
    REG_W_VOLT,        //W相电压
    REG_U_CURR,        //U相电流
    REG_V_CURR,        //V相电流
    REG_W_CURR,        //W相电流

    REG_CURR_SPEED,    //当前速度
    REG_CURR_THETA,    //当前角度
    
    REG_FLASH_W25N_CLR,     //清除 w25n 芯片的数据

    REG_EVT_CODE0 = 124,    //事件码 Bit0 ~ bit15
    REG_EVT_CODE1,          //事件码 Bit16 ~ bit31
    REG_EVT_CODE2,          //事件码 Bit32 ~ bit47
    REG_EVT_CODE3,          //事件码 Bit48 ~ bit63

    REG_MAX = 128,
} mb_reg_e;

/**
 * 显示故障bit排位 L -> H
 * 
 * ERR  代表故障类事件，需停机处理
 * WARN 代表警告类事件，可记录但不影响运行
 * EVT  代表一般类事件，可记录但不影响运行
 */ 
typedef enum {
//ERR 类事件
    ERR_UBUS_OVER_VOLT,         //直流母线过压
    ERR_UBUS_UNDER_VOLT,        //直流母线欠压
    ERR_U_CURR_SENS,            //U相电流传感器故障
    ERR_V_CURR_SENS,            //V相电流传感器故障

    ERR_W_CURR_SENS,            //W相电流传感器故障
    ERR_ROTOR_ABNORMAL,         //转子异常(堵转)
    ERR_STARTUP_FAIL,           //启动失败
    ERR_PIM_IGBT_T_OVER_TH,     //PIM   过温故障

    ERR_RAD_T_OVER_TH,          //散热片 过温故障
    ERR_IGBT_FLT_HW,            //IGBT故障硬件反馈
    ERR_UVW_IN_PHASE_LOSS_HW,   //UVW 输入缺相硬件反馈
    ERR_U_OVER_CURR,            //U相过流          -- 均值电流过大

    ERR_V_OVER_CURR,            //V相过流
    ERR_W_OVER_CURR,            //W相过流
    ERR_U_STEP_CURR,            //U相阶跃电流超限  -- 阶跃瞬间电流过大
    ERR_V_STEP_CURR,            //V相阶跃电流超限

    ERR_W_STEP_CURR,            //W相阶跃电流超限
    ERR_U_OUT_PHASE_LOSS,       //U相输出缺相
    ERR_V_OUT_PHASE_LOSS,       //V相输出缺相
    ERR_W_OUT_PHASE_LOSS,       //W相输出缺相

    ERR_TRAN_OUT_ABNORMAL,      //变压器输出异常
//WARN 类事件
    WARN_PIM_IGBT_T_LIMIT,      //PIM 高温警告   -- 限频处理
    WARN_RAD_T_LIMIT,           //散热片 高温警告
    WARN_CTRL_BSP_T,            //控制板载 高温警告

    WARN_U_CURR_LIMIT,          //U相限流警告     -- 超过限频电流值，降频处理
    WARN_V_CURR_LIMIT,          //V相限流警告
    WARN_W_CURR_LIMIT,          //W相限流警告
    WARN_EB_WU_HW,              //EB WU硬件反馈
    WARN_EA_VU_HW,              //EA VU硬件反馈
    WARN_BOX_TSENS_ERR,         //控制板载温度传感器故障
    WARN_PIM_TSENS_ERR,         //PIM 温度传感器故障
    WARN_RAD_TSENS_ERR,         //IGBT散热片 温度传感器故障
    WARN_BASE_VOLT,             //基准电压异常 -- 只提示
    WARN_SPIFLASH_ABNOR,        //SPI flash 异常
//EVT 类事件
    EVT_STARTUP_HW,             //启动硬件反馈
    EVT_RESET_HW,               //复位硬件反馈
    EVT_DBG,                    //正在调试
} sys_evtcode_mask_e;



/*********** 电机状态结构体 **************/

// 全局应用参数
typedef struct
{
    uint8_t         slave_addr;         // modbus 从机地址

    motor_sta_e     motor_sta;          // 电机状态
    motor_sta_e     pre_motor_sta;      // 电机前一状态
    motor_dir_e     motor_dir;          // 电机方向
    motor_cmd_e     motor_cmd;          // 电机命令
    motor_cmd_e     old_motor_cmd;      // 上一次电机命令

    float           target_speed_ring_s;   // 电机设定速度，单位： ring/s 圈/秒
//运行参数
    float           curr_speed_ring_s;  // 电机当前速度，单位   ring/s 圈/秒
    float           vf_target_uq;       // vf阶段目标Uq, q轴电压 单位V
    float           vf_target_ud;       // vf阶段目标Ud, d轴电压 单位V
    float           target_iq;          // q轴电流 单位A
    float           vf_curr_uq;         // vf阶段当前Uq
    float           curr_iq;            // 当前Iq
    float           vf_curr_theta;      // vf阶段当前角度值
    float           vf_step_rad;        // vf阶段， 步进角度，单位：弧度
    float           step_ring_s;        // 步进加速度，单位：圈/秒
    float           vf_ratio;           // vf比例系数

// 阈值
    float           step_curr_th;       // 阶跃电流阈值
    float           over_curr_th;       // 过流保护阈值
    float           limit_curr_th;      // 限流保护阈值

    float           pim_igbt_over_t_th;        // pim-igbt 过温保护阈值
    float           pim_igbt_limit_t_th;       // pim-igbt 限温保护阈值
    float           rad_over_t_th;             // 散热片过温保护阈值
    float           rad_limit_t_th;            // 散热片限温保护阈值
    float           ctrl_bsp_warn_t_th;        // 控制板温度预警值
    uint16_t        ubus_over_volt_th;         // 母线过压阈值
    uint16_t        ubus_under_volt_th;        // 母线欠压阈值

    uint64_t        evt_code;           // 事件代码
} app_param_t;

extern app_param_t g_app_param;

//调试参数
typedef struct
{
    //速度环参数
    float    speed_pid_p;        //
    float    speed_pid_i;        //
    float    speed_pid_kb;       //
    float    speed_pid_limit;    //正负对称

    //电流环参数
    float    i_pid_p;            //
    float    i_pid_i;            //
    float    i_pid_kb;           //
    float    i_pid_limit;        //正负对称

    //电机参数
    float    phase_rs;           // 相电阻
    float    phase_ls;           // 相电感
    float    flux_link;          // 磁链

    float    i_err_th;           // 母线过流阈值
    float    v_err_th;           // 母线过压阈值

    float    pll_p;              // pll p参数
    float    pll_i;              // pll i参数

    uint16_t motor_pole_pairs;   // 电机极对数
} mb_ctrl_param_t;


extern mb_ctrl_param_t g_mb_ctrl_param;
extern w25nxx_t g_w25nxx_dev;

extern lfs_t g_lfs;
extern lfs_file_t g_running_data_file;
extern lfs_file_t g_boot_cnt_file;
extern struct lfs_config lfs_cfg;

extern volt_dq_t           gt_vdq;
extern transf_cos_sin_t    gt_cos_sin;
extern float               gt_theta;
extern volt_alpha_beta_t   gt_v_alpha_beta;
extern int gt_sector;

#endif
