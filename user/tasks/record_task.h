#ifndef RECORD_TASK_H__
#define RECORD_TASK_H__

#include "parameters.h"

typedef struct
{
    uint32_t    record_frame_id;        //记录帧ID, 每次记录加1

    uint16_t    power_on_cnt;           //上电次数
    motor_sta_e motor_sta;              //系统模式
    uint8_t     crc;                    //数据校验

    uint32_t    record_cnt;             //记录次数 -- 累计，掉电也累计
    uint64_t    sys_tick;               //系统时间
    uint64_t    evt_code;               //系统事件码

    float       target_speed_ring_s;    //系统目标速度 ring/s
    float       curr_speed_ring_s;      //系统当前速度 ring/s
    float       vf_ratio;               //vf比例系数
    float       pim_igbt_t;             //当前pim_igbt_t模块温度值
    float       rad_t;                  //当前rad_t散热器温度值
    float       tran_out_volt;          //变压器输出电压值
    float       ctrl_bsp_t;             //控制板温度
    float       base_volt;              //基准电压 -- 只采样
    float       u_i_rms_aver;           //U相均方根电流值
    float       v_i_rms_aver;           //V相均方根电流值
    float       w_i_rms_aver;           //W相均方根电流值
    float       ubus_volt;              //母线电压值
} __attribute__((__packed__ )) running_record_data_t;


int record_task(void);

#endif
