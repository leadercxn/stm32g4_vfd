#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"

#include "boards.h"
#include "util.h"

#include "w25n01gvxxig.h"
#include "at24cxx.h"
#include "parameters.h"
#include "record_task.h"
#include "trace.h"


#define RECORD_FRAME_DATA_LEN           128      //每一帧记录的数据长度, 定长
#define RECORD_FRAME_TOTAL_MAX          1048576  // W25N_DEVICE_SIZE / RECORD_FRAME_DATA_LEN      最大总共可以记录到多少条数据
#define NUM_OF_FRAME_IN_BLOCK           1024     // W25N_BLOCK_128K_SIZE / RECORD_FRAME_DATA_LEN  一块数据区可以储存多少条数据


static uint16_t m_record_block_idx = 0;          //当前记录所在的块索引
static uint16_t m_record_off_in_block = 0;       //当前记录在所在块的偏置
static uint32_t m_record_frame_id = 0;           //当前记录所在的记录帧ID，每次记录加1
static uint16_t m_power_on_cnt = 0;

/**
 * 运行日志记录处理函数
 */
static void running_datalog_record_handle(void)
{
    running_record_data_t running_record_data;

    memset((uint8_t *)&running_record_data, 0, sizeof(running_record_data_t));

    if(m_record_frame_id != 0)
    {
        m_record_frame_id++;
    }

    // 数据填充
    running_record_data.record_frame_id = m_record_frame_id;
    running_record_data.power_on_cnt    = m_power_on_cnt;

}

/**
 * 控制参数记录处理函数
 */
static void ctrl_data_record_handle(void)
{

}


/**
 * @brief 记录任务，负责记录电机运行数据到外部存储器中
 */
int record_task(void)
{
    static uint32_t record_ticks = 0;
    static uint32_t old_evt_code = 0;

    if(IS_PRE_MINUS_MID_OVER_POST(sys_time_ms_get(), record_ticks, 30000))   //间隔
    {
        record_ticks = sys_time_ms_get();

        trace_debug("running_record_data_t len %d\r\n", sizeof(running_record_data_t));

        running_datalog_record_handle();        //记录一次运行数据
    }

    if(old_evt_code != g_app_param.evt_code)    //事件码发生变化，记录一次
    {
        record_ticks = sys_time_ms_get();       //重置记录时间
        old_evt_code = g_app_param.evt_code;

        running_datalog_record_handle();        //记录一次运行数据
    }


    return 0;
}

