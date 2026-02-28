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


#define LOG_FRAME_DATA_LEN              128      //每一帧记录的数据长度, 定长
#define RECORD_FRAME_TOTAL_MAX          1048576  // W25N_DEVICE_SIZE / LOG_FRAME_DATA_LEN      最大总共可以记录到多少条数据
#define NUM_OF_FRAME_IN_BLOCK           1024     // W25N_BLOCK_128K_SIZE / LOG_FRAME_DATA_LEN  一块数据区可以储存多少条数据


static uint16_t m_record_block_idx = 0;          //当前记录所在的块索引
static uint16_t m_record_off_in_block = 0;       //当前记录在所在块的偏置

static uint32_t m_record_frame_id = 0;           //当前记录所在的记录帧ID，每次记录加1
static uint16_t m_boot_cnt;                      //当前开技数

/**
 * SPI_FLASH 运行日志记录处理函数
 */
static void running_log_record_handle(void)
{
    running_log_t running_log_data;
    uint8_t       temp[LOG_FRAME_DATA_LEN];
    

    memset((uint8_t *)&running_log_data, 0, sizeof(running_log_t));
    memset(temp, 0, LOG_FRAME_DATA_LEN);

    m_record_frame_id++;

    if(m_record_frame_id < 5)       // ---- 有限次数的测试
    {
        // 数据填充
        running_log_data.app_ver         = APP_VERSION;
        running_log_data.boot_cnt        = m_boot_cnt;
        running_log_data.record_frame_id = m_record_frame_id;
        running_log_data.motor_sta       = g_app_param.motor_sta;
        running_log_data.sys_tick        = sys_time_ms_get();
        running_log_data.evt_code        = g_app_param.evt_code;
        running_log_data.target_speed_ring_s = g_app_param.curr_speed_ring_s;
        running_log_data.curr_speed_ring_s   = g_app_param.curr_speed_ring_s;
        running_log_data.vf_ratio            = g_app_param.vf_ratio;

        trace_debug("app ver %d, boot %d, frame_id %ld, motor_sta %d, tick %llu, evt_code %#llx, tar_ring %0.2f, vf_ratio %0.2f\r\n",  \
                        running_log_data.app_ver, running_log_data.boot_cnt, running_log_data.record_frame_id, running_log_data.motor_sta,\
                        running_log_data.sys_tick, running_log_data.evt_code, running_log_data.target_speed_ring_s, running_log_data.vf_ratio);

        memcpy(temp, (uint8_t *) &running_log_data, sizeof(running_log_t));

        lfs_file_open(&g_lfs, &g_running_log_file, RUNNING_LOG_FILE, LFS_O_RDWR | LFS_O_CREAT);     //打开文件
        lfs_file_seek(&g_lfs, &g_running_log_file, 0, LFS_SEEK_END);                                //光标以为到文件末端
        lfs_file_write(&g_lfs, &g_running_log_file, temp, sizeof(temp));                            //写入数据
        lfs_file_close(&g_lfs, &g_running_log_file);                                                //关闭文件
    }
    
}

/**
 * EEPROM 控制参数记录处理函数
 */
static void ctrl_data_record_handle(void)
{

}

/**
 * @brief 打印储存的数据
 */
static void running_log_printf(void)
{
    static uint32_t ticks = 0;
//    static uint8_t  print_idx = 0;

    int32_t  file_size = 0;
    uint16_t frame_cnt;

    uint8_t temp[LOG_FRAME_DATA_LEN] = {0};
    running_log_t running_log_data;

    if(IS_PRE_MINUS_MID_OVER_POST(sys_time_ms_get(), ticks, 60000))   //1分钟间隔
    {
        ticks = sys_time_ms_get();

        lfs_file_open(&g_lfs, &g_running_log_file, RUNNING_LOG_FILE, LFS_O_RDWR | LFS_O_CREAT);
        file_size = lfs_file_size(&g_lfs, &g_running_log_file);
        trace_debug("%s file size %d \r\n", RUNNING_LOG_FILE, file_size);

        if(file_size > 0)
        {
            frame_cnt = file_size / LOG_FRAME_DATA_LEN;
            for(uint16_t i = 0; i < frame_cnt; i++)
            {
                lfs_file_seek(&g_lfs, &g_running_log_file, 0, i * LOG_FRAME_DATA_LEN);      //光标倒回到倒数第一帧数据头
                lfs_file_read(&g_lfs, &g_running_log_file, temp, LOG_FRAME_DATA_LEN);

                memcpy((uint8_t *)&running_log_data, temp, sizeof(running_log_t));      //读出上一帧数据内容

                trace_debug("app ver %d, boot %d, frame_id %ld, motor_sta %d, tick %llu, evt_code %#llx, tar_ring %0.2f, vf_ratio %0.2f\r\n",  \
                        running_log_data.app_ver, running_log_data.boot_cnt, running_log_data.record_frame_id, running_log_data.motor_sta,\
                        running_log_data.sys_tick, running_log_data.evt_code, running_log_data.target_speed_ring_s, running_log_data.vf_ratio);
            }
        }
        

        lfs_file_close(&g_lfs, &g_running_log_file);
    }
}

/**
 * @brief 记录任务，负责记录电机运行数据到外部存储器中
 */
int record_task(void)
{
    static uint32_t record_ticks = 0;
    static uint64_t old_evt_code = 0;

    static bool run_once = true;

    int32_t       file_size = 0;
    uint32_t      offset = 0;

    // 创建文件，并读出上一次数据帧的 id 和 开机次数
    if(run_once)
    {
        uint8_t temp[LOG_FRAME_DATA_LEN] = {0};
        running_log_t running_log_data;

        run_once = false;

        lfs_file_open(&g_lfs, &g_running_log_file, RUNNING_LOG_FILE, LFS_O_RDWR | LFS_O_CREAT);     //打开文件
        file_size = lfs_file_size(&g_lfs, &g_running_log_file);                                     //算出文件大小
        if(file_size == 0)                                              // 文件从来未写过的
        {
            m_record_frame_id = 0;
            m_boot_cnt++;
        }
        else
        {
            offset = file_size - LOG_FRAME_DATA_LEN;                    //计算倒数第一帧数据头
            lfs_file_seek(&g_lfs, &g_running_log_file, 0, offset);      //光标倒回到倒数第一帧数据头
            lfs_file_read(&g_lfs, &g_running_log_file, temp, LOG_FRAME_DATA_LEN);   //读出最后一帧数据

            memcpy((uint8_t *)&running_log_data, temp, sizeof(running_log_t));      //读出上一帧数据内容

            m_boot_cnt        = running_log_data.boot_cnt;
            m_record_frame_id = running_log_data.record_frame_id;

            trace_debug("histroy boot cnt %d, frame id %d\r\n", m_boot_cnt, m_record_frame_id);

            m_boot_cnt++;
        }

        lfs_file_close(&g_lfs, &g_running_log_file);
    }

    if(IS_PRE_MINUS_MID_OVER_POST(sys_time_ms_get(), record_ticks, 30000))   //间隔
    {
        record_ticks = sys_time_ms_get();

        trace_debug("START RECORD DATA len %d\r\n", sizeof(running_log_t));

        running_log_record_handle();            //记录一次运行数据
    }

    if(old_evt_code != g_app_param.evt_code)    //事件码发生变化，记录一次
    {
        record_ticks = sys_time_ms_get();       //重置记录时间
        old_evt_code = g_app_param.evt_code;

        trace_debug("START RECORD DATA %d\r\n", sizeof(running_log_t));
        running_log_record_handle();        //记录一次运行数据
    }

//    running_log_printf();

    return 0;
}

