#include "parameters.h"
#include "math.h"


app_param_t g_app_param = {
    .slave_addr = 1,

    .motor_sta = MOTOR_STA_STOP,
    .pre_motor_sta = MOTOR_STA_STOP,
    .motor_dir = MOTOR_DIR_CCW,

    .target_speed_ring_s = 15.0f,

    .vf_target_uq = 10.0f,
    .vf_target_ud = 0.0f,
    .target_iq = 0.5f,

    .vf_curr_uq = 0.0f,
    .curr_iq = 0.0f,
    .vf_curr_theta = 0.0f,
    .vf_step_rad = 0.001f,
    .step_ring_s = 1.0f,
    .vf_ratio = 1.5f,

// 阈值
    .step_curr_th  = IGBT_STEP_CURR_TH,
    .over_curr_th  = IGBT_OVERCURR_TH,
    .limit_curr_th = IGBT_LIMIT_CURR_TH,

    .pim_igbt_over_t_th = IGBT_TEMP_TH,
    .pim_igbt_limit_t_th = IGBT_TEMP_LIMIT_TH,
    .rad_over_t_th = 80.0f,
    .rad_limit_t_th = 60.0f,
    .ctrl_bsp_warn_t_th = 80.0f,
    .ubus_over_volt_th = MB_VOLT_OVER_TH,
    .ubus_under_volt_th = MB_VOLT_UNDER_TH,

    .evt_code = 0,
};

mb_ctrl_param_t g_mb_ctrl_param;

w25nxx_t g_w25nxx_dev = {
    .select = w25nxx_select,
    .disselect = w25nxx_disselect,
    .delay_ms = delay_ms,
    .transfer = spi1_bytes_wr,
};

lfs_t g_lfs;                        //文件系统
lfs_file_t g_running_log_file;     //运行数据交记录文档
lfs_file_t g_boot_cnt_file;         //开机次数记录文档

struct lfs_config lfs_cfg = {
    // block device operations
    .read  = w25nxx_lfs_read,
    .prog  = w25nxx_lfs_write,
    .erase = w25nxx_lfs_erase,
    .sync  = w25nxx_lfs_sync,

    // block device configuration
    .read_size = W25N_PAGE_SIZE,
    .prog_size = W25N_PAGE_SIZE,
    .block_size = W25N_BLOCK_128K_SIZE,
    .block_count = W25N_NUM_OF_BLOCKS,
    .cache_size = W25N_PAGE_SIZE,
    .lookahead_size = W25N_PAGE_SIZE,
    .block_cycles = 500,
};


volt_dq_t           gt_vdq;
transf_cos_sin_t    gt_cos_sin;
float               gt_theta = PI / 5;
volt_alpha_beta_t   gt_v_alpha_beta;
int gt_sector = 0;

