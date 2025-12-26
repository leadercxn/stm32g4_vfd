#include "parameters.h"
#include "math.h"


app_param_t g_app_param = {
    .slave_addr = 1,

    .motor_sta = MOTOR_STA_STOP,
    .pre_motor_sta = MOTOR_STA_STOP,
    .motor_dir = MOTOR_DIR_CCW,
    .motor_start_acc_sta = MOTOR_START_STA_ACC,

    .motor_speed_set    = 10.0f,

    .target_uq = 0.5f,
    .target_iq = 0.5f,

    .curr_uq = 0.0f,
    .curr_iq = 0.0f,
    .curr_theta = 0.0f,

    .iq_acc_dir = ACC_DONE,
    .is_speed_ring_start = false,

    .evt_code = 0,

    .target_step_angle = 0.001f,
};

mb_ctrl_param_t g_mb_ctrl_param;

w25nxx_t g_w25nxx_dev = {
    .select = w25nxx_select,
    .disselect = w25nxx_disselect,
    .delay_ms = delay_ms,
    .transfer = spi1_bytes_wr,
};

volt_dq_t           gt_vdq;
transf_cos_sin_t    gt_cos_sin;
float               gt_theta = PI / 5;
volt_alpha_beta_t   gt_v_alpha_beta;
int gt_sector = 0;

