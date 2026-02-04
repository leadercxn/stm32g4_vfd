#include "parameters.h"
#include "math.h"


app_param_t g_app_param = {
    .slave_addr = 1,

    .motor_sta = MOTOR_STA_STOP,
    .pre_motor_sta = MOTOR_STA_STOP,
    .motor_dir = MOTOR_DIR_CCW,

    .target_speed_ring_s = 2.0f,

    .vf_target_uq = 10.0f,
    .vf_target_ud = 0.0f,
    .target_iq = 0.5f,

    .vf_curr_uq = 0.0f,
    .curr_iq = 0.0f,
    .vf_curr_theta = 0.0f,
    .vf_step_rad = 0.001f,
    .step_ring_s = 1.0f,
    .vf_ratio = 1.5f,

    .u_rms_curr = 0.0f,
    .v_rms_curr = 0.0f,
    .w_rms_curr = 0.0f,

    .step_curr_th  = IGBT_STEP_CURR_TH,
    .over_curr_th  = IGBT_OVERCURR_TH,
    .limit_curr_th = IGBT_LIMIT_CURR_TH,

    .evt_code = 0,
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

