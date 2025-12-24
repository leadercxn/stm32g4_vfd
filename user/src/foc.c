#include "string.h"
#include "boards.h"
#include "foc.h"
#include "adc.h"
#include "timer.h"
#include "util.h"
#include "math.h"

#include "parameters.h"
#include "sensors_task.h"

#include "trace.h"

/**
 * @brief 弧度归一化 [0 , 2PI]
 */
float radian_normalize(float radian)
{
    float a;

	a = fmod(radian, DOUBLE_PI);
	
	return (a >= 0) ? a : (a + DOUBLE_PI);
}

void pi_cal(pi_cal_t *sptr, float error)
{
    sptr->out     = sptr->kp * error + sptr->ki * sptr->sum_err;
    sptr->sum_err += error;

    //积分限幅
    if(sptr->sum_err>=sptr->iout_max)
        sptr->sum_err=sptr->iout_max;
    else if (sptr->sum_err<=-sptr->iout_max)
        sptr->sum_err=-sptr->iout_max;

    //输出限幅
    if(sptr->out >= sptr->out_max)
        sptr->out = sptr->out_max;
    else if (sptr->out <= -sptr->out_max)
        sptr->out=-sptr->out_max;
}


/**
 * @brief 设置力矩 , 传入需要进行 iPark 变换的参数
 * 
 * @param uq q轴电压，单位：V
 * @param ud d轴电压，单位：V， 常规为 0
 * @param pos_radian 位置角度，单位：弧度
 */
void torque_set(float uq, float ud, float pos_radian)
{
    foc_param_t foc_u;

    foc_u.q = uq;  // q轴电压
    foc_u.d = ud;  // d轴电压

/**
 * 参考灯哥FOC
 * 验证波形正确
 */
#if 1

    float u_out = 0;

    if(ud)
    {
        u_out = sqrtf(uq * uq + ud * ud) / VBUS_VLOT;  // 计算输出电压
        foc_u.pos_radian = radian_normalize(pos_radian + atan2f(uq, ud));               // 归一化位置角度
    }
    else
    {
        u_out = uq / VBUS_VLOT;    // 计算输出电压
        foc_u.pos_radian = radian_normalize(pos_radian);            // 归一化位置角度
    }

    int sector = (int)(foc_u.pos_radian / PI_DIV_3) + 1;            // 计算扇区
    float t0 = 0.0f, t1 = 0.0f, t2 = 0.0f;
    float ts = PWM_PERIOD;                                          // 因为中心对齐，20KHz PWM 的周期是 50us

    uint32_t tu = 0, tv = 0, tw = 0;                                // 三相 PWM 占空比

    t1 = SQRT_3 * u_out  * sin(sector * PI_DIV_3 - foc_u.pos_radian);
    t2 = SQRT_3 * u_out  * sin(foc_u.pos_radian - (sector - 1)  * PI_DIV_3);
    t0 = 1 - t1 - t2;

    // 根据分区设置时间
    if(sector == 1)
    {
        tu = (uint32_t) ((t1 + t2 + t0 * 0.5f) * ts);  // u相占空比
        tv = (uint32_t) ((t2 + t0 * 0.5f) * ts);       // v相占空比
        tw = (uint32_t) ((t0 * 0.5) * ts);             // w相占空比
    }
    else if(sector == 2)
    {
        tu = (uint32_t) ((t1 + t0 * 0.5f) * ts);       // u相占空比
        tv = (uint32_t) ((t1 + t2 + t0 * 0.5f) * ts);  // v相占空比
        tw = (uint32_t) ((t0 * 0.5) * ts);             // w相占空比
    }
    else if(sector == 3)
    {
        tu = (uint32_t) ((t0 * 0.5f) * ts);            // u相占空比
        tv = (uint32_t) ((t1 + t2 + t0 * 0.5f) * ts);  // v相占空比
        tw = (uint32_t) ((t2 + t0 * 0.5f) * ts);       // w相占空比
    }
    else if(sector == 4)
    {
        tu = (uint32_t) ((t0 * 0.5f) * ts);            // u相占空比
        tv = (uint32_t) ((t1 + t0 * 0.5f) * ts);       // v相占空比
        tw = (uint32_t) ((t1 + t2 + t0 * 0.5f) * ts);  // w相占空比
    }
    else if(sector == 5)
    {

        tu = (uint32_t) ((t2 + t0 * 0.5f) * ts);       // u相占空比
        tv = (uint32_t) ((t0 * 0.5f) * ts);            // v相占空比
        tw = (uint32_t) ((t1 + t2 + t0 * 0.5f) * ts);  // w相占空比
    }
    else if(sector == 6)
    {

        tu = (uint32_t) ((t1 + t2 + t0 * 0.5f) * ts);  // u相占空比
        tv = (uint32_t) ((t0 * 0.5f) * ts);            // v相占空比
        tw = (uint32_t) ((t1 + t0 * 0.5f) * ts);       // w相占空比
    }

#if 0
    trace_debug("sector %d, uq %f, radian %f\r\n", sector, uq, foc_u.pos_radian);
    trace_debug("t0 %f->%f, t1 %f->%f, t2 %f->%f\r\n", t0, t0 * 50.f, t1, t1 * 50.0f ,t2, t2 * 50.0f);
    trace_debug("tu %lu, tv %lu, tw %lu\r\n", tu, tv, tw);
#endif

#endif

    phase_pwm_set(tu, tv, tw);
}


/**
 * @brief 设置uvw三相的PWMs输出电压, 在 iClark 变换后使用
 * 
 * @param u 期望U相电压，单位：V
 * @param v 期望V相电压，单位：V
 * @param w 期望W相电压，单位：V
 */
void svpwm_set(float u, float v, float w)
{
    //限幅
    float u_value = CONSTRAIN( u, 0, VBUS_VLOT );
    float v_value = CONSTRAIN( v, 0, VBUS_VLOT );
    float w_value = CONSTRAIN( w, 0, VBUS_VLOT );

    //计算各通道占空比
    float u_pwm_per = u_value / VBUS_VLOT;
    float v_pwm_per = v_value / VBUS_VLOT;
    float w_pwm_per = w_value / VBUS_VLOT;

    // 将电压转换为PWM占空比
    uint32_t u_pwm = (uint32_t)(u_pwm_per * PWM_PERIOD);
    uint32_t v_pwm = (uint32_t)(v_pwm_per * PWM_PERIOD);
    uint32_t w_pwm = (uint32_t)(w_pwm_per * PWM_PERIOD);

    phase_pwm_set(u_pwm, v_pwm, w_pwm);
}