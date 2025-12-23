#ifndef TIMER_H__
#define TIMER_H__

typedef void (*timer1_irq_cb_t)(void);
typedef void (*timer8_irq_cb_t)(void);

int timer1_init(void);

void phase_pwm_set(uint32_t u, uint32_t v, uint32_t w);
void phase_pwm_start(void);
void phase_pwm_stop(void);

void timer1_irq_cb_register(timer1_irq_cb_t cb);

#endif


