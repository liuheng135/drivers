#ifndef __HR_TIMER_H_
#define __HR_TIMER_H_

#include <stdint.h>

void hrtimer_init(void);
float hrtimer_get_time(void);
float hrtimer_elapse_time(float start);
void mdelay(uint32_t ms);

#endif
