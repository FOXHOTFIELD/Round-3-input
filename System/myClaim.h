#include <stdint.h>
#ifndef __MYCLAIM_H
#define __MYCLAIM_H

/* 在定时器中断中使用的变量需要设为 volatile */
extern volatile int16_t speed1, speed2;
extern volatile uint16_t adcf1, adcf2, adcf3;

extern int16_t thrd_BLACK, thrd_WHITE;

extern uint8_t g_thrd_correct_wip;

extern volatile float v1, v2, v3;
extern volatile float offset;          //if offset > 0, turn right.
extern volatile float Kp, Ki, Kd;					//比例项，积分项，微分项的权重
extern volatile int8_t adcf0, adcf4;

#endif
