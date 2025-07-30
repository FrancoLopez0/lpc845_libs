/*
 * DELAY.h
 *
 *  Created on: 6 jul. 2024
 *      Author: Franco
 */

#ifndef DELAY_H_
#define DELAY_H_
#include "LPC845.h"

extern volatile uint32_t flag_tick;
extern volatile uint32_t program_time;

//uint32_t get_deta_t(void);

void SysTick_Handler(void);

void delay_ms(int);

void SYSTICK_INIT(void);

uint32_t get_curret_time(void);

#endif /* DELAY_H_ */
