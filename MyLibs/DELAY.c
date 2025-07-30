/*
 * DELAY.c
 *
 *  Created on: 6 jul. 2024
 *      Author: Franco
 */
#include "DELAY.h"

volatile uint32_t flag_tick;
volatile uint32_t program_time;

//uint32_t time_ant = 0;
//uint32_t delta_time_ms = 0;
//
//uint32_t get_deta_t(void){
//	delta_time_ms = (time - time_ant);
//	time_ant = time;
//	return delta_time_ms;
//}


void SYSTICK_INIT(void){
	SysTick_Config(SystemCoreClock/1000);
	flag_tick = 0;
	program_time = 0;
}

void SysTick_Handler(void){
	flag_tick++;
	program_time++;
}

void delay_ms(int final){
	flag_tick=0;
	while(flag_tick <= final);
	flag_tick = 0;
}

uint32_t get_curret_time(void){
	return program_time;
}
