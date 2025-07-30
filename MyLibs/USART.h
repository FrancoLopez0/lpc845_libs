/*
 * USART.h
 *
 *  Created on: 16 ago. 2024
 *      Author: Franco
 */

#ifndef L1_L2_USART_H_
#define L1_L2_USART_H_

#include "fsl_usart.h"
#include "board.h"
#include <math.h>
//#include "TIME.h"

#define MAX_BUFF_SIZE 64
#define END_TX '.'

extern uint8_t flagReceived , buff_data_usart[MAX_BUFF_SIZE], counter, aux;


void write_byte(uint8_t);

void usart1_write(char *);

void usart_comand(uint8_t *, char *, char *, char *);

void clean_buffer(void);

uint32_t get_num_from_usart(char *);

#endif /* L1_L2_USART_H_ */
