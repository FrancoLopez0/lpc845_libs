/*
 * _PWM.h
 *
 *  Created on: 15 oct. 2024
 *      Author: Franco López
 */

#ifndef PWM_H_
#define PWM_H_

#define SCTIMER_CLK_FREQ CLOCK_GetFreq(kCLOCK_Fro)
#define MAX_WIDTH 1023U

#include "fsl_sctimer.h"

typedef struct{
	uint32_t pulse_width;
	sctimer_pwm_level_select_t level;
	sctimer_out_t output;
	uint32_t event;
	uint16_t resolution;
}pwm_t;

status_t PWM_SetUp(SCT_Type * ,const pwm_t * ,sctimer_pwm_mode_t ,uint32_t ,uint32_t ,uint32_t *);

void PWM_Update(SCT_Type *, sctimer_out_t, uint16_t, uint32_t);

void PWM_DEFAULT_CONFIG(pwm_t *);

void INIT_PWM(pwm_t *);

void _SCTIMER_CONFIG(void);

#endif /* PWM_H_ */
