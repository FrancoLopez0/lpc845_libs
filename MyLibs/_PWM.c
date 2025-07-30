/*
 * _PWM.c
 *
 *  Created on: 15 oct. 2024
 *      Author: Franco López
 */
#include "_PWM.h"

//Para que funcione correctamenta hay que agregar a la libreria del sctimer
//uint32_t SCTIMER_GetCurrentEvent(void){
//	return s_currentEvent;
//}

void _SCTIMER_CONFIG(void){
	sctimer_config_t sctimerInfo;

	SCTIMER_GetDefaultConfig(&sctimerInfo);

	/* Initialize SCTimer module */
	SCTIMER_Init(SCT0, &sctimerInfo);
}

status_t PWM_SetUp(SCT_Type *base,
        const pwm_t *pwmParams,
        sctimer_pwm_mode_t mode,
        uint32_t pwmFreq_Hz,
        uint32_t srcClock_Hz,
        uint32_t *event){

	status_t status = kStatus_Fail;
	status_t status2 = kStatus_Fail;
	uint32_t period, pulsePeriod = 0;
	uint32_t sctClock    = srcClock_Hz / (((base->CTRL & SCT_CTRL_PRE_L_MASK) >> SCT_CTRL_PRE_L_SHIFT) + 1U);
	uint32_t periodEvent = 0, pulseEvent = 0;
	uint32_t reg;

	assert(pwmParams->pulse_width <= MAX_WIDTH);

	base->CTRL |= SCT_CTRL_BIDIR_L_MASK;

	period = sctClock / (pwmFreq_Hz * 2U);

	if ((SCTIMER_GetCurrentEvent()+2U)<= (uint32_t)FSL_FEATURE_SCT_NUMBER_OF_EVENTS){
		pulsePeriod = (uint32_t)(((uint64_t)period * pwmParams->pulse_width) / pwmParams->resolution);

		status = SCTIMER_CreateAndScheduleEvent(base, kSCTIMER_MatchEventOnly, period, 0, kSCTIMER_Counter_U, &periodEvent);

		status2 = SCTIMER_CreateAndScheduleEvent(base, kSCTIMER_MatchEventOnly, pulsePeriod, 0, kSCTIMER_Counter_U, &pulseEvent);

        /* Reset the counter when we reach the PWM period */
        SCTIMER_SetupCounterLimitAction(base, kSCTIMER_Counter_U, periodEvent);

        /* Return the period event to the user */
        *event = periodEvent;

        /* For high-true level */
        if ((uint32_t)pwmParams->level == (uint32_t)kSCTIMER_HighTrue)
        {
                /* Set the initial output level to high which is the active state */
                base->OUTPUT |= (1UL << (uint32_t)pwmParams->output);
                /* Clear the output when we reach the PWM pulse event */
                SCTIMER_SetupOutputClearAction(base, (uint32_t)pwmParams->output, pulseEvent);
                /* Reverse output when down counting */
                reg = base->OUTPUTDIRCTRL;
                reg &= ~((uint32_t)SCT_OUTPUTDIRCTRL_SETCLR0_MASK << (2U * (uint32_t)pwmParams->output));
                reg |= (1UL << (2U * (uint32_t)pwmParams->output));
                base->OUTPUTDIRCTRL = reg;
        }
        /* For low-true level */
        else
        {
                /* Set the initial output level to low which is the active state */
                base->OUTPUT &= ~(1UL << (uint32_t)pwmParams->output);
                /* Set the output when we reach the PWM pulse event */
                SCTIMER_SetupOutputSetAction(base, (uint32_t)pwmParams->output, pulseEvent);
                /* Reverse output when down counting */
                reg = base->OUTPUTDIRCTRL;
                reg &= ~((uint32_t)SCT_OUTPUTDIRCTRL_SETCLR0_MASK << (2U * (uint32_t)pwmParams->output));
                reg |= (1UL << (2U * (uint32_t)pwmParams->output));
                base->OUTPUTDIRCTRL = reg;
        }

	}
	else
	{
	   status = kStatus_Fail;
	}

	return status;
}

void PWM_Update(SCT_Type *base, sctimer_out_t output, uint16_t pulse_width, uint32_t event)
{
    assert(pulse_width <= MAX_WIDTH);
    assert((uint32_t)output < (uint32_t)FSL_FEATURE_SCT_NUMBER_OF_OUTPUTS);
    assert(1U == (base->CONFIG & SCT_CONFIG_UNIFY_MASK));

    uint32_t periodMatchReg, pulseMatchReg;
    uint32_t pulsePeriod = 0, period;
    bool isHighTrue      = (0U != (base->OUT[output].CLR & (1UL << (event + 1U))));

    /* Retrieve the match register number for the PWM period */
    periodMatchReg = base->EV[event].CTRL & SCT_EV_CTRL_MATCHSEL_MASK;

    /* Retrieve the match register number for the PWM pulse period */
    pulseMatchReg = base->EV[event + 1U].CTRL & SCT_EV_CTRL_MATCHSEL_MASK;

    period = base->MATCH[periodMatchReg];

    /* Stop the counter before updating match register */
    SCTIMER_StopTimer(base, (uint32_t)kSCTIMER_Counter_U);

    /* For 100% dutycyle, make pulse period greater than period so the event will never occur */
    if (pulse_width >= MAX_WIDTH)
    {
        pulsePeriod = period + 2U;

        /* Set the initial output level base on output mode */
        if (isHighTrue)
        {
            base->OUTPUT |= (1UL << (uint32_t)output);
        }
        else
        {
            base->OUTPUT &= ~(1UL << (uint32_t)output);
        }
    }
    else
    {
        pulsePeriod = (uint32_t)(((uint64_t)period * pulse_width) / MAX_WIDTH);
    }

    /* Update dutycycle */
    base->MATCH[pulseMatchReg]    = pulsePeriod;
    base->MATCHREL[pulseMatchReg] = pulsePeriod;

    /* Restart the counter */
    SCTIMER_StartTimer(base, (uint32_t)kSCTIMER_Counter_U);
}

void PWM_DEFAULT_CONFIG(pwm_t * pwm){
	pwm->level = kSCTIMER_LowTrue;
	pwm->output = kSCTIMER_Out_1;
	pwm->pulse_width = 0;
	pwm->resolution = 1024;
}

void INIT_PWM(pwm_t *PWM){

	assert(PWM->pulse_width <= MAX_WIDTH);

	uint32_t sctimerClock;

	sctimerClock = SCTIMER_CLK_FREQ;

	/* Configure first PWM with frequency 24kHZ from first output */
	SCTIMER_StopTimer(SCT0, kSCTIMER_Counter_U);

	if (PWM_SetUp(SCT0, PWM, kSCTIMER_CenterAlignedPwm, 24000, sctimerClock, &(PWM->event)) == kStatus_Fail)
	    {
	    	return;
	    }

}
