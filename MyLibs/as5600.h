/*
 * as5600.h
 *
 *  Created on: 5 feb 2025
 *      Author: User
 */

#ifndef AS5600_H_
#define AS5600_H_
#include "fsl_i2c.h"

#define AS5600_ADDR 0x36
#define CANT_TICKS 4096
#define REV_CM 21.7
#define REV_MM 217.0
//#define DISTANCE_FOR_TICK_MM REV_MM/((float)CANT_TICKS)
#define DISTANCE_FOR_TICK_MM 0.0525
#define DISTANCE_FOR_DEG_MM 0.72f//0.6028f

typedef struct{
	float angle;
	float prev_angle;
	float get_vel_mm;
	uint32_t current_time;
	uint32_t prev_time;
}as5600_t;

void INIT_AS5600(I2C_Type * i2c_chann);
float as5600_get_vel_mm(float current_angle,float prev_angle, int delta_time_ms);
uint16_t as5600_get_raw(I2C_Type * i2c_chann);
int as5600_get_angle_12bit(I2C_Type * i2c_chann);
float as5600_get_angle_deg(I2C_Type * i2c_chann);
#endif /* AS5600_H_ */
