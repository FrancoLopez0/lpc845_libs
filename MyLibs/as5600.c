/*
 * as5600.c
 *
 *  Created on: 5 feb 2025
 *      Author: Franco Lopez
 */
#include "as5600.h"

void INIT_AS5600(I2C_Type * i2c_chann)
{
	uint8_t data[2];
	uint32_t r;
	data[0]=0x0C;
	data[1]=0x0D;

	if (kStatus_Success == I2C_MasterStart(i2c_chann, AS5600_ADDR, kI2C_Write))
	{
		r = I2C_MasterWriteBlocking(i2c_chann, data, 1, 0);
		r = I2C_MasterStop(i2c_chann);
	}
}

int as5600_reset(as5600_t *sense,I2C_Type * i2c_channel)
{
	sense->angle = 0.0;
	sense->prev_angle = 0.0;
	sense->get_vel_mm = 0.0;
	sense->current_time = 0;
	sense->prev_time = 0;
}

int as5600_get_angle_12bit(I2C_Type * i2c_chann)
{
	uint32_t r;
	uint8_t x[2];
	if (kStatus_Success == I2C_MasterRepeatedStart(i2c_chann, AS5600_ADDR, kI2C_Read))
	{
		r = I2C_MasterReadBlocking(i2c_chann, x, 2, 0);
		r = I2C_MasterStop(i2c_chann);
		r = x[0]*256 + x[1];
	}
	return r;
}

float as5600_get_angle_deg(I2C_Type * i2c_chann)
{
	uint16_t r;
	uint8_t x[2];
	if (kStatus_Success == I2C_MasterRepeatedStart(i2c_chann, AS5600_ADDR, kI2C_Read))
	{
		r = I2C_MasterReadBlocking(i2c_chann, x, 2, 0);
		r = I2C_MasterStop(i2c_chann);
		r = ((x[0] << 8) | x[1]) & 0x0FFF;
	}
	return ((float)r) * (360.0/4095.0);
}

uint16_t as5600_get_raw(I2C_Type * i2c_chann)
{
	uint16_t r;
	uint8_t x[2];
	if (kStatus_Success == I2C_MasterRepeatedStart(i2c_chann, AS5600_ADDR, kI2C_Read))
	{
		r = I2C_MasterReadBlocking(i2c_chann, x, 2, 0);
		r = I2C_MasterStop(i2c_chann);
		r = ((x[0] << 8) | x[1]) & 0x0FFF;
	}
	return r;
}

float as5600_get_vel_cm(float current_angle,float prev_angle, int delta_time_ms)
{
	if(delta_time_ms>0)
	{
		float angle_diff = current_angle - prev_angle;

		if (angle_diff > 180) {
			angle_diff -= 360;  // Ajustar la diferencia de ángulo a -180°
		} else if (angle_diff < -180) {
			angle_diff += 360;  // Ajustar la diferencia de ángulo a 180°
		}

		return abs(angle_diff) * 1000.0 *(REV_CM/360) / (float)(delta_time_ms);
	}
	return 0;
}

float as5600_get_vel_mm(float current_angle,float prev_angle, int delta_time_ms)
{
	if(delta_time_ms>0)
	{
		float angle_diff = current_angle - prev_angle;

		if (angle_diff > 180) {
			angle_diff -= 360;  // Ajustar la diferencia de ángulo a -180°
		} else if (angle_diff < -180) {
			angle_diff += 360;  // Ajustar la diferencia de ángulo a 180°
		}

		return abs(angle_diff) * 1000.0 *(DISTANCE_FOR_DEG_MM) / (float)(delta_time_ms);
	}
	return 0;
}
