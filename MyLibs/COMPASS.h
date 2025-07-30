/*
 * COMPASS.h
 *
 *  Created on: 7 jul. 2024
 *      Author: Franco A. López
 */

/**
 * @file    COMPASS.h
 * @brief   HMC5883L Handler.
 */

#ifndef COMPASS_H_
#define COMPASS_H_
#define HMC5883L  0x0D

#include "fsl_i2c.h"
#include "_I2C_.h"
#include "FastMath.h"

#define M_PI_4 (M_PI/4.0)
#define PI_F 3.1415927f
#define A 0.0776509570923569
#define B -0.287434475393028
#define C (M_PI_4 - A - B)

typedef struct{
	int mx;
	int my;
	int mz;
	int last_mx;
	int last_my;
	int azimuthInt;
	float azimuth;
	float direction;
	float magnetic_declination;
	float set_angle_objective;
	float delta_angle;
	float zero;
	float orientation;
}hmc5883l_t;

extern hmc5883l_t magnetometer;
void INIT_QMC5883l(I2C_Type *i2c_chann);
float getRelativeOrientation(float current_theta,float theta0);
void qmc5883l_update(hmc5883l_t* mag,I2C_Type *i2c_chann);
/**
 * @brief Init communication and config
 *
 * @return void
 * */
void hmc5883l_init(void);

/**
 * @brief Call hmc5883l and send data
 *
 * @param tx               pointer to send buffer
 * @param buff_size buffer size
 *
 * @return void
 */
void hmc5883l(uint8_t*, uint8_t);

/**
 * @brief Update all measurements
 *
 * @param rxBuff Pointer to rx buffer
 *
 */
void hmc5883l_update(int32_t *);

/**
 * @brief Get azimuth
 *
 * @param sense Pointer to variable to storage
 */
void hmc5883l_azimuth(hmc5883l_t*);

/**
 * @brief Get the measurements
 */
void hmc5883l_get(hmc5883l_t*);

void hmc5883l_set_zero(hmc5883l_t* sense);

#endif /* COMPASS_H_ */
