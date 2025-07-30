/*
 * COMPASS.c
 *
 *  Created on: 7 jul. 2024
 *      Author: Franco
 */

#include "COMPASS.h"


#define COEFF_A_FILTER 0.95
#define COEFF_B_FILTER 0.05

//TODO: Se puede mejorar la precision cambiando las constantes a flotantes


const float max_x =631;//617;//-1175;//552;//397;//517;//846;//1014;//= 797;
const float min_x =-223;//-280;//-191;//-383;//-530;//-522;//-93;//50;//= -90;
const float max_y =1667;//-958;//511;//841;//1197;//618;//757;//1148;
const float min_y =712;//263;//-497;//-27;//190;//-352;//236;

const float bias_x =(max_x + min_x)/2;
const float delta_x = (max_x - min_x)/2;

const float bias_y =(max_y + min_y)/2;
const float delta_y = (max_y - min_y)/2;

const float delta_avg = (delta_y+delta_x)/2;

const float scale_x = delta_avg / delta_x;
const float scale_y = delta_avg / delta_y;

hmc5883l_t magnetometer;

void QMC5883L_WriteRegister(uint8_t reg, uint8_t value, I2C_Type *i2c_chann){
    uint8_t data[2] = {reg, value};  // Dirección del registro y valor a escribir

    // Enviar dirección y dato a través de I2C
    I2C_MasterWriteBlocking(i2c_chann, data, sizeof(data), 0);
}

uint8_t QMC5883L_ReadRegister(uint8_t reg, I2C_Type *i2c_chann) {
    uint8_t value = 0;

    // Escribir el registro que deseamos leer (fase de escritura)
    I2C_MasterWriteBlocking(i2c_chann, &reg, 1, 0);  // Enviar solo la dirección del registro

    // Leer el valor del registro (fase de lectura)
    I2C_MasterReadBlocking(i2c_chann, &value, 1, 0);  // Leer el byte de datos

    return value;
}

void QMC5883L_ReadRegisters(uint8_t start_reg, uint8_t *data, uint8_t length, I2C_Type *i2c_chann) {
    // Escribir el registro de inicio para la lectura secuencial (fase de escritura)
    I2C_MasterWriteBlocking(i2c_chann, &start_reg, 1, 0); // Enviar solo la dirección del registro
    I2C_MasterStop(i2c_chann);

    I2C_MasterRepeatedStart(i2c_chann, HMC5883L, kI2C_Read);
    // Leer múltiples bytes (fase de lectura)
    I2C_MasterReadBlocking(i2c_chann, data, length, 0); // Leer los registros secuenciales
    I2C_MasterStop(i2c_chann);
    // Los registros se leen secuencialmente ya que la dirección se incrementa automáticamente
}

void INIT_QMC5883l(I2C_Type *i2c_chann)
{
	uint8_t data[2];
	uint32_t r;
	data[0]=0x00;
	data[1]=0x0D;

	if (kStatus_Success == I2C_MasterStart(i2c_chann, HMC5883L, kI2C_Write))
	{
		r = I2C_MasterWriteBlocking(i2c_chann, data, 1, 0);
		r = I2C_MasterStop(i2c_chann);
	}
}

//void qmc5883l_update(hmc5883l_t* mag)
//{
//
//}

void qmc5883l_update(hmc5883l_t* mag,I2C_Type *i2c_chann) {
    uint8_t data[6];  // Arreglo para almacenar los 6 bytes de datos (X, Y, Z)
    I2C_MasterStop(i2c_chann);
    // Leer los 6 registros secuenciales de datos a partir del registro de datos de X (0x00)

    if (kStatus_Success == I2C_MasterRepeatedStart(i2c_chann, HMC5883L, kI2C_Write))
    {
		QMC5883L_ReadRegisters(0x00, data, 6, i2c_chann);
    }

    mag->mx = (int16_t)(data[0] | data[1] << 8);
    mag->my= (int16_t)(data[2] | data[3] << 8);
    mag->mz= (int16_t)(data[4] | data[5] << 8);

    mag->mx = (int16_t)(scale_x * (mag->mx - bias_x));
    mag->my = (int16_t)(scale_y * (mag->my - bias_y));

    hmc5883l_azimuth(mag);
}

float getRelativeOrientation(float current_theta,float theta0){
//    float current_theta = readAzimuth();
    float rel_theta = current_theta - theta0;

    // Ajuste para mantener el ángulo en el rango -180° a 180°
    if (rel_theta > PI_F) rel_theta -= 2*PI_F;
    if (rel_theta < -PI_F) rel_theta += 2*PI_F;

    return rel_theta;
}

void hmc5883l_init(void){
	uint8_t txBuff[2] = {0,0};

	txBuff[0] = 0xA;
	txBuff[1] = 0x80; // SET/RESET
	hmc5883l(txBuff, 2);

	txBuff[0] = 0x0B;
	txBuff[1] = 0x1; // SET/RESET
	hmc5883l(txBuff, 2);

	txBuff[0] = 0x09; //Config
	txBuff[1] = 0x1 | (0x2<<2) | (0x01<<4) | (0x0<<6); // continuous measurement mode
	hmc5883l(txBuff, 2);

}

void hmc5883l(uint8_t* tx, uint8_t buff_size){
	I2C1_INIT_WRITE(HMC5883L);

	for(int i=0; i<buff_size; i++)I2C1_SEND_CONTINUE(tx[i]);

	I2C1_STOP();
}

void hmc5883l_update(int32_t *rxBuff){
	uint8_t txBuff[6] = {0x00, 0x1, 0x2, 0x3, 0x4, 0x5};
	for(int i = 0; i < 6; i++)
		{

		I2C1_INIT_WRITE(HMC5883L);

		hmc5883l(&txBuff[i],1);

		I2C1_INIT_READ(HMC5883L);

		I2C1_RECEIVE_CONTINUE(&rxBuff[i], 1);

		I2C1_STOP();

		}

}


void hmc5883l_azimuth(hmc5883l_t* sense){
	sense->azimuth= atan2f((float)sense->my,(float)sense->mx);
//	sense->azimuthInt= FastAtan2Int(sense->my, sense->mx);
//	float z = ((float)sense->my) / ((float) sense->mx);

//	sense->azimuth = FastAtan2i((float)sense->my, (float)sense->mx);



//	sense->azimuth = multiply_floats(sense->azimuth, 180/M_PI);
//	if(z>0){
//		if(z<1){
//			sense->azimuth = fastAtan(z) * (180/PI);
//		}
//		else if(z>1){
//			sense->azimuth = (90 - fastAtan(1/z))*(180/PI);
//		}
//	}
//	if(z<0){
//			z *=(-1);
//			if(z<1){
//				sense->azimuth = fastAtan(z) * (180/PI) * (-1);
//			}
//			else if(z>1){
//				sense->azimuth = (90 - fastAtan(1/z))*(180/PI)*(-1);
//			}
//	}
//	else
//	{
//		sense->azimuth = atanf(x);
//	}
//	sense->azimuth = add_floats(sense->azimuth, sense->magnetic_declination);
//	sense->azimuth+=sense->magnetic_declination;
		//sense->set_angle_objective -=
//	if(sense->azimuth<0){
////		sense->azimuth +=360;
////		sense->azimuth = add_floats(sense->azimuth, 360.0);
////		sense->azimuth = add_floats(sense->azimuth, 360.0);
//		sense->azimuth = sense->azimuth + 2*PI;
//	}
//	sense->delta_angle = sense->set_angle_objective - sense->azimuth ; //Angular distance at desired orientation
//	sense->delta_angle = subtract_floats(sense->set_angle_objective, sense->azimuth);
//	sense->orientation = sense->azimuth + sense->zero - PI_2;
}

void hmc5883l_set_zero(hmc5883l_t* sense)
{
	sense->zero = sense->azimuth;
}

void hmc5883l_get(hmc5883l_t* sense){
	int rxBuff[6];

	hmc5883l_update(rxBuff);

	sense->mx = (int16_t)(rxBuff[0] | rxBuff[1] << 8);
	sense->my= (int16_t)(rxBuff[2] | rxBuff[3] << 8);
	sense->mz= (int16_t)(rxBuff[4] | rxBuff[5] << 8);

//	sense->mx = (int16_t)((1027 * (sense->mx - bias_x)) / 1000);
//	sense->my = (int16_t)((974 * (sense->my - bias_y)) / 1000);

//	sense->mx = (int16_t)(multiply_floats(scale_x, subtract_floats(sense->mx, bias_x)));
//	sense->my = (int16_t)(multiply_floats(scale_y, subtract_floats(sense->my, bias_y)));

//	sense->mx = ((sense->mx) + (sense->last_mx))/2;
//	sense->my = ((sense->my) + (sense->last_my))/2;
//
////	sense->mx = add_floats(multiply_floats(sense->mx, COEFF_A_FILTER), multiply_floats(sense->last_mx, COEFF_B_FILTER));
////	sense->my = add_floats(multiply_floats(sense->my, COEFF_A_FILTER), multiply_floats(sense->last_my, COEFF_B_FILTER));
////
//	sense->last_mx = sense->mx;
//	sense->last_my = sense->my;

	sense->mx = (int16_t)(scale_x * (sense->mx - bias_x));
	sense->my = (int16_t)(scale_y * (sense->my - bias_y));


}

