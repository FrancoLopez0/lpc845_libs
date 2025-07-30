/*
 * Filters.c
 *
 *  Created on: 17 feb 2025
 *      Author: User
 */
#include "Filters.h"



// Función de Runge-Kutta de 4to orden para la cinemática del robot
void runge_kutta_update(float* x, float* y, float* theta, float robot_vel, float robot_angular_vel, float dt) {
    // k1 para las ecuaciones dx/dt, dy/dt y dtheta/dt
    float k1x = robot_vel * cosf(*theta) * dt;
    float k1y = robot_vel * sinf(*theta) * dt;
    float k1theta = robot_angular_vel * dt;

    // k2 para las ecuaciones dx/dt, dy/dt y dtheta/dt
    float k2x = robot_vel * cosf(*theta + k1theta / 2.0f) * dt;
    float k2y = robot_vel * sinf(*theta + k1theta / 2.0f) * dt;
    float k2theta = robot_angular_vel * dt;

    // k3 para las ecuaciones dx/dt, dy/dt y dtheta/dt
    float k3x = robot_vel * cosf(*theta + k2theta / 2.0f) * dt;
    float k3y = robot_vel * sinf(*theta + k2theta / 2.0f) * dt;
    float k3theta = robot_angular_vel * dt;

    // k4 para las ecuaciones dx/dt, dy/dt y dtheta/dt
    float k4x = robot_vel * cosf(*theta + k3theta) * dt;
    float k4y = robot_vel * sinf(*theta + k3theta) * dt;
    float k4theta = robot_angular_vel * dt;

    // Actualización de las posiciones y la orientación con RK4
    *x += (k1x + 2.0f * k2x + 2.0f * k3x + k4x) / 6.0f;
    *y += (k1y + 2.0f * k2y + 2.0f * k3y + k4y) / 6.0f;
    *theta += (k1theta + 2.0f * k2theta + 2.0f * k3theta + k4theta) / 6.0f;
}

uint32_t lowpass_filter_uint(uint32_t value, uint32_t prev_value_filtered, uint32_t alpha_filter_e3)
{
	return (alpha_filter_e3 * value + prev_value_filtered * (1000 - alpha_filter_e3))/1000;
}

int lowpass_filter_int(int value, int prev_value_filtered, uint32_t alpha_filter_e3)
{
	return (alpha_filter_e3 * value + prev_value_filtered * (1000 - alpha_filter_e3))/1000;
}

void append_to_buffer(float* buffer, int buff_size, float value, int* index)
{
	buffer[*index] = value;
	*index = (*index + 1) % buff_size;
}

void moving_average_filter(float* buffer, int buff_size, float* sum, int* index, float new_value, float* avg) {
    // Obtener el índice del valor más antiguo antes de actualizar el buffer
    int old_index = *index;

    // Restar el valor antiguo del buffer de la suma acumulada
    *sum -= buffer[old_index];

    // Agregar el nuevo valor al buffer y actualizar la suma acumulada
    buffer[old_index] = new_value;
    *sum += new_value;

    // Avanzar el índice circularmente
    *index = (old_index + 1) % buff_size;

    // Calcular la nueva media
    *avg = *sum / buff_size;
}

float normalize_angle_180(float angle)
{
	return fmodf((angle + PI_F), (2.0f * PI_F)) - PI_F;
}

float get_distance(float x, float y)
{
	return  sqrtf(x * x + y * y);
}

void get_filtered(float new_sample, float *value_filtered,float *samples, int size_samples, float *accumulator,int *buff_index,bool *is_not_full)
{
	if(*is_not_full)
	{
		samples[*buff_index] = new_sample;
		*accumulator+=samples[*buff_index];

		*value_filtered = new_sample;

		(*buff_index)++;

		if(*buff_index == size_samples)
		{
			*buff_index = 0;

			*is_not_full = false;
		}
	}
	else{
		moving_average_filter(samples, size_samples, accumulator, buff_index, new_sample, value_filtered);
	}
}

void moving_average_filter_angle(float* buffer, int buff_size, float* sum_cos, float* sum_sin, int* index, float new_angle, float* avg_angle) {
    // Convertir el ángulo a coordenadas (cos, sin)
    float cos_a = cosf(new_angle);
    float sin_a = sinf(new_angle);

    // Restar los valores antiguos de la suma
    int old_index = *index;
    *sum_cos -= cosf(buffer[old_index]);
    *sum_sin -= sinf(buffer[old_index]);

    // Agregar el nuevo ángulo al buffer y actualizar la suma
    buffer[old_index] = new_angle;
    *sum_cos += cos_a;
    *sum_sin += sin_a;

    // Avanzar el índice circularmente
    *index = (old_index + 1) % buff_size;

    // Calcular el promedio angular usando atan2
    *avg_angle = atan2f(*sum_sin, *sum_cos);
}
