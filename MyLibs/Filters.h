/*
 * Filters.h
 *
 *  Created on: 17 feb 2025
 *      Author: Franco A. Lopez
 */

#ifndef MYLIBS_FILTERS_H_
#define MYLIBS_FILTERS_H_
#include "board.h"
#include "math.h"

#define PI_F 3.1415927f

void runge_kutta_update(float* x, float* y, float* theta, float robot_vel, float robot_angular_vel, float dt);

uint32_t lowpass_filter_uint(uint32_t value, uint32_t prev_value_filtered, uint32_t alpha_filter_e3);

int lowpass_filter_int(int value, int prev_value_filtered, uint32_t alpha_filter_e3);

void moving_average_filter_angle(float* buffer, int buff_size, float* sum_cos, float* sum_sin, int* index, float new_angle, float* avg_angle);

void get_filtered(float new_sample, float *value_filtered,float *samples, int size_samples, float *accumulator,int *buff_index,bool *is_not_full);

float get_distance(float x, float y);

float normalize_angle_180(float angle);

void moving_average_filter(float* buffer, int buff_size, float* sum, int* index, float new_value, float* avg);

void append_to_buffer(float* buffer, int buff_size, float value, int* index);

#endif /* MYLIBS_FILTERS_H_ */
