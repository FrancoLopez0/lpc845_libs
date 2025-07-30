/*
 * FastMath.h
 *
 *  Created on: 25 oct. 2024
 *      Author: Franco
 */

#ifndef MYLIBS_FASTMATH_H_
#define MYLIBS_FASTMATH_H_
#include "stdint.h"
#define M_PI 3.1415926535897932384626433832795
#define PI_180 57.29577951308232// 180/PI
#define PI_2 1.5707963267948966
#define PI_2_neg -1.5707963267948966
#include "math.h"
typedef union
{
    float f;
    struct
    {
        unsigned int mantisa : 23;
        unsigned int exponent : 8;
        unsigned int sign : 1;
    } parts;
} float_cast;

int FastCosInt_e4(int x);
int FastSinInt_e4(int);
int FastAtanInt_e4(int);
int FastAtanInt_e4_z_inv(int x);
int FastAtan2Int(int y, int x);
float divide_floats(float, float);
float multiply_floats(float, float);
float add_floats(float, float );
float subtract_floats(float, float );
float mult_neg(float);
float FastAtan(float);
float FastAtanINV(float);
float FastAtan2f(float ,float );
float FastAtan2i(int, int);

#endif /* MYLIBS_FASTMATH_H_ */
