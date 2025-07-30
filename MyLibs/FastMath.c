/*
 * FastMath.c
 *
 *  Created on: 25 oct. 2024
 *      Author: Franco
 */
#include "FastMath.h"

#define Atan_A 0.99921381473541259765625
#define Atan_B -0.321175038814544677734375
#define Atan_C 0.146264731884002685546875
#define Atan_D -3.8986742496490478515625e-2
#define Sin_A 0.4052
#define Sin_A_e3 4052
#define Sin_B_e3 31415

//Lo declaro como const para que se guarde en la ROM, ocupando menos espacio en la RAM
const int FastSin_e4[180] = {
		0, 175, 349, 523, 698, 872, 1045, 1219, 1392, 1564, 1736, 1908, 2079, 2250, 2419, 2588, 2756, 2924, 3090, 3256, 3420, 3584, 3746, 3907, 4067, 4226, 4384, 4540, 4695, 4848, 5000, 5150, 5299, 5446, 5592, 5736, 5878, 6018, 6157, 6293, 6428, 6561, 6691, 6820, 6947, 7071, 7193, 7314, 7431, 7547, 7660, 7771, 7880, 7986, 8090, 8192, 8290, 8387, 8480, 8572, 8660, 8746, 8829, 8910, 8988, 9063, 9135, 9205, 9272, 9336, 9397, 9455, 9511, 9563, 9613, 9659, 9703, 9744, 9781, 9816, 9848, 9877, 9903, 9925, 9945, 9962, 9976, 9986, 9994, 9998, 10000, 9998, 9994, 9986, 9976, 9962, 9945, 9925, 9903, 9877, 9848, 9816, 9781, 9744, 9703, 9659, 9613, 9563, 9511, 9455, 9397, 9336, 9272, 9205, 9135, 9063, 8988, 8910, 8829, 8746, 8660, 8572, 8480, 8387, 8290, 8192, 8090, 7986, 7880, 7771, 7660, 7547, 7431, 7314, 7193, 7071, 6947, 6820, 6691, 6561, 6428, 6293, 6157, 6018, 5878, 5736, 5592, 5446, 5299, 5150, 5000, 4848, 4695, 4540, 4384, 4226, 4067, 3907, 3746, 3584, 3420, 3256, 3090, 2924, 2756, 2588, 2419, 2250, 2079, 1908, 1736, 1564, 1392, 1219, 1045, 872, 698, 523, 349, 175
};

const int FastAtan_e4[101] = {
		0, 573, 1146, 1718, 2291, 2862, 3434, 4004, 4574, 5143, 5711, 6277, 6843, 7407, 7970, 8531, 9090, 9648, 10204, 10758, 11310, 11860, 12407, 12953, 13496, 14036, 14574, 15110, 15642, 16172, 16699, 17223, 17745, 18263, 18778, 19290, 19799, 20304, 20807, 21306, 21801, 22294, 22782, 23268, 23749, 24228, 24702, 25174, 25641, 26105, 26565, 27022, 27474, 27924, 28369, 28811, 29249, 29683, 30114, 30541, 30964, 31383, 31799, 32211, 32619, 33024, 33425, 33822, 34216, 34606, 34992, 35375, 35754, 36129, 36501, 36870, 37235, 37596, 37954, 38309, 38660, 39007, 39352, 39693, 40030, 40365, 40696, 41023, 41348, 41669, 41987, 42302, 42614, 42923, 43229, 43531, 43831, 44128, 44421, 44712, 45000
};

int FastSinInt_e4(int x){
	if(x >= -180 && x < 180)
	{
		if(x > 0) return FastSin_e4[x];
		else return (-1)*FastSin_e4[x];
	}
	return 0;
}

int FastCosInt_e4(int x){
	if(x<0) x=-x;
	x = x + 90;
	while(x>180) x -=180;
	return FastSinInt_e4(x);
}

float multiply_floats(float a, float b) {
    float_cast fa, fb, result;
    fa.f = a;
    fb.f = b;

    // Multiplicación del signo
    result.parts.sign = fa.parts.sign ^ fb.parts.sign;

    // Multiplicación del exponente (ajustar por el sesgo)
    int exponent_a = fa.parts.exponent - 127;
    int exponent_b = fb.parts.exponent - 127;
    int exponent_result = exponent_a + exponent_b + 127;
    result.parts.exponent = exponent_result & 0xFF;

    // Multiplicación de la mantisa (ajustar por el punto implícito)
    unsigned long long mantisa_a = (1 << 23) | fa.parts.mantisa;
    unsigned long long mantisa_b = (1 << 23) | fb.parts.mantisa;
    unsigned long long mantisa_result = (mantisa_a * mantisa_b) >> 23;

    // Normalización si es necesario
    if (mantisa_result & (1 << 24)) {
        mantisa_result >>= 1;
        result.parts.exponent++;
    }

    result.parts.mantisa = mantisa_result & 0x7FFFFF;

    return result.f;
}

float divide_floats(float a, float b) {
	float_cast fa, fb, result;
	    fa.f = a;
	    fb.f = b;

	    // División del signo
	    result.parts.sign = fa.parts.sign ^ fb.parts.sign;

	    // División del exponente (ajustar por el sesgo)
	    int exponent_a = fa.parts.exponent - 127;
	    int exponent_b = fb.parts.exponent - 127;
	    int exponent_result = exponent_a - exponent_b + 127;
	    result.parts.exponent = exponent_result & 0xFF;

	    // División de la mantisa (ajustar por el punto implícito)
	    unsigned long long mantisa_a = (1 << 23) | fa.parts.mantisa;
	    unsigned long long mantisa_b = (1 << 23) | fb.parts.mantisa;
	    unsigned long long mantisa_result = (mantisa_a << 23) / mantisa_b;

	    // Normalización si es necesario
	    if (mantisa_result & (1ULL << 24)) {
	        mantisa_result >>= 1;
	        result.parts.exponent++;
	    }

	    result.parts.mantisa = mantisa_result & 0x7FFFFF;

	    // Asegurar que la mantisa no se desborde
	    while ((mantisa_result & (1ULL << 23)) == 0 && result.parts.exponent > 0) {
	        mantisa_result <<= 1;
	        result.parts.exponent--;
	    }

	    result.parts.mantisa = mantisa_result & 0x7FFFFF;

	    return result.f;
}

float add_floats(float a, float b) {
    float_cast fa, fb, result;
    fa.f = a;
    fb.f = b;

    // Asegurar que fa tiene el mayor exponente
    if (fb.parts.exponent > fa.parts.exponent) {
        float_cast temp = fa;
        fa = fb;
        fb = temp;
    }

    // Alineación de exponentes
    int exponent_diff = fa.parts.exponent - fb.parts.exponent;
    unsigned int mantisa_b = (1 << 23) | fb.parts.mantisa;
    mantisa_b >>= exponent_diff;

    // Suma de mantisas
    unsigned int mantisa_a = (1 << 23) | fa.parts.mantisa;
    unsigned int mantisa_result;
    if (fa.parts.sign == fb.parts.sign) {
        mantisa_result = mantisa_a + mantisa_b;
    } else {
        mantisa_result = mantisa_a - mantisa_b;
    }

    // Ajuste del exponente y normalización
    if (mantisa_result & (1 << 24)) {
        mantisa_result >>= 1;
        fa.parts.exponent++;
    }

    while ((mantisa_result & (1 << 23)) == 0 && fa.parts.exponent > 0) {
        mantisa_result <<= 1;
        fa.parts.exponent--;
    }

    result.parts.mantisa = mantisa_result & 0x7FFFFF;
    result.parts.exponent = fa.parts.exponent;
    result.parts.sign = fa.parts.sign;

    return result.f;
}

float subtract_floats(float a, float b){
    float_cast fa, fb, result;
    fa.f = a;
    fb.f = b;

    // Alineación de exponentes
    if (fa.parts.exponent < fb.parts.exponent) {
        float_cast temp = fa;
        fa = fb;
        fb = temp;
        result.parts.sign = 1;  // Cambia el signo del resultado si intercambiamos los números
    } else {
        result.parts.sign = 0;  // Mantiene el signo del número con mayor exponente
    }

    int exponent_diff = fa.parts.exponent - fb.parts.exponent;
    unsigned int mantisa_b = (1 << 23) | fb.parts.mantisa;
    mantisa_b >>= exponent_diff;

    // Resta de mantisas con ajuste del signo
    unsigned int mantisa_a = (1 << 23) | fa.parts.mantisa;
    unsigned int mantisa_result;

    if (fa.parts.sign == fb.parts.sign) {
        if (mantisa_a >= mantisa_b) {
            mantisa_result = mantisa_a - mantisa_b;
        } else {
            mantisa_result = mantisa_b - mantisa_a;
            result.parts.sign = !result.parts.sign;  // Cambia el signo si mantisa_b es mayor
        }
    } else {
        mantisa_result = mantisa_a + mantisa_b;
    }

    // Normalización
    while ((mantisa_result & (1 << 23)) == 0 && fa.parts.exponent > 0) {
        mantisa_result <<= 1;
        fa.parts.exponent--;
    }

    result.parts.mantisa = mantisa_result & 0x7FFFFF;
    result.parts.exponent = fa.parts.exponent;

    return result.f;
}

float mult_neg(float x){
    float_cast fa;
    fa.f = x;
    fa.parts.sign = !(fa.parts.sign);
    return fa.f;
}

float FastAtan(float x){
    float x2 = multiply_floats(x,x);
    float result = multiply_floats(Atan_D,x2);
    result = multiply_floats(x2, add_floats(Atan_C, result));
    result = multiply_floats(x2, add_floats(Atan_B, result));
    result = multiply_floats(x2, result);
    result = multiply_floats(x, add_floats(Atan_A, result));
    return result;
//    return x * (Atan_A + x2 * (Atan_B + x2 * ( Atan_C+ x2*Atan_D)));
}

float FastAtanINV(float x){
    return FastAtan(divide_floats(1.0,x));
}
//Deben ingresar valores entre -100 y 100
int FastAtanInt_e4_z_inv(int x)
{
    if (x == 0) return FastAtan_e4[0];
    // Aseguramos que no se divida por cero y manejamos la inversión correctamente.
    if (x > 0) return FastAtanInt_e4(10000 / x);  // Cálculo de 1/x
    return -FastAtanInt_e4(10000 / -x);  // Manejo del signo negativo
}

int FastAtanInt_e4(int x)
{
    if(x < 0) {
        return -FastAtanInt_e4(-x);
    }

    if(x > 100) return 90000 - FastAtanInt_e4_z_inv(x);

    return FastAtan_e4[x];
}
//float FastAtan2i(float y, float x){
//
//    float z = divide_floats(y,x);
//    int z = y * 100000 / x;
//
//    if(x>0){
//        if(z < -1) return mult_neg(add_floats((PI_2) , FastAtanINV(z)));
//        if(z > 1) return subtract_floats(PI_2, (FastAtanINV(z)));
//        else return FastAtan(z);
//    }
//    if(x<0){
//        if(y>=0){
//            if(z < -1){
//                float result = multiply_floats(FastAtanINV(z),100.0);
//                result = subtract_floats(multiply_floats((PI_2),100.0), result);
//                result = divide_floats(result, 100.0);
//                return result;
//            }
//            else return add_floats(FastAtan(z), M_PI);
//        }
//        if(y<0)
//        {
//            if(z > 1) return mult_neg(add_floats(PI_2, FastAtanINV(z)));
//            //if(z > 1) return (M_PI/2 + FastAtan(1/z))*(-1);
//            //if(z>1) return subtract_floats(FastAtanUP1(z), M_PI);
//            else return subtract_floats(FastAtan(z), M_PI);
//        }
//    }
//    if(x==0){
//        if(y>0) return PI_2;
//        if(y<0) return PI_2_neg;
//        if(y==0) return 0;
//    }
//    return 0.0;
//}

int FastAtan2Int(int y, int x){

    if(x==0){
            if(y>0) return 90000;
            if(y<0) return -90000;
            if(y==0) return 0;
    }

    int z = y * 100 / x;

    if(x>0){
            return FastAtanInt_e4(z);
        }
        if(x<0){
            if(y>=0){
                return FastAtanInt_e4(z)+ 180000;
            }
            if(y<0)
            {
                return (FastAtanInt_e4(z) - 180000);
            }
        }

    return 0;
}



float FastAtan2f(float y, float x){

    float z = divide_floats(y,x);

    if(x>0){
        if(z < -1) return mult_neg(add_floats((PI_2) , FastAtanINV(z)));
        if(z > 1) return subtract_floats(PI_2, (FastAtanINV(z)));
        else return FastAtan(z);
    }
    if(x<0){
        if(y>=0){
            if(z < -1){
                float result = multiply_floats(FastAtanINV(z),100.0);
                result = subtract_floats(multiply_floats((PI_2),100.0), result);
                result = divide_floats(result, 100.0);
                return result;
            }
            else return add_floats(FastAtan(z), M_PI);
        }
        if(y<0)
        {
            if(z > 1) return mult_neg(add_floats(PI_2, FastAtanINV(z)));
            //if(z > 1) return (M_PI/2 + FastAtan(1/z))*(-1);
            //if(z>1) return subtract_floats(FastAtanUP1(z), M_PI);
            else return subtract_floats(FastAtan(z), M_PI);
        }
    }
    if(x==0){
        if(y>0) return PI_2;
        if(y<0) return PI_2_neg;
        if(y==0) return 0;
    }
    return 0.0;
}

