#ifndef NN_HEADER_H__
#define NN_HEADER_H__

#include "arm_math.h"
#include "math_helper.h"
#if defined(SEMIHOSTING)
#include <stdio.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif
	
union data{
	uint32_t * intval;
	float32_t * floatval;
};

void multiplication_test();

void sigmoid_test();
	
void sigmoid(float32_t * input, float32_t * output, int size);

/**@brief  autoencoder. */
void autoencode(float32_t * input, float32_t * output);

///**@brief  CN */
void cn(float32_t * input4, float32_t * input5, float32_t * input7, float32_t * input6, float32_t * output);

///**@brief  PC ATTN */
void pc_attn(float32_t input3[], float32_t input2[], float32_t * output);

#ifdef __cplusplus
}
#endif
#endif // NN_HEADER_H__

