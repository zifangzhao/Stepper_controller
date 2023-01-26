/*
 * FTL_Sampling.h
 *
 *  Created on: Aug 13, 2020
 *      Author: aeria
 */

#ifndef INC_FTL_SAMPLING_H_
#define INC_FTL_SAMPLING_H_
#include "stm32f3xx_hal.h"
#define SAMPLE_POINT 1500

#define FTL_SAMPLE_STATE_BUSY 0x01
#define FTL_SAMPLE_STATE_FINISH	0x02
typedef struct{
	uint16_t buf[SAMPLE_POINT];
	int cycles;
	int cycle_cnt;
	int data_ptr;
	int data_size;
	volatile uint16_t state;
}FTL_sampling;

void FTL_Sampling_Init(FTL_sampling *obj,int cycle_cnt);
int FTL_Sampling_Input(FTL_sampling *obj,uint16_t data);
void FTL_Sampling_GetRst(FTL_sampling *obj,uint16_t* data,int* data_size);

#endif /* INC_FTL_SAMPLING_H_ */
