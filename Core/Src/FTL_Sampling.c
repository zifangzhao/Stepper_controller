/*
 * FTL_Sampling.c
 *
 *  Created on: Aug 13, 2020
 *      Author: aeria
 */


#include "FTL_Sampling.h"

void FTL_Sampling_Init(FTL_sampling *obj,int cycle_cnt)
{
	obj->cycle_cnt=0;
	obj->cycles=cycle_cnt;
	obj->state=0;
	obj->data_size=SAMPLE_POINT;
	obj->data_ptr=0;
	for(int i=0;i<obj->data_size;i++)
	{
		obj->buf[i]=0;
	}
}

int FTL_Sampling_Input(FTL_sampling *obj,uint16_t data)
{
	if((obj->state&FTL_SAMPLE_STATE_FINISH)!=0)
	{
		return -1;
	}
	obj->state=FTL_SAMPLE_STATE_BUSY;
	obj->buf[obj->data_ptr++]=data/obj->cycles;
	if(obj->data_ptr>=obj->data_size)
	{
		obj->data_ptr=0;
		obj->cycle_cnt++;
		if(obj->cycle_cnt>=obj->cycles)
		{
			obj->state=FTL_SAMPLE_STATE_FINISH;
		}
		return 1;
	}
	return 0;
}

void FTL_Sampling_GetRst(FTL_sampling *obj,uint16_t* data,int* data_size)
{
	data=obj->buf;
	data_size=&obj->data_size;
}
