#include "stm32f3xx_hal.h"
#ifndef __STEPPER
#define __STEPPER

#define CE32_STEPPER_PEND	0x0001
typedef struct{
	int state;
	int delay;
	int RPM;
	int Pos_delay;
	long long step_total;
	long long step_cnt;
	int current_beat;
	long long  target_step;
	int direction;
	float step_angle;
	float distPerDegree;
	GPIO_TypeDef* M_port[4];
	uint16_t M_pin[4];
}CE32_stepMotor;

typedef struct{
	GPIO_TypeDef *Port_0,*Port_1,*Port_2,*Port_3;
	uint32_t Pin_0,Pin_1,Pin_2,Pin_3;
	float step_angle;
	float distPerDegree;
	int On_Period;//Connection time per step In ms
	int Int_Period;//Delay time between steps
	int Init_Pos;
}CE32_stepMotor_InitStruct;

void CE32_stepMotor_Init(CE32_stepMotor *x, CE32_stepMotor_InitStruct *init);
void Motor_SetSpeed(CE32_stepMotor *x,int RPM);
void Stepping_down_Xstep(CE32_stepMotor *x,long long  steps);
void Stepping_up_Xstep(CE32_stepMotor *x,long long  steps);
void Stepping_down_Degree(CE32_stepMotor *x,uint32_t degree);
void Stepping_up_Degree(CE32_stepMotor *x,uint32_t degree);
void Stepping_down_Distance(CE32_stepMotor *x,uint32_t distance);
void Stepping_up_Distance(CE32_stepMotor *x,uint32_t distance);
void Stepping_To_Step(CE32_stepMotor *x,long long  step);
void Stepping_To_Degree(CE32_stepMotor *x,long long  degree);
void Stepping_To_Distance(CE32_stepMotor *x,long long  distance);
void Stepping_down(CE32_stepMotor *x);
void Stepping_up(CE32_stepMotor *x);
void Stepping(CE32_stepMotor* motor);
void motor_delay(CE32_stepMotor* motor);

#endif
