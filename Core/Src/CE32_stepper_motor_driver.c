#include "CE32_stepper_motor_driver.h"

void CE32_stepMotor_Init(CE32_stepMotor *x, CE32_stepMotor_InitStruct *init)
{
	x->M_port[0]=init->Port_0;
	x->M_port[1]=init->Port_1;
	x->M_port[2]=init->Port_2;
	x->M_port[3]=init->Port_3;
	x->M_pin[0]=init->Pin_0;
	x->M_pin[1]=init->Pin_1;
	x->M_pin[2]=init->Pin_2;
	x->M_pin[3]=init->Pin_3;
	x->step_angle=init->step_angle;
	x->distPerDegree=init->distPerDegree;
	x->delay=init->On_Period;
	x->Pos_delay=init->Int_Period;
	x->current_beat=0;
	x->step_cnt=0;
}

void Motor_SetSpeed(CE32_stepMotor *x,int RPM)
{
	x->RPM=RPM;
	x->delay=(1000.0/(RPM/60.0*360/x->step_angle))/4; //in ms
}
void Stepping_down_Xstep(CE32_stepMotor *x,long long  steps)
{
	int goal_delay=x->delay;
	int current_delay=CE32_STEPPER_INIT_DELAY;
	for(long long  i=0;i<steps;i++){
		Stepping_down(x);
		if(current_delay>goal_delay)
			current_delay--;
			x->delay=current_delay;
	};
	x->delay=goal_delay;
}

void Stepping_up_Xstep(CE32_stepMotor *x,long long  steps)
{
	int goal_delay=x->delay;
	int current_delay=CE32_STEPPER_INIT_DELAY;
	for(long long  i=0;i<steps;i++){
		Stepping_up(x);
		if(current_delay>goal_delay)
			current_delay--;
			x->delay=current_delay;
	};
	x->delay=goal_delay;
}

void Stepping_down_Degree(CE32_stepMotor *x,uint32_t degree)
{
	int steps=degree/x->step_angle;
	Stepping_down_Xstep(x,steps);
}
void Stepping_up_Degree(CE32_stepMotor *x,uint32_t degree)
{
	int steps=degree/x->step_angle;
	Stepping_up_Xstep(x,steps);
}

void Stepping_down_Distance(CE32_stepMotor *x,uint32_t distance)
{
	int degree=distance/x->distPerDegree;
	Stepping_down_Degree(x,degree);
}
void Stepping_up_Distance(CE32_stepMotor *x,uint32_t distance)
{
	int degree=distance/x->distPerDegree;
	Stepping_up_Degree(x,degree);
}

void Stepping_To_Step(CE32_stepMotor *x,long long  step)
{
	long long  n_step=step-x->step_cnt;
	if(n_step>0)
	{
		Stepping_down_Xstep(x, n_step);
	}
	else
	{
		Stepping_up_Xstep(x, -n_step);
	}
}

void Stepping_To_Degree(CE32_stepMotor *x,long long  degree)
{
	Stepping_To_Step(x,degree/x->step_angle);
}

void Stepping_To_Distance(CE32_stepMotor *x,long long  distance)
{
	Stepping_To_Degree(x,distance/x->distPerDegree);
}

void Stepping_down(CE32_stepMotor* x)
{
	if(x->current_beat==8)
	{
		x->current_beat-=8;
	}
	x->current_beat++;
	x->step_cnt++;

	Stepping(x);
}

void Stepping_up(CE32_stepMotor* x)
{
	if(x->current_beat==0)
	{
		x->current_beat=8;
	}
	x->current_beat--;
	x->step_cnt--;

	Stepping(x);
}

void motor_delay(CE32_stepMotor* x)
{
	//HAL_TIM_Base_Start_IT
	//TIM_MOTOR->PSC=47999;					//set pre-scale
	//TIM_MOTOR->EGR=TIM_EGR_UG; //generate a update event to reload the prescaler value
	//TIM_MOTOR->SR=0; //clear state register
	//TIM_MOTOR->CNT=65536-x->delay;//set count
	//HAL_TIM_Base_Start(htim16);
	//TIM_MOTOR->CR1|=(TIM_CR1_CEN);//ticking timer
	//while((TIM_MOTOR->SR&TIM_SR_UIF)==0){};
	//TIM_MOTOR->CR1&=~(TIM_CR1_CEN);//close timer
	//HAL_Delay(x->delay);
	custom_delay(x->delay);
}
void Stepping(CE32_stepMotor* motor)
{
	switch(motor->current_beat){
	case 0:{
		motor->M_port[0]->BSRR = motor->M_pin[0];
		motor->M_port[1]->BSRR = motor->M_pin[1];
		motor_delay(motor);
		break;
	};
	case 1:{
		motor->M_port[1]->BSRR = motor->M_pin[1];
		motor_delay(motor);
		break;
	};
	case 2:{
		motor->M_port[1]->BSRR = motor->M_pin[1];
		motor->M_port[2]->BSRR = motor->M_pin[2];
		motor_delay(motor);
		break;
	};
	case 3:{
		motor->M_port[2]->BSRR = motor->M_pin[2];
		motor_delay(motor);
		break;
	};
	case 4:{
		motor->M_port[2]->BSRR = motor->M_pin[2];
		motor->M_port[3]->BSRR = motor->M_pin[3];
		motor_delay(motor);
		break;
	};
	case 5:{
		motor->M_port[3]->BSRR = motor->M_pin[3];
		motor_delay(motor);
		break;
	};
	case 6:{
		motor->M_port[0]->BSRR = motor->M_pin[0];
		motor->M_port[3]->BSRR = motor->M_pin[3];
		motor_delay(motor);
		break;
	}
	case 7:{
		motor->M_port[0]->BSRR = motor->M_pin[0];
		motor_delay(motor);
		break;
	};
	}

	motor->M_port[0]->BSRR = (uint32_t)(motor->M_pin[0])<<16U;
	motor->M_port[1]->BSRR = (uint32_t)(motor->M_pin[1])<<16U;
	motor->M_port[2]->BSRR = (uint32_t)(motor->M_pin[2])<<16U;
	motor->M_port[3]->BSRR = (uint32_t)(motor->M_pin[3])<<16U;
	//motor_delay(motor);
}
#define hTIM htim17
extern TIM_HandleTypeDef hTIM;
void custom_delay(int delay)
{
	hTIM.Instance->ARR=delay;
	hTIM.Instance->SR&=~TIM_SR_UIF;
	HAL_TIM_Base_Start(&hTIM);
	while((hTIM.Instance->SR&TIM_SR_UIF)==0);
	HAL_TIM_Base_Stop(&hTIM);
}
