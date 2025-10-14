
#include <stdio.h>

#define INTEGRAL_SATURATION 30000

typedef struct {
	int x0_r; //present value of the integral
	int x0_l;
	int x1_r; //(k-1) value of the integral
	int x1_l;
	int windup_r; //Integrator Saturation Windup
	int windup_l;
	int x0w_r; //present value of the integral after windup
	int x0w_l;
	char d_r; //Spinning direction
	char d_l;
	int V_r; //Voltage in the motor (PWM value)
	int V_l;
	int KI; //State Space Controller
	int K_speed;
	int K_current;
} control_param_t;

void IntegralWindup(control_param_t* param, int saturation_limit);
void InitializeControlParam(control_param_t* p);

int main(void)
{
	control_param_t param_1;
	control_param_t param_2;
	InitializeControlParam(&param_1);
	InitializeControlParam(&param_2);
	param_1.x0_r = 33000;
	param_1.x0_l = 29000;
	param_2.x0_r = -33000;
	param_2.x0_l = -29000;
	
	IntegralWindup(&param_1, INTEGRAL_SATURATION);
	IntegralWindup(&param_2, INTEGRAL_SATURATION);
	
	printf("Original 1: r: %d | l: %d\n", param_1.x0_r, param_1.x0_l);
	printf("Windup_1 Output: r: %d | l: %d\n", param_1.x0w_r, param_1.x0w_l);
	printf("Windup_1 Sat Protection: r: %d | l: %d\n", param_1.windup_r, param_1.windup_l);
	
	printf("Original 2: r: %d | l: %d\n", param_2.x0_r, param_2.x0_l);
	printf("Windup_2 Output: r: %d | l: %d\n", param_2.x0w_r, param_2.x0w_l);
	printf("Windup_2 Sat Protection: r: %d | l: %d\n", param_2.windup_r, param_2.windup_l);
}


void IntegralWindup(control_param_t* param, int saturation_limit)
{
	//Check Saturation Limit
	if ((param->x0_r >= saturation_limit) || (param->x0_r <= -saturation_limit))
	{
		//Check if it is positive or negative
		if (param->x0_r >= saturation_limit)
		{
			param->x0w_r = saturation_limit;
			param->windup_r = (param->x0_r - saturation_limit);
		}else{
			param->x0w_r = -saturation_limit;
			param->windup_r = (param->x0_r + saturation_limit); //(-)·(-) = +
		}

	}else{
		param->x0w_r = param->x0_r;
		param->windup_r = 0;
	}
	//Check Saturation Limit
	if ((param->x0_l >= saturation_limit) || (param->x0_l <= -saturation_limit))
	{
		//Check if it is positive or negative
		if (param->x0_l >= saturation_limit)
		{
			param->x0w_l = saturation_limit;
			param->windup_l = (param->x0_l - saturation_limit);
		}else{
			param->x0w_l = -saturation_limit;
			param->windup_l = (param->x0_l + saturation_limit); //(-)·(-) = +
		}

	}else{
		param->x0w_l = param->x0_l;
		param->windup_l = 0;
	}
}

void InitializeControlParam(control_param_t* p)
{
	p->x0_l = 0;
	p->x0_r = 0;
	p->x1_l = 0;
	p->x1_r = 0;
	p->windup_r = 0;
	p->windup_l = 0;
	p->x0w_r = 0;
	p->x0w_l = 0;
	p->V_r = 0;
	p->V_l = 0;
	p->KI = -6476; // -3.1623 (11 bit left shifted)
	p->K_speed = 1819; // 0.8883 (11 bit left shifted)
	p->K_current = 271; //0.1327 (11 bit left shifted)
}