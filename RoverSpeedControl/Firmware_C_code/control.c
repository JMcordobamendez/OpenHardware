/*
 *This is the source file to develop control algorithms
 *Author: Jose Miguel Cordoba Mendez
 *Date: July 2023
 */
#include "driver/ledc.h"
#include "control.h"
#include "communications.h"
#include "common_fcn.h"


int SpeedCalculation(unsigned int timer_count, int max_count, char spin_direction)
{
	if (timer_count > max_count) //timer_count will overflow after 120 h
	{
		return 0;
	}
	//speed factor = 10000(200us/s)*1/20 (rev)*2*3.141592 (rad/rev) = 1571 (rad·200us/s) approx
	//12 bit shift left (2^(12))*1571 = 25133 << 8 = 6434816
	int speed = (6434816/timer_count); //(1571 (rad·200us/s) / timer ticks(200us)) << 12
	if(spin_direction)
	{
		return -speed;
	}else{
		return speed;
	}
}

void InitializeCounters(timer_counter_t* p, int max_count)
{
	p->c_l = max_count;
	p->c_l_b = max_count;
	p->c_r = max_count;
	p->c_r_b = max_count;
}

void ActualizeTimers(timer_counter_t* p, wheel_t wheel)
{
	if (wheel)
	{
		//LEFT_WHEEL
		p->c_l_b = p->c_l;
		p->c_l = 1;
	}else{
		//RIGHT_WHEEL
		p->c_r_b = p->c_r;
		p->c_r = 1;
	}
}

void IncrementTimers(timer_counter_t* p)
{
	p->c_l++;
	p->c_r++;
}

void InitializeCurrent(current_values_t* p)
{
	p->I_l = 0;
	p->I_r = 0;
}

void Analog2Current(current_values_t* p, int adc_r, int adc_l, int adc_offset)
{
	//The measurement range is [-12.18; 3.98] V
	//(Value_adc - offset)*(1/254.71 *A/ADC)*(2^12) = 16.67 ~= 17 A (12 bits left shifted) (2.2% error)
	p->I_r = 17*(adc_r - adc_offset);
	p->I_l = 17*(adc_l - adc_offset);
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

void IntegralBackwardEuler(control_param_t* p, int error_r, int error_l, char zero_r, char zero_l)
{

	//If the speed reference is 0 reset the integrator value
	if (zero_r)
	{
		p->x0_r = 0;
	} else {
		//y(k) = y(k-1) + DT*f(k)
		//DT = 200 us = 0.0002 = 0.0002*2^20 >> 20 ~= 210 >> 20 (0.13 % error)
		//p->x0_r = ((p->x1_r << 5) + (210*error_r >> 15)) >> 5; //With this the precision
		p->x0_r = SignedLeftShift(p->x1_r, 5) + SignedRightShift(210*error_r, 15);
		p->x0_r = SignedRightShift(p->x0_r, 5);
	}
	if (zero_l)
	{
		p->x0_l = 0;
	} else {
		//y(k) = y(k-1) + DT*f(k)
		//DT = 200 us = 0.0002 = 0.0002*2^20 >> 20 ~= 210 >> 20 (0.13 % error)
		//p->x0_l = ((p->x1_l << 5) + (210*error_l >> 15)) >> 5; //Increases for low speeds
		p->x0_l = SignedLeftShift(p->x1_l, 5) + SignedRightShift(210*error_l, 15);
		p->x0_l = SignedRightShift(p->x0_l, 5);
	}
	//Actualize the last value
	p->x1_r = p->x0_r;
	p->x1_l = p->x0_l;
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

void VoltageApplication(control_param_t* param)
{
	if(param->V_r >= 0)
	{
		param->d_r = SPIN_FORWARD;
	}else{
		param->V_r = -(param->V_r);
		param->d_r = SPIN_BACKWARD;
	}
	if(param->V_l >= 0)
	{
		param->d_l = SPIN_FORWARD;
	}else{
		param->V_l = -(param->V_r);
		param->d_l = SPIN_BACKWARD;
	}
	if (param->V_r > 1023) //Saturation
	{
		param->V_r = 1023;
	}
	if (param->V_l > 1023) //Saturation
	{
		param->V_l = 1023;
	}
}

void StateSpaceController(control_param_t* param, current_values_t* current, data_shared_t* data)
{
	char zero_r, zero_l;
	//Check the speed reference, for very low speed impose 0 V output
	if (data->speed_ref_r == 0){
		zero_r = 1;
	} else {
		zero_r = 0;
	}
	if (data->speed_ref_l == 0){
		zero_l = 1;
	} else {
		zero_l = 0;
	}
	//Calculates Speed error
	int error_r = (data->speed_ref_r - data->speed_r);
	int error_l = (data->speed_ref_l - data->speed_l);
	//Apply antiwindup saturation action
	error_r = error_r - 10*param->windup_r;
	error_l = error_l - 10*param->windup_l;
	//Integrates the error (Servo System Strategy "Ogata" -> error ~= 0 in steady state)
	IntegralBackwardEuler(param, error_r, error_l, zero_r, zero_l);
	//Windup Action
	IntegralWindup(param, INTEGRAL_SATURATION);
	//param->x0w_r = param->x0_r;
	//param->x0w_l = param->x0_l;
	//Summation of the State Space Controller terms before control action
	//Summation from Integral part
	int sum_r = - SignedRightShift(param->KI*param->x0w_r, 11); //Control parameter was 11 bits left shifted
	int sum_l = - SignedRightShift(param->KI*param->x0w_l, 11);
	//Summation from x2 State (Wheel Speed) feedback
	sum_r += - SignedRightShift(param->K_speed*data->speed_r, 11); //Control parameter was 11 bits left shifted
	sum_l += - SignedRightShift(param->K_speed*data->speed_l, 11);
	//Summation from x1 State (Motor Current) feedback
	sum_r += - SignedRightShift(param->K_current*current->I_r, 11); //Control parameter was 11 bits left shifted
	sum_l += - SignedRightShift(param->K_current*current->I_l, 11);
	//Transforming control action from voltage to PWM (1023/14 ~= 73 -> 0.9% error)
	//Force the output voltage to be set at 0 V (if the reference speed is 0)
	if (zero_r)
	{
		param->V_r = 0;
	}else{
		//Compensating the 12 left bits originally shifted (in current and speed)
		param->V_r = SignedRightShift(73*sum_r, 12);
	}
	if (zero_l)
	{
		param->V_l = 0;
	}else{
		//Compensating the 12 left bits originally shifted (in current and speed)
		param->V_l = SignedRightShift(73*sum_l, 12);
	}

	//OPEN LOOP
	/*
	param->V_r = data->speed_ref_r;
	param->V_l = data->speed_ref_l;
	*/
	//It makes the changes to apply the desired voltage in the motors
	VoltageApplication(param);
}


void ControlAction(control_param_t* param)
{
	/* Hardware Vehicle Control
	 *  _________________________
	 * |--------LEFT WHEEL-------|
	 * |FORWARD  | 1A->1 | 2A->0 |
	 * |-------------------------|
	 * |BACKWARD | 1A->0 | 2A->1 |
	 * |-------RIGHT WHEEL-------|
	 * |FORWARD  | 4A->1 | 3A->0 |
	 * |-------------------------|
	 * |BACKWARD | 4A->0 | 3A->1 |
	 * |_________________________|
	 */
	//Set Power
	ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, (unsigned int)param->V_r);
	ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, (unsigned int)param->V_l);
	ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
	ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
	//Set Direction
	if (param->d_r)
	{//RIGHT BACKWARD
		gpio_set_level(GPIO_NUM_21, 0); //4A
		gpio_set_level(GPIO_NUM_22, 1); //3A
	}else{//RIGHT FORWARD
		gpio_set_level(GPIO_NUM_21, 1); //4A
		gpio_set_level(GPIO_NUM_22, 0); //3A
	}
	if (param->d_l)
	{//LEFT BACKWARD
		gpio_set_level(GPIO_NUM_33, 0); //1A
		gpio_set_level(GPIO_NUM_32, 1); //2A
	}else{//LEFT FORWARD
		gpio_set_level(GPIO_NUM_33, 1); //1A
		gpio_set_level(GPIO_NUM_32, 0); //2A
	}
}
