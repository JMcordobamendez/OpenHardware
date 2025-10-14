/*
 *This is the header file to develop the control algorithms
 *Author: Jose Miguel Cordoba Mendez
 *Date: July 2023
 */


#ifndef MAIN_CONTROL_H_
#define MAIN_CONTROL_H_


#define MAX_COUNT 10000 //1.25 cm/s of speed -> 45 m/h
#define CURRENT_OFFSET 3227 //3103 -> 2.5 V ; 3227 -> 2.6 V
#define INTEGRAL_SATURATION 40000 //Simulation Data; 14V·(2^12)/10 = 5734

typedef enum {
	RIGHT_WHEEL,
	LEFT_WHEEL
} wheel_t;

typedef struct {
	unsigned int c_r; //value of counter right wheel
	unsigned int c_r_b; //before value of counter right wheel
	unsigned int c_l; //value of counter left wheel
	unsigned int c_l_b; //before value of counter left wheel
} timer_counter_t;

typedef struct {
	int I_r;
	int I_l;
} current_values_t;

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


typedef enum {
	SPIN_FORWARD,
	SPIN_BACKWARD
} spin_t;

#ifndef MAIN_COMMUNICATIONS_H_
typedef struct {
	char r;
	char l;
} direction_t;

typedef struct{
	int speed_ref_r;
	int speed_ref_l;
	int speed_r;
	int speed_l;

} data_shared_t;

#endif

/*
 * function "SpeedCalculation" Calculates the speed of the wheels
 *-/param "timer_count" the value accumulated by the timer
 *-/param "max_count" maximum value where the speed will be 0
 *-/param "spin_direction" value to set the sign of the speed
 *-/return value of the speed in (rad/s)
 */
int SpeedCalculation(unsigned int timer_count, int max_count ,char spin_direction);

/*
 * function "InitializeCounters" Initializes the timer counters
 *-/param "p" Structure with the values accumulated
 *-/param "max_count" maximum value where the speed will be 0
 */
void InitializeCounters(timer_counter_t* p, int max_count);

/*
 * function "ActualizeTimers" This function restart the timer counter storing the las value
 *-/param "p" Structure with the values accumulated
 *-/param "wheel" this parameter tells which wheel is used
 */
void ActualizeTimers(timer_counter_t* p, wheel_t wheel);

/*
 * function "IncrementTimers" This function increments the timer counters
 *-/param "p" Structure with the values accumulated
 */
void IncrementTimers(timer_counter_t* p);

/*
 * function "InitializeCurrent" This function Initializes the current values to 0
 *-/param "p" Structure with the current values
 */
void InitializeCurrent(current_values_t* p);

/*
 * function "Analog2Current" This function converts the ADC values to current values
 *-/param "p" Structure with the current values
 *-/param "adc_r" is the ADC value measured of the right wheel current sensor
 *-/param "adc_l" is the ADC value measured of the left wheel current sensor
 *-/param "adc_offset" is the ADC Value for current = 0
 */
void Analog2Current(current_values_t* p, int adc_r, int adc_l, int adc_offset);

/*
 * function "InitialzeControlParam" This function Initializes the control values
 *-/param "p" Structure with controller parameters
 */
void InitializeControlParam(control_param_t* p);

/*
 * function "IntegralBackwardEuler" This function implement the Euler 2 method with a step of 100us
 *-/param "p" Structure with controller parameters
 *-/param "error" is the error in speed value calculated
 *-/param "zero" is a logic value checking if the reference value is zero or not
 */
void IntegralBackwardEuler(control_param_t* p, int error_r, int error_l, char zero_r, char zero_l);

/*
 * function "VoltageApplication" This function makes the changes to apply the desired voltage in the motors
 *-/param "p" Structure with controller parameters
 */
void VoltageApplication(control_param_t* param);

/*
 * function "VoltageApplication" This function makes the changes to apply the desired voltage in the motors
 *-/param "param" Structure with controller parameters
 *-/param "current" Structure with the current values
 *-/param "data" is a pointer reference to the shared data structure
 */
void StateSpaceController(control_param_t* param, current_values_t* current, data_shared_t* data);

/*
 * function "ControlAction" This function applies in the HAL
 *-/param "p" Structure with controller parameters
 */
void ControlAction(control_param_t* param);

/*
 * function "IntegralWindup" This function avoids Integral Saturation
 *-/param "p" Structure with controller parameters
 *-/param "saturation_limit" Integral saturation limit in absolute value
 */
void IntegralWindup(control_param_t* param, int saturation_limit);



#endif /* MAIN_CONTROL_H_ */
