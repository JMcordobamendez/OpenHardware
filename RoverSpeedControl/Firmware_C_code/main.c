/*
 *Main file :)
 *Author: Jose Miguel Cordoba Mendez
 *Date: July 2024
 */

//Given Libraries
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_task_wdt.h"
//Created by me Libraries
#include "communications.h"
#include "control.h"
#include "common_fcn.h"

#define TX_PIN 17
#define RX_PIN 16
#define PINOUT_TIMER_1 GPIO_NUM_26
#define PINOUT_TIMER_2 GPIO_NUM_27
#define PINOUT_PWM_R GPIO_NUM_12
#define PINOUT_PWM_L GPIO_NUM_13
#define TIMER_PERIOD_1 200 //Timer period in us (5 KHz)
#define TIMER_PERIOD_2 200 //Timer period in us

static const uart_port_t uart_num = UART_NUM_2;
static const int uart_buffer_size = 255*sizeof(char);
static QueueHandle_t uart2_queue;

static void Timer_ISR_1(void* params);
static void Timer_ISR_2(void* params);
static void GPIO_ISR_1(void* params);
static void GPIO_ISR_2(void* params);
static void PWMControl(void *pvParameters);
static void ReceiveUART(void *pvParameters);

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
volatile char flag_timer_1 = 0;
volatile char flag_control = 0;
volatile char flag_math = 0;
volatile char flag_timer_2 = 0;

channel_t channel;
timer_counter_t timer_counter;
volatile data_shared_t shared_data;
//Communications Timer Global Variables
volatile msg_status_t msg_status = CHANNEL_FAILURE; //Until communications begins it is a failure
direction_t dir;
volatile data_shared_t local_data;
volatile data_shared_t copy_control_side;

void app_main(void)
{
	//-Initialize Drivers
	InitializeCounters(&timer_counter, MAX_COUNT);

	//--GPIO
	//Output
	gpio_set_direction(PINOUT_TIMER_1, GPIO_MODE_OUTPUT);
	gpio_set_direction(PINOUT_TIMER_2, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_NUM_21, GPIO_MODE_OUTPUT); //Right Wheel 4A
	gpio_set_direction(GPIO_NUM_22, GPIO_MODE_OUTPUT); //Right Wheel 3A
	gpio_set_direction(GPIO_NUM_32, GPIO_MODE_OUTPUT); //Left Wheel 2A
	gpio_set_direction(GPIO_NUM_33, GPIO_MODE_OUTPUT); //Left Wheel 1A
	//Input
	gpio_set_direction(GPIO_NUM_4, GPIO_MODE_INPUT);
	gpio_set_direction(GPIO_NUM_2, GPIO_MODE_INPUT);
	//--Timer
	/* Configuration of the Timer inside of the PWM block (4 fast timers and 4 slow timers) */
	ledc_timer_config_t pwm_timer = {
	    .speed_mode       = LEDC_HIGH_SPEED_MODE,
	    .timer_num        = LEDC_TIMER_0,
	    .duty_resolution  = LEDC_TIMER_10_BIT,
	    .freq_hz          = 30000,
	    .clk_cfg          = LEDC_AUTO_CLK
	};
	ledc_timer_config(&pwm_timer);

	 /* The Timer is going to be initialized inside the task to control the core where it returns
	  * after the ISR */
	//--UART
	uart_config_t uart_config = {
	    .baud_rate = 115200,
	    .data_bits = UART_DATA_8_BITS,
	    .parity = UART_PARITY_DISABLE,
	    .stop_bits = UART_STOP_BITS_1,
	    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB,
	    //.rx_flow_ctrl_thresh = 122,
	};
	// Install UART driver using an event queue here
	uart_driver_install(uart_num, uart_buffer_size,  uart_buffer_size, 20, &uart2_queue, 0);
	// Configure UART parameters
    uart_param_config(uart_num, &uart_config);
	// Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
	uart_set_pin(uart_num, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	//--PWM (ledc pwm module)
	ledc_channel_config_t pwm_channel_r = {
	    .speed_mode     = LEDC_HIGH_SPEED_MODE,
	    .channel        = LEDC_CHANNEL_0,
	    .timer_sel      = LEDC_TIMER_0,
	    .intr_type      = LEDC_INTR_DISABLE,
	    .gpio_num       = PINOUT_PWM_R,
	    .duty           = 0,
	    .hpoint         = 0
	};
	ledc_channel_config_t pwm_channel_l = {
		    .speed_mode     = LEDC_HIGH_SPEED_MODE,
		    .channel        = LEDC_CHANNEL_1,
		    .timer_sel      = LEDC_TIMER_0, //Same timer so both pwm are synchronized
		    .intr_type      = LEDC_INTR_DISABLE,
		    .gpio_num       = PINOUT_PWM_L,
		    .duty           = 0,
		    .hpoint         = 0
		};
	ledc_channel_config(&pwm_channel_r);
	ledc_channel_config(&pwm_channel_l);
	ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
	ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 0);
	ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
	ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
	//--ADC
	adc1_config_width(ADC_WIDTH_12Bit);
	adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_MAX);
	adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_MAX);
	//-Initialize Interrupts
	//--UART Interrupt (State Machine Controller uses a DMA so it is not necessary)

	//-Initialize shared data between processes
	InitSharedData(&shared_data);

	//-Creation of 2 processes with core affinity, CORE_1 -> Control Algorithm, CORE_0 -> Communications
	//TASK FUNCTION | TASK NAME | MEMORY ALLOCATED | SHARED POINTER | PRIORITY | TASK HANDLER | CORE AFFINITY
	xTaskCreatePinnedToCore(PWMControl, "PWMControl", 4096, NULL, 13, NULL, CORE_1);
	xTaskCreatePinnedToCore(ReceiveUART, "ReceiveUART", 4096, NULL, 12, NULL, CORE_0);

}

//-Timer ISR
//Control Timer
void IRAM_ATTR Timer_ISR_1(void* params)
{
	flag_control = 1;
	IncrementTimers(&timer_counter);
}
//Communication Timer
void IRAM_ATTR Timer_ISR_2(void* params)
{
	//Timer debugging
	RealTimeDebugCheck(&flag_timer_2, PINOUT_TIMER_2);
	/*
	IncrementTimers(&timer_counter);
    //Use this CPU to do intensive math calculations apart of the communications
	//Do multiplications
	SpeedReferenceCalculation(&local_data,&channel,&dir, msg_status); //12 bits shift left
	//Do divisions
	local_data.speed_r = SpeedCalculation(timer_counter.c_r_b, MAX_COUNT, dir.r); //12 bits shift left
	local_data.speed_l = SpeedCalculation(timer_counter.c_l_b, MAX_COUNT, dir.l);
	//Inter-process Communication
	//taskENTER_CRITICAL(&spinlock);
	taskENTER_CRITICAL_ISR(&spinlock);
	//TestLocalDataFailure(&local_data, msg_status); //Test the failure
	SharedLoad(&shared_data, &local_data); //Shared Data <- Local Copy
	taskENTER_CRITICAL_ISR(&spinlock);
	//taskEXIT_CRITICAL(&spinlock);
	 *
	 */

}

void IRAM_ATTR GPIO_ISR_1(void* params)
{
	//RIGHT WHEEL
	ActualizeTimers(&timer_counter, RIGHT_WHEEL);
}

void IRAM_ATTR GPIO_ISR_2(void* params)
{
	//LEFT WHEEL
	ActualizeTimers(&timer_counter, LEFT_WHEEL);
}

//-Tasks Functions
//-PWM Control
static void PWMControl(void *pvParameters)
{
	esp_task_wdt_delete(NULL);
	//Timer configuration for this core
	const esp_timer_create_args_t timer_args = {
		      .callback = &Timer_ISR_1,
		      .name = "Timer CORE_1"
		 };
	esp_timer_handle_t timer_handler;
	esp_timer_create(&timer_args, &timer_handler);
	esp_timer_start_periodic(timer_handler, TIMER_PERIOD_1);
	//Local Variables (Stack Memory)
	int ch_6; //Left Motor Current Sensor
	int ch_7; //RIGHT Motor Current Sensor
	current_values_t current;
	//data_shared_t copy_control_side;
	control_param_t control_param;
	InitializeCurrent(&current);
	InitializeControlParam(&control_param);
	//GPIO configuration for this core
	//--GPIO Interrupt
	gpio_set_intr_type(GPIO_NUM_4, GPIO_INTR_POSEDGE);
	gpio_set_intr_type(GPIO_NUM_2, GPIO_INTR_POSEDGE);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(GPIO_NUM_4, GPIO_ISR_1, NULL); //RIGHT WHEEL
	gpio_isr_handler_add(GPIO_NUM_2, GPIO_ISR_2, NULL); //LEFT WHEEL


    while (true) {
    	// Polling waiting for timer ISR
    	if (flag_control){
    		flag_control = 0;
    		RealTimeDebugCheck(&flag_timer_1, PINOUT_TIMER_1);

    		//Task communication (Shared information is received)
    		taskENTER_CRITICAL(&spinlock);
    		//TestLocalDataFailure(&copy_control_side); //Test the failure
    		SharedLoad(&copy_control_side, &shared_data); //Local Copy <- Shared Data
    		taskEXIT_CRITICAL(&spinlock);

    		//ADC read (12bits range [0 3.3] V)
    		ch_6 = adc1_get_raw(ADC1_CHANNEL_6);
    		ch_7 = adc1_get_raw(ADC1_CHANNEL_7);
    		//Speed Calculation
    		//Do divisions (non-deterministic math operation)
    		copy_control_side.speed_r = SpeedCalculation(timer_counter.c_r_b, MAX_COUNT, dir.r); //12
    		copy_control_side.speed_l = SpeedCalculation(timer_counter.c_l_b, MAX_COUNT, dir.l);
    		//Current Calculation
    		Analog2Current(&current, ch_7, ch_6, CURRENT_OFFSET);
    		//Control algorithm application
    		StateSpaceController(&control_param, &current, &copy_control_side);
    		//Actuation
    		ControlAction(&control_param);
    	}
    }
}

//-UART receive
static void ReceiveUART(void *pvParameters)
{
	esp_task_wdt_delete(NULL);
	InitChannel(&channel, DEFAULT_VALUE, CHANNEL_NUMBER);
	InitSharedData(&local_data);
	//Timer configuration for this core
		const esp_timer_create_args_t timer_args = {
			      .callback = &Timer_ISR_2,
			      .name = "Timer CORE_0"
			 };
		esp_timer_handle_t timer_handler;
		esp_timer_create(&timer_args, &timer_handler);
		esp_timer_start_periodic(timer_handler, TIMER_PERIOD_2);
	//GPIO configuration for this core
	//--GPIO Interrupt
	/*
	gpio_set_intr_type(GPIO_NUM_4, GPIO_INTR_POSEDGE);
	gpio_set_intr_type(GPIO_NUM_2, GPIO_INTR_POSEDGE);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(GPIO_NUM_4, GPIO_ISR_1, NULL); //RIGHT WHEEL
	gpio_isr_handler_add(GPIO_NUM_2, GPIO_ISR_2, NULL); //LEFT WHEEL
	*/
	//Local variables
	//msg_status_t msg_status = CHANNEL_FAILURE; //Until communications begins it is a failure
	//direction_t dir;
	//data_shared_t local_data;

	//InitChannel(&channel, DEFAULT_VALUE, CHANNEL_NUMBER);
	//InitSharedData(&local_data);
	uart_event_t event;
	//size_t buffered_size;
	uint8_t* dtmp = (uint8_t*) malloc(uart_buffer_size);
	while (1){
		//Waiting for UART event.
		if(xQueueReceive(uart2_queue, (void * )&event, (TickType_t)MAX_TIME)){ //Polling wait, 2 conditions
			bzero(dtmp, uart_buffer_size);
			switch(event.type){
			case UART_DATA:
				uart_read_bytes(uart_num, dtmp, event.size, portMAX_DELAY);
				msg_status = ActualizeChannel(&channel, &dtmp[0], CHANNEL_NUMBER);
				uart_flush(uart_num);

				break;
			default:
				break;

			}
		}
		//Do multiplications
		SpeedReferenceCalculation(&local_data,&channel,&dir, msg_status); //12 bits shift left
		//Inter-process Communication
		taskENTER_CRITICAL(&spinlock);
		//TestLocalDataFailure(&local_data, msg_status); //Test the failure
		SharedLoad(&shared_data, &local_data); //Shared Data <- Local Copy
		taskEXIT_CRITICAL(&spinlock);
	}
}
