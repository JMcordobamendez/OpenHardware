/*
 *This is the header creates common functions for the other heather files
 *Author: Jose Miguel Cordoba Mendez
 *Date: September 2024
 */

#include "control.h"
#include "communications.h"
#include "common_fcn.h"
#include "driver/gpio.h"

int SignedRightShift(int signed_data, char positions_shift)
{
	if (signed_data < 0){ //Signed data is negative
		signed_data = -signed_data; //Turn it positive
		signed_data = signed_data >> positions_shift; //Right shift
		signed_data = -signed_data; //Turn it negative again
	} else { //Signed data is positive
		signed_data = signed_data >> positions_shift;
	}
	return signed_data;
}

int SignedLeftShift(int signed_data, char positions_shift)
{
	if (signed_data < 0){ //Signed data is negative
		signed_data = -signed_data; //Turn it positive
		signed_data = signed_data << positions_shift; //Left shift
		signed_data = -signed_data; //Turn it negative again
	} else { //Signed data is positive
		signed_data = signed_data << positions_shift;
	}
	return signed_data;
}

void RealTimeDebugCheck(volatile char* rt_flag, char rt_gpio_num)
{
	if(*rt_flag == 1){
		*rt_flag = 0;
		gpio_set_level(rt_gpio_num, 0);
	} else {
		*rt_flag = 1;
		gpio_set_level(rt_gpio_num, 1);
	}
}
