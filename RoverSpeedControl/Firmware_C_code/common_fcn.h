/*
 *This header file creates common functions for the other heather files
 *Author: Jose Miguel Cordoba Mendez
 *Date: September 2024
 */

#ifndef MAIN_COMMON_FCN_H_
#define MAIN_COMMON_FCN_H_


/*
 * function "SignedRightShift" does a right shift of the data without
 * producing an over/under flow when the value is negative
 *-/param "signed_data" data where the right shift will be applied
 *-/param "positions_shift" the number of right shift movements to be applied
 */
int SignedRightShift(int signed_data, char positions_shift);

/*
 * function "SignedLeftShift" does a left shift of the data without
 * producing an over/under flow when the value is negative
 *-/param "signed_data" data where the left shift will be applied
 *-/param "positions_shift" the number of left shift movements to be applied
 */
int SignedLeftShift(int signed_data, char positions_shift);

/*
 * function "RealTimeDebugCheck" toggles the GPIO to check if Real Time
 * is working with the oscilloscope
 *-/param "rt_flag" variable to check the toggle
 *-/param "rt_gpio_num" GPIO number where the RT must be tested
 */
void RealTimeDebugCheck(volatile char* rt_flag, char rt_gpio_num);

#endif /* MAIN_COMMON_FCN_H_ */
