/*
 *This is the header file to develop the communications
 *Author: Jose Miguel Cordoba Mendez
 *Date: July 2023
 */

#ifndef MAIN_COMMUNICATIONS_H_
#define MAIN_COMMUNICATIONS_H_

#define CORE_1 1
#define CORE_0 0
#define DEFAULT_VALUE 1000
#define CHANNEL_NUMBER 6
#define CHANNEL_OK 0
#define CHANNEL_FAILURE 1
#define MAX_TIME 1

typedef struct {
	unsigned int CH[CHANNEL_NUMBER];
} channel_t;


typedef char msg_status_t;

typedef enum {
	COM_CHANNEL_1 = 0,
	COM_CHANNEL_2,
	COM_CHANNEL_3,
	COM_CHANNEL_4,
	COM_CHANNEL_5,
	COM_CHANNEL_6,
} communication_channel_t;

#ifndef MAIN_CONTROL_H_

typedef struct {
	char r;
	char l;
} direction_t;

typedef enum {
	SPIN_FORWARD_COM,
	SPIN_BACKWARD_COM
} spin_com_t;

typedef struct{
	int speed_ref_r;
	int speed_ref_l;
	int speed_r;
	int speed_l;

} data_shared_t;

#endif


/*
 * function "InitChannel" initialize the values of the channels
 *-/param "p" is a pointer reference to the channel structure
 *-/param "default_value" is the initial value given to all the channels
 *-/param "n_channels" is the the number of channels available
 */
void InitChannel(channel_t* p, int default_value, int n_channels);

/*
 * function "ActualizeChannel" actualizes the values of the channels with the UART buffer
 *-/param "p" is a pointer reference to the channel structure
 *-/param "temp_buffer" is a pointer reference to the UART buffer
 *-/param "n_channels" is the the number of channels available in the UART buffer
 *-/return the function returns if the communication has worked or not
 */
msg_status_t ActualizeChannel(channel_t* p, unsigned char* temp_buffer, int n_channels);

/*
 * function "SelectChannel" returns the value of one of the channels available
 *-/param "p" is a pointer reference to the channel structure
 *-/param "n_channels" is the the number of channels available in the UART buffer
 *-/param "channel_chosen" is the channel which value is going to be given
 *-/return the function returns the value stored in the channel selected
 */
unsigned int SelectChannel(channel_t* p, int n_channels, communication_channel_t channel_chosen);

/*
 * function "InitSharedData" initialize the values to 0
 *-/param "p" is a pointer reference to the shared data structure
 */
void InitSharedData(volatile data_shared_t* p);

/*
 * function "SpeedReferenceCalculation" calculates the speed reference in each wheel
 *-/param "p" is a pointer reference to the shared data structure
 *-/param "q" is a pointer reference to the channel structure
 *-/param "dir" is a pointer reference to the spin direction of the wheel structure
 *-/param "msg" checks if the communication was working properly
 *-/return the function returns the value stored in the channel selected
 */
void SpeedReferenceCalculation(volatile data_shared_t* p, channel_t* q, direction_t* dir, msg_status_t msg);

void SharedLoad(volatile data_shared_t* shared_memory, volatile data_shared_t* local_memory);

void TestLocalDataFailure(volatile data_shared_t* local_memory, msg_status_t msg);

#endif /* MAIN_COMMUNICATIONS_H_ */
