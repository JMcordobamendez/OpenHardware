/*
 *This is the source file to develop the communications
 *Author: Jose Miguel Cordoba Mendez
 *Date: July 2023
 */
#include "communications.h"
#include "control.h"
#include "common_fcn.h"


void InitChannel(channel_t* p, int default_value, int n_channels){
	int count;
	for(count = 0; count<n_channels; count++)
	{
		p->CH[count] = default_value;
	}
}

msg_status_t ActualizeChannel(channel_t* p, unsigned char* temp_buffer, int n_channels){
	//Check if the UART message stored in buffer has the correct header
	if((*(temp_buffer) == 0x20) & (*(temp_buffer+1) == 0x40))
	{
		int count;
		for(count = 0; count<n_channels; count++)
		{
			//Little Endian ordering
			p->CH[count] = *(temp_buffer + 2*(count+1)); //LSB, pair bits beginning in 2
			p->CH[count] += *(temp_buffer + 2*(count+1)+1) << 8; //MSB, unpair bits beginning in 3
		}
		return CHANNEL_OK;
	} else {
		return CHANNEL_FAILURE;
	}

}

unsigned int SelectChannel(channel_t* p, int n_channels, communication_channel_t channel_chosen){
	if (channel_chosen > n_channels)
	{
		return DEFAULT_VALUE;
	}
	return p->CH[channel_chosen];
}

void InitSharedData(volatile data_shared_t* p){
	p->speed_r = 0;
	p->speed_l = 0;
	p->speed_ref_r = 0;
	p->speed_ref_l = 0;
}

void SpeedReferenceCalculation(volatile data_shared_t* p, channel_t* q, direction_t* dir, msg_status_t msg){

	unsigned int abs_speed;
	int spin_speed;
	if (msg)
	{//CHANNEL_FAILURE
		abs_speed = 0;
		spin_speed = 0;
	}else{//CHANNEL_OK
		//spin_speed_max = 37.5rpm/60·2·pi = 3.927 rad/s = channel_max_value*3.927/1000 *2^(12) = 16
		spin_speed = (SelectChannel(q, CHANNEL_NUMBER, COM_CHANNEL_1)-1500)*16;
		//spin_speed = SignedRightShift(spin_speed, 1);
		//abs_speed_max = 120rpm/60·2·pi = 12.5664 rad/s = channel_max_value*12.5664/1000 *2^(12) = 51
		abs_speed = (SelectChannel(q, CHANNEL_NUMBER, COM_CHANNEL_3)-1000)*51; //12 bits left shifted
	}

	//Wheel speed
	p->speed_ref_l = abs_speed+spin_speed;
	p->speed_ref_r = abs_speed-spin_speed;

	//Dead Zone for joystick neutral point deviations (30/1000 = 3% of full speed)
	//if((p->speed_ref_l <= 30*64)&&(p->speed_ref_l >= -30*64))
	if((p->speed_ref_l <= 10*32)&&(p->speed_ref_l >= -10*32))
	{
		p->speed_ref_l = 0;
		p->speed_ref_r = 0;
	}

	if (p->speed_ref_l >= 0)
	{
		dir->l = SPIN_FORWARD_COM;
	}else{
		dir->l = SPIN_BACKWARD_COM;
	}
	if (p->speed_ref_r >= 0)
	{
		dir->r = SPIN_FORWARD_COM;
	}else{
		dir->r = SPIN_BACKWARD_COM;
	}
}

void SharedLoad(volatile data_shared_t* shared_memory, volatile data_shared_t* local_memory){
	//shared_memory->speed_l = local_memory->speed_l;
	//shared_memory->speed_r = local_memory->speed_r;
	shared_memory->speed_ref_l = local_memory->speed_ref_l;
	shared_memory->speed_ref_r = local_memory->speed_ref_r;
}

void TestLocalDataFailure(volatile data_shared_t* local_memory, msg_status_t msg){
	if (msg){//CHANNEL_FAILURE
		local_memory->speed_ref_l = 0;
		local_memory->speed_ref_r = 0;
	}else{//CHANNEL_OK
		local_memory->speed_ref_l = 1000;
		local_memory->speed_ref_r = 1000;
	}
}
