/*
 * usb_comm.h
 *
 *  Created on: Apr 3, 2021
 *      Author: ben
 */

#ifndef INC_USB_COMM_H_
#define INC_USB_COMM_H_


#include "stm32f4xx_hal.h"

#define N_OUTPUT	20
#define N_INPUT		20


typedef struct{
	union{
		uint8_t out_buff[N_OUTPUT*4];
		float out_floats[N_OUTPUT];
		int out_ints[N_OUTPUT];
	};
	union{
		uint8_t in_buff[N_INPUT*4];
		float in_floats[N_INPUT];
		int in_ints[N_INPUT];
	};

} USBDataStruct;

extern USBDataStruct usb_data;
#endif /* INC_USB_COMM_H_ */
