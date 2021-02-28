/*
 * espresso.h
 *
 *  Created on: Feb 28, 2021
 *      Author: ben
 */

#ifndef INC_ESPRESSO_H_
#define INC_ESPRESSO_H_

#include "hw_config.h"
#include "usart.h"
#include "math_ops.h"
#include "spi.h"
#include "adc.h"
#include "can.h"
#include "gpio.h"

typedef struct{
	/* Pressure/Flow/Temp Control */
	/* Pressure/flow variables */
	float pressure, pressure_des, pressure_error;		// Pascals
	float d_pressure, d_pressure_error;					// Pascals/s
	float flow_est, flow_des, flow_error;				// mL
	float d_flow_est;									// mL/s
	float volume;										// mL
	float t_pump;										// N-m
	float v_pump;										// rad/s
	/* Water Temp */
	uint8_t rtd_spi_tx_buff[2];
	uint8_t rtd_spi_rx_buff[2];
	float t_water, t_water_des, t_water_error;			// C
	float d_t_water, d_t_water_error;					// C/s
} PFTCStruct;


void analog_sample();
void spi_sample(PFTCStruct *ptfc);
void can_sample();
float calc_ntc_temp(float r, float r_nom, float t1, float beta);
float calc_rtd_temp(float r, float r_nom, float r_ref, float a, float b);

#endif /* INC_ESPRESSO_H_ */
