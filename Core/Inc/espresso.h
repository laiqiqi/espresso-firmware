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
#include "usb_comm.h"

#define TANK_FLOW_MASK	0x00
#define GROUP_FLOW_MASK	0x01
#define DRIP_FLOW_MASK	0x02

typedef struct{
	/* Pressure/Flow/Temp Control */
	/* Pressure/flow variables */
	int adc_p_raw, adc_p_offset;
	float loop_time;
	int flow_dir;		// Flow direction (0 = tank, 1 = group, 2 = drip tray, 3 = water spout
	int pump_cmd_type;	// 0 = disabled, 1 = pressure, 2 = flow, 3 = velocity
	float pump_cmd;
	float pressure, pressure_des, pressure_filt;		// Pascals
	float pressure_error[2];
	float pressure_error_int;
	float lead[2];
	float d_pressure, d_pressure_error, d_pressure_error_filt;	// Pascals/s
	float flow_est, flow_est_filt, flow_des, flow_error, leak_flow;				// mL/s
	float d_flow_est;									// mL/s/s
	float weight;										// g
	int tare;
	union{
		float motor_state[3];
		struct{
			float pos_pump, vel_pump, torque_pump;
		};
	};
	float vel_des_pump;
	float k_vel_pump;
	float torque_des_pump[2];
	/* Water Temp */
	uint8_t rtd_spi_tx_buff[2];
	uint8_t rtd_spi_rx_buff[2];
	float t_water, t_water_des, t_water_error;			// C
	float d_t_water, d_t_water_error;					// C/s
	/* Heater Temp */
	float t_heater, t_heater_des, t_heater_error;		// C
	float d_t_heater, d_t_heater_error;					// C/s
	/* Group Temp */
	float t_group, t_group_des, t_group_error;			// C
	float d_t_group, d_t_group_error;					// C/s
	float p_group_heater, t_group_int;					// W

	int flag;
} PFTCStruct;

void update_commands(PFTCStruct *pftc, USBDataStruct *data);
void analog_sample(PFTCStruct *ptfc);
void spi_sample(PFTCStruct *ptfc);
void can_sample(PFTCStruct *pftc);
void zero_sensors(PFTCStruct *pftc);
void pump_control(PFTCStruct *pftc);
void pressure_control(PFTCStruct *pftc);
void update_flow_est(PFTCStruct *pftc);
void flow_control(PFTCStruct *pftc);
void group_temp_control(PFTCStruct *pftc);
void heater_temp_control(PFTCStruct *pftc);
void water_temp_control(PFTCStruct *pftc);
void set_valves(PFTCStruct *pftc);
float calc_ntc_temp(float r, float r_nom, float t1, float beta);
float calc_rtd_temp(float r, float r_nom, float r_ref, float a, float b);
void pump_enable(void);
void pump_disable(void);

void fake_data(PFTCStruct *pftc);

#endif /* INC_ESPRESSO_H_ */
