/*
 * espresso.c
 *
 *  Created on: Feb 28, 2021
 *      Author: ben
 */


#include "espresso.h"
#include "structs.h"
#include "usart.h"

void update_commands(PFTCStruct *pftc, USBDataStruct *data){
	pftc->pump_cmd = data->in_floats[0];
	pftc->t_water_des = data->in_floats[1];
	pftc->t_group_des = data->in_floats[2];
	pftc->pump_cmd_type = (int)data->in_floats[3];
	pftc->flow_dir = (int)data->in_floats[4];
	pftc->tare = (int)data->in_floats[5];

	//pftc->pressure_des = pump_cmd*PA_PER_BAR;
}


void analog_sample(PFTCStruct *pftc){
	/* Reads ADC sensors */
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	int adc1_raw = HAL_ADC_GetValue(&hadc1);
	int adc2_raw = HAL_ADC_GetValue(&hadc2);
	pftc->adc_p_raw = HAL_ADC_GetValue(&hadc3);

	float v_adc1 = (float)adc1_raw*3.3f/4095.0f;
	float v_adc2 = (float)adc2_raw*3.3f/4095.0f;

	float r1 = (R_REF_NTC*v_adc1/5.0f)/(1 - v_adc1/5.0f);
	float r2 = (R_REF_NTC*v_adc2/5.0f)/(1 - v_adc2/5.0f);

	pftc->t_heater = .98f*pftc->t_heater + .02f*calc_ntc_temp(r1, R_NTC, NTC_T1, NTC_BETA);
	pftc->t_group = .98f*pftc->t_group + .02f*calc_ntc_temp(r2, R_NTC, NTC_T1, NTC_BETA);
	pftc->pressure = (float)(pftc->adc_p_raw - pftc->adc_p_offset)*P_SCALE;
	pftc->pressure_filt = (1.0f-ALPHA_P)*pftc->pressure_filt + ALPHA_P*pftc->pressure;
}

void spi_sample(PFTCStruct *pftc){
	/* Reads SPI RTD */
	pftc->rtd_spi_tx_buff[0] = 0x80;	// register
	pftc->rtd_spi_tx_buff[1] = 0xD2;	//

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET ); 	// CS low
	HAL_SPI_TransmitReceive(&hspi3, &pftc->rtd_spi_tx_buff[0], &pftc->rtd_spi_rx_buff[0], 1, 100);
	while( hspi3.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
	HAL_SPI_TransmitReceive(&hspi3, &pftc->rtd_spi_tx_buff[1], &pftc->rtd_spi_rx_buff[1], 1, 100);
	while( hspi3.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET ); 	// CS high

	pftc->rtd_spi_tx_buff[0] = 0x1;
	pftc->rtd_spi_tx_buff[1] = 0x0;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET ); 	// CS low
	HAL_SPI_TransmitReceive(&hspi3, &pftc->rtd_spi_tx_buff[0], &pftc->rtd_spi_rx_buff[0], 1, 100);
	while( hspi3.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
	HAL_SPI_TransmitReceive(&hspi3, &pftc->rtd_spi_tx_buff[1], &pftc->rtd_spi_rx_buff[0], 1, 100);
	while( hspi3.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
	HAL_SPI_TransmitReceive(&hspi3, &pftc->rtd_spi_tx_buff[1], &pftc->rtd_spi_rx_buff[1], 1, 100);
	while( hspi3.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET ); 	// CS high

	uint16_t spi_word = pftc->rtd_spi_rx_buff[0]<<7 | pftc->rtd_spi_rx_buff[1]>>1;

	float resistance = R_REF_RTD*(float)spi_word/32767.0f;
	pftc->t_water = calc_rtd_temp(resistance, R_RTD, R_REF_RTD, RTD_A, RTD_B);
}

void can_sample(PFTCStruct *pftc){
	/* Reads CAN sensors */
	uint32_t TxMailbox;
	pack_cmd(&can_tx, 0.0f, pftc->vel_des_pump, 0.0f, pftc->k_vel_pump, pftc->torque_des_pump[0]);	// Pack commands
	HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send command
	HAL_CAN_GetRxMessage(&CAN_H, CAN_RX_FIFO0, &can_rx.rx_header, can_rx.data);	// Read response
	unpack_reply(can_rx, pftc->motor_state);	// Unpack commands
}

void pump_enable(void){
	uint32_t TxMailbox;
	for(int i = 0; i<7; i++){can_tx.data[i] = 0xFF;}
	can_tx.data[7] = 0xFC;
	HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send command
}

void pump_disable(void){
	uint32_t TxMailbox;
	for(int i = 0; i<7; i++){can_tx.data[i] = 0xFF;}
	can_tx.data[7] = 0xFD;
	HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send command
}

void zero_sensors(PFTCStruct *pftc){
	/* Measure  ADC offset */
	int adc_offset = 0;
	int n = 1000;

	for (int i = 0; i<n; i++){               // Average n samples
		analog_sample(pftc);
		adc_offset +=  pftc->adc_p_raw;
	 }
	pftc->adc_p_offset = adc_offset/n;

}
void pump_control(PFTCStruct *pftc){
	/* run the appropriate type of pump control based on pump mode */
	switch(pftc->pump_cmd_type){
		case 0:
			pftc->torque_des_pump[0] = 0.0f;
			pftc->pressure_des = 0.0f;
			pftc->k_vel_pump = 0.0f;
			pftc->vel_des_pump = 0.0f;
			pftc->pressure_error_int = 0;
			pftc->flow_error_int = 0;
			break;
		case 1:
			if(pftc->last_cmd_type != pftc->pump_cmd_type){
				pftc->pressure_des = pftc->pressure_filt;
			}
			pftc->pressure_des = (1.0f-CMD_FILT_ALPHA)*pftc->pressure_des + CMD_FILT_ALPHA*pftc->pump_cmd*PA_PER_BAR;
			pressure_control(pftc);
			pftc->flow_error_int = 0;
			break;
		case 2:
			pftc->flow_des = pftc->pump_cmd;
			flow_control(pftc);
			pftc->pressure_error_int = 0;
			break;
		case 3:
			pftc->k_vel_pump = .0005f;
			pftc->vel_des_pump = pftc->pump_cmd;
			break;
		case 4:
			pftc->k_vel_pump = 0.0f;
			pftc->torque_des_pump[0] = pftc->pump_cmd;

	}
	pftc->last_cmd_type = pftc->pump_cmd_type;
}

void pressure_control(PFTCStruct *pftc){
	/* Shift values from last sample */
	pftc->torque_des_pump[1] = pftc->torque_des_pump[0];
	pftc->pressure_error[1] = pftc->pressure_error[0];
	pftc->lead[1] = pftc->lead[0];

	/* Feed-forward torque */
	float t_ff = pftc->pressure_des * PRESSURE_TO_TORQUE;
	/* PI + Lead */

	pftc->k_vel_pump = .0005f;
	pftc->vel_des_pump = .99f*pftc->vel_des_pump;	// Decay commanded velocity to smooth things out
	pftc->pressure_error[0] = pftc->pressure_des - pftc->pressure_filt;
	pftc->d_pressure_error = (pftc->pressure_error[0] - pftc->pressure_error[1]);
	pftc->d_pressure_error_filt = .9f*pftc->d_pressure_error_filt + .1f*pftc->d_pressure_error;
	pftc->lead[0] = C_LEAD*pftc->pressure_error[0] - A_LEAD*pftc->pressure_error[1] + B_LEAD*pftc->lead[1];
	pftc->torque_des_pump[0] = K_P*pftc->pressure_error[0] + pftc->pressure_error_int + 0*KD_P*pftc->d_pressure_error_filt + K_P*pftc->lead[0] + t_ff;
	pftc->pressure_error_int += DT*K_P*pftc->pressure_error[0]/TI_P;

	if(pftc->flag){
		printf("%.1f %.1f %.4f %.6f\r\n", pftc->pressure, pftc->pressure_des, pftc->torque_des_pump[0], pftc->lead[0]*K_P);
	}
}

void update_temp_est(PFTCStruct *pftc){
	float p_in = pftc->p_group_heater - (pftc->t_group_est - T_AMB)/R_TH_GROUP;
	pftc->t_group_est += DT*p_in/M_TH_GROUP;
	pftc->t_group_sensor_est = .99985f*pftc->t_group_sensor_est + .00015f*pftc->t_group_est;
	float error = pftc->t_group - pftc->t_group_sensor_est;
	pftc->t_group_est += K_GROUP_OBS*error;
}

void update_flow_est(PFTCStruct *pftc){
	float flow_noleak = pftc->vel_pump*PUMP_DISP;
	/* Solve pressure = C1_LEAK*flow + C2_LEAK*flow^2  for flow*/
	pftc->leak_flow = (-C1_LEAK + sqrtf(C1_LEAK*C1_LEAK + 4.0f*C2_LEAK*pftc->pressure_filt))/(2.0f*C2_LEAK);
	pftc->flow_est = flow_noleak - pftc->leak_flow;
	pftc->flow_est_filt = .9f*pftc->flow_est_filt + .1f*pftc->flow_est;
	pftc->weight += pftc->flow_est_filt*DT;
	if(pftc->tare){pftc->weight = 0.0f;}
}

void flow_control(PFTCStruct *pftc){
	pftc->torque_des_pump[0] = 0.0f;
	float flow_error = pftc->flow_des - pftc->flow_est;
	pftc->k_vel_pump = .007f;
	pftc->vel_des_pump = (pftc->flow_des + pftc->leak_flow)/PUMP_DISP + pftc->flow_error_int;
	pftc->flow_error_int += DT*flow_error/TI_F;
}

void group_temp_control(PFTCStruct *pftc){

	float error = pftc->t_group_des - pftc->t_group;
	float p_ff = (pftc->t_group_des - T_AMB)/R_TH_GROUP;
	pftc->d_t_group_error = .98f*pftc->d_t_group_error + .02f*(error - pftc->t_group_error)/DT;
	pftc->t_group_error = error;
	//pftc->p_group_heater = P_MAX_GROUP*(error>0) - P_MAX_GROUP*(error<0);
	pftc->group_cmd = K_T_GROUP*error + 0.0f*p_ff;
	pftc->group_cmd += KD_T_GROUP*pftc->d_t_group_error;
	pftc->group_cmd += pftc->t_group_int;

	if(fabsf(error)<12.0f){pftc->t_group_int += K_T_GROUP*KI_T_GROUP*DT*error;}
	else{pftc->t_group_int = 0.0f;}
	pftc->t_group_int = fminf(fmaxf(-.5f*P_MAX_GROUP, pftc->t_group_int), .5f*P_MAX_GROUP);


	pftc->p_group_heater = fminf(fmaxf(0.0, pftc->group_cmd), P_MAX_GROUP);
	float dtc = DITHER_PERIOD*pftc->p_group_heater/P_MAX_GROUP;
	dtc = fminf(fmaxf(0.0f, dtc), 1.0f);
	int pwm_state = dtc>pftc->dither_counter;
	//float error = pftc->t_group_des - pftc->t_group_est;
	//if(fabsf(error) > GROUP_HIST){
	//		pftc->group_cmd = P_MAX_GROUP*(error>0);//50.0f*P_MAX_GROUP *error;// + pftc->t_group_int;
		//pftc->t_group_int += K_T_GROUP*KI_T_GROUP*DT*error;
		//pftc->t_group_int = fminf(fmaxf(-P_MAX_GROUP, pftc->t_group_int), P_MAX_GROUP);
		//pftc->p_group_heater = 50.0f;



	__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_GH, ((TIM_PWM.Instance->ARR))*pwm_state);
	//}

}

void heater_temp_control(PFTCStruct *pftc){

}

void water_temp_control(PFTCStruct *pftc){
	if(pftc->flow_des = 0.0f){		// If no water is flowing, just control the heater temp
		heater_temp_control(pftc);
		return;
	}

	//pftc->t_heater_des = pftc->t_water_des + 15.0f + pftc->t_water_int;
	pftc->t_inlet = 60.0f;

	float p_ff = (pftc->t_water_des - pftc->t_inlet)*pftc->flow_est_filt*CP_W;
	//float error_h = pftc->t_heater_des - pftc->t_heater;
	float error_w = pftc->t_water_des - pftc->t_water;
	pftc->d_t_water_error = .97f*pftc->d_t_water_error + .03f*(error_w - pftc->t_water_error)/DT;
	pftc->t_water_error = error_w;

	pftc->t_heater_des = pftc->t_water_des;
	float error_h = pftc->t_heater_des - pftc->t_heater;
	pftc->d_t_heater_error = .97f*pftc->d_t_heater_error + .03f*(error_h - pftc->t_heater_error)/DT;
	pftc->t_heater_error = error_h;


	pftc->heater_cmd = K_T_WATER*error_w + 0.0f*p_ff;
	pftc->heater_cmd += KD_T_WATER*pftc->d_t_heater_error;//d_t_water_error;
	pftc->heater_cmd += pftc->t_water_int;
	pftc->heater_cmd += K_T_HEATER*(T_HEATER_MAX - pftc->t_heater)*(pftc->t_heater > T_HEATER_MAX);

	pftc->t_water_int += K_T_WATER*KI_T_WATER*DT*error_w;
	pftc->t_water_int = fminf(fmaxf(-.0f*P_MAX_HEATER, pftc->t_water_int), .4f*P_MAX_HEATER);
	//pftc->t_heater_int += KI_T_HEATER*DT*error_w;
	//pftc->t_heater_int = fminf(fmaxf(-.5f*P_MAX_HEATER, pftc->t_heater_int), .5f*P_MAX_HEATER);

	pftc->p_water_heater = fminf(fmaxf(0.0, pftc->heater_cmd), P_MAX_HEATER);
	float dtc = DITHER_PERIOD*pftc->p_water_heater/P_MAX_HEATER;
	dtc = fminf(fmaxf(0.0f, dtc), 1.0f);
	int pwm_state = dtc>pftc->dither_counter;
	__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_WH, ((TIM_PWM.Instance->ARR))*pwm_state);
}

void set_valves(PFTCStruct *pftc){
	int group_valve = 0;
	int drip_valve = 0;
	int steam_valve = 0;
	int water_valve = 0;
	switch(pftc->flow_dir){
	case 0:			// Tank
		break;
	case 1:			// Group
		group_valve = 1;
		break;
	case 2:			// Drip tray
		group_valve = 1;
		drip_valve = 1;
		break;
	case 3:			// Spout
		break;
	case 4:			// Steam
		break;
	}
	HAL_GPIO_WritePin(GROUP_VALVE, group_valve);
	HAL_GPIO_WritePin(DRIP_VALVE, drip_valve);
	//HAL_GPIO_WritPin(STEAM_VALVE, steam_valve);
	//HAL_GPIO_WritePin(WATER_VALVE, water_valve);
}

float calc_ntc_temp(float r, float r_nom, float t1, float beta){
	/* Calculate ntc temperature from resistance change */
	return beta*t1/(-t1*logf(r_nom/r) + beta) - 273.0f;
}

float calc_rtd_temp(float r, float r_nom, float r_ref, float a, float b){
	/* calculate platinum rtd temperature from resistance change */
	float Z1 = -a;
	float Z2 = a * a - (4.0f * b);
	float Z3 = (4.0f * b) / r_nom;
	float Z4 = 2.0f * b;
	float temp = Z2 + (Z3 * r);
	temp = (sqrtf(temp) + Z1) / Z4;
	return temp;
}

void fake_data(PFTCStruct *pftc){
	/* Fakes sensor data for testing without hardware */

}
