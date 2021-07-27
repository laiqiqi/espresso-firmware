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

	pftc->t_heater = calc_ntc_temp(r1, R_NTC, NTC_T1, NTC_BETA);
	pftc->t_group = calc_ntc_temp(r2, R_NTC, NTC_T1, NTC_BETA);
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
			break;
		case 1:
			pftc->pressure_des = pftc->pump_cmd*PA_PER_BAR;
			pressure_control(pftc);
			break;
		case 2:
			pftc->flow_des = pftc->pump_cmd;
			flow_control(pftc);
			break;
		case 3:
			pftc->k_vel_pump = .0005f;
			pftc->vel_des_pump = pftc->pump_cmd;
			break;
		case 4:
			pftc->k_vel_pump = 0.0f;
			pftc->torque_des_pump[0] = pftc->pump_cmd;

	}
}

void pressure_control(PFTCStruct *pftc){
	/* Shift values from last sample */
	pftc->torque_des_pump[1] = pftc->torque_des_pump[0];
	pftc->pressure_error[1] = pftc->pressure_error[0];
	pftc->lead[1] = pftc->lead[0];

	/* Feed-forward torque */
	float t_ff = pftc->pressure_des * PRESSURE_TO_TORQUE;
	/* PI + Lead */
	pftc->k_vel_pump = .00015f;
	pftc->pressure_error[0] = pftc->pressure_des - pftc->pressure_filt;
	pftc->d_pressure_error = (pftc->pressure_error[0] - pftc->pressure_error[1]);
	pftc->d_pressure_error_filt = .9f*pftc->d_pressure_error_filt + .1f*pftc->d_pressure_error;
	pftc->lead[0] = C_LEAD*pftc->pressure_error[0] - A_LEAD*pftc->pressure_error[1] + B_LEAD*pftc->lead[1];
	pftc->torque_des_pump[0] = K_P*pftc->pressure_error[0] + pftc->pressure_error_int + 0*KD_P*pftc->d_pressure_error_filt + K_P*pftc->lead[0] + 0.0f*t_ff;
	pftc->pressure_error_int += DT*K_P*pftc->pressure_error[0]/TI_P;

	if(pftc->flag){
		printf("%.1f %.1f %.4f %.6f\r\n", pftc->pressure, pftc->pressure_des, pftc->torque_des_pump[0], pftc->lead[0]*K_P);
	}
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
	pftc->k_vel_pump = .002f;
	pftc->vel_des_pump = (pftc->flow_des + pftc->leak_flow)/PUMP_DISP;
}

void group_temp_control(PFTCStruct *pftc){
	pftc->t_group_des = pftc->t_water_des;
	float error = pftc->t_group_des - pftc->t_group;
	pftc->d_t_group_error = .9f*pftc->d_t_group_error + .1f*(error - pftc->t_group_error)/DT;
	pftc->t_group_error = error;
	pftc->t_group_int += K_T_GROUP*KI_T_GROUP*DT*error;
	pftc->t_group_int = fminf(fmaxf(-P_MAX_GROUP, pftc->t_group_int), P_MAX_GROUP);
	pftc->p_group_heater = P_MAX_GROUP*(error>0) - P_MAX_GROUP*(error<0);
	pftc->p_group_heater = K_T_GROUP*error;
	pftc->p_group_heater += KD_T_GROUP*pftc->d_t_group_error;
	pftc->p_group_heater += pftc->t_group_int;


	//pftc->p_group_heater = 50.0f;
	float dtc = pftc->p_group_heater/P_MAX_GROUP;
	dtc = fminf(fmaxf(0.0f, dtc), 1.0f);
	__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_GH, ((TIM_PWM.Instance->ARR))*dtc);

}

void heater_temp_control(PFTCStruct *pftc){
	float dtc = .1f;
	__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_WH, ((TIM_PWM.Instance->ARR))*dtc);
}

void water_temp_control(PFTCStruct *pftc){

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
