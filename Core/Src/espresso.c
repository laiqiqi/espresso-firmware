/*
 * espresso.c
 *
 *  Created on: Feb 28, 2021
 *      Author: ben
 */


#include "espresso.h"
#include "structs.h"

void analog_sample(PFTCStruct *ptfc){
	/* Reads ADC sensors */
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	int adc1_raw = HAL_ADC_GetValue(&hadc1);
	int adc2_raw = HAL_ADC_GetValue(&hadc2);
	int adc3_raw = HAL_ADC_GetValue(&hadc3);

	float v_adc1 = (float)adc1_raw*3.3f/4095.0f;
	float v_adc2 = (float)adc2_raw*3.3f/4095.0f;

	float r1 = (R_REF_NTC*v_adc1/5.0f)/(1 - v_adc1/5.0f);
	float r2 = (R_REF_NTC*v_adc2/5.0f)/(1 - v_adc2/5.0f);

	ptfc->t_heater = calc_ntc_temp(r1, R_NTC, NTC_T1, NTC_BETA);
	ptfc->t_group = calc_ntc_temp(r2, R_NTC, NTC_T1, NTC_BETA);

	ptfc->pressure = (float)adc3_raw*P_SCALE;
}

void spi_sample(PFTCStruct *ptfc){
	/* Reads SPI sensors */
	ptfc->rtd_spi_tx_buff[0] = 0x80;	// register
	ptfc->rtd_spi_tx_buff[1] = 0xD2;	//

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET ); 	// CS low
	HAL_SPI_TransmitReceive(&hspi3, &ptfc->rtd_spi_tx_buff[0], &ptfc->rtd_spi_rx_buff[0], 1, 100);
	while( hspi3.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
	HAL_SPI_TransmitReceive(&hspi3, &ptfc->rtd_spi_tx_buff[1], &ptfc->rtd_spi_rx_buff[1], 1, 100);
	while( hspi3.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET ); 	// CS high

	ptfc->rtd_spi_tx_buff[0] = 0x1;
	ptfc->rtd_spi_tx_buff[1] = 0x0;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET ); 	// CS low
	HAL_SPI_TransmitReceive(&hspi3, &ptfc->rtd_spi_tx_buff[0], &ptfc->rtd_spi_rx_buff[0], 1, 100);
	while( hspi3.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
	HAL_SPI_TransmitReceive(&hspi3, &ptfc->rtd_spi_tx_buff[1], &ptfc->rtd_spi_rx_buff[0], 1, 100);
	while( hspi3.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
	HAL_SPI_TransmitReceive(&hspi3, &ptfc->rtd_spi_tx_buff[1], &ptfc->rtd_spi_rx_buff[1], 1, 100);
	while( hspi3.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET ); 	// CS high

	uint16_t spi_word = ptfc->rtd_spi_rx_buff[0]<<7 | ptfc->rtd_spi_rx_buff[1]>>1;

	float resistance = R_REF_RTD*(float)spi_word/32767.0f;
	ptfc->t_water = calc_rtd_temp(resistance, R_RTD, R_REF_RTD, RTD_A, RTD_B);
}

void can_sample(PFTCStruct *pftc){
	/* Reads CAN sensors */
	uint32_t TxMailbox;
	pack_cmd(&can_tx, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f);	// Pack commands
	HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send response
	HAL_CAN_GetRxMessage(&CAN_H, CAN_RX_FIFO0, &can_rx.rx_header, can_rx.data);	// Read CAN
	unpack_reply(can_rx, pftc->motor_state);	// Unpack commands
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
