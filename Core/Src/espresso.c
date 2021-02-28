/*
 * espresso.c
 *
 *  Created on: Feb 28, 2021
 *      Author: ben
 */


#include "espresso.h"
#include "structs.h"

void analog_sample(){
	/* Reads ADC sensors */

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

void can_sample(){
	/* Reads CAN sensors */

}

float calc_ntc_temp(float r, float r_nom, float t1, float beta){
	/* Calculate ntc temperature from resistance change */
}

float calc_rtd_temp(float r, float r_nom, float r_ref, float a, float b){
	/* calculate platinum rtd temperature from resistance change */
}
