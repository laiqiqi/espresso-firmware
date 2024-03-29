#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#include "math_ops.h"

/* Timer and PWM */
#define TIM_PWM			htim3				// PWM/ISR timer handle
#define TIM_CH_WH		TIM_CHANNEL_1		// Water heater timer channel
#define TIM_CH_GH		TIM_CHANNEL_2		// Group heater timer channel

/* GPIO */
#define DRIP_VALVE 		GPIOA, GPIO_PIN_9  // Switches flow between tank and group
#define GROUP_VALVE	 	GPIOC, GPIO_PIN_7  // Opens/closes group to drip flow
/* ADC */

#define ADC_CH_MAIN		hadc1
#define ADC_CH_PR		hadc1
#define ADC_CH_TW		hadc2
#define ADC_CH_TH		0
#define ADC_CH_TG		hadc3

/* SPI */
#define SPI1_H			hspi3				// SPI handle
#define SPI1_CS			GPIOB, GPIO_PIN_10	// CS pin

/* CAN */
#define CAN_H			hcan1		// CAN handle
#define CAN_ID			0x0			// CAN ID
#define M_ID			0x1			// Motor CAN ID

/* Other hardware-related constants */
#define PUMP_DISP		0.0509f		// Pump displacement (ml/rad)
#define EFF_HYDRAULIC	.7f			// torque*eff/disp = pressure
#define C1_LEAK			73791.8f	// Linear leak term (ml/s/Pa)
#define C2_LEAK			12094.7f	// Quadratic leak term (ml/s/Pa^2)
#define KT				0.02395f	// Pump motor torque constant


#define R_RTD 			100.0f
#define R_REF_RTD		430.0f
#define RTD_A			3.9083e-3f
#define RTD_B			-5.775e-7f

#define R_NTC			100000.0f
#define R_REF_NTC 		100000.0f
#define NTC_T1 			298.0f
#define NTC_BETA 		3950.0f

#define P_SCALE			333.982174f	// Pa per A/D count

/* Filters */
#define ALPHA_P				1.0f
#define CMD_FILT_ALPHA		.01f
/* Pressure Control */
#define DT					.00072f			// Loop period
#define K_P					.5e-6f			// Pressure control loop gain
#define KD_P				.05*1e-4f
#define KV_P				.002f;			// Speed loop gain, N-m/rad/s
#define TI_P				.045f			// Pressure integrator time constant
#define TI_F				.05f;			// Flow loop integrator time constant
#define W_LEAD				25.0f			// Lead compensator max phase frequency
#define ALPHA_LEAD			10.0f			// Lead compensator pole/zero separation
#define W_MAX				1000.0f			// Max speed
#define LEAK				25.3f			// Rad/s/bar
#define PSI_PER_BAR			14.5038f		// bar to psi
#define PSI_PER_PA			0.000145038f	// psi to pa
#define PA_PER_BAR			100000.0f		// bar to pascals
#define BAR_PER_PA			.00001f			// pascals to bar
#define M3_PER_ML			.000001f		// mL to cubic meter

/* Calculated Pressure Control Constants */
#define PRESSURE_TO_TORQUE	(PUMP_DISP*M3_PER_ML/EFF_HYDRAULIC)	// Torque to pressure conversion with a perfect pump
#define W1_LEAD				(W_LEAD*sqrtf(ALPHA_LEAD))
#define T_LEAD				(1.0f/W_LEAD)
#define A_LEAD				9.752f//(ALPHA_LEAD*T_LEAD/(DT + T_LEAD))
#define B_LEAD				.7523f//(T_LEAD/(DT + T_LEAD))
#define C_LEAD				10.0f//((ALPHA_LEAD*T_LEAD + DT)/(DT + T_LEAD))

/* Temperature Control */
#define CP_W			4.186f		// Water specific heat, J/g*K
#define M_TH_GROUP		20.0f		// Group thermal mass, J/K
#define R_TH_GROUP		2.7f		// Group thermal resitance, C/W
#define T_AMB			25.0f		// Ambient temp, C (will measure later)
#define K_GROUP_OBS		0.0003f		// Group temp observer gain
#define KI_GROUP_OBS	.0001f		// Group temp observer integrator gain
#define GROUP_HIST		.25f		// Temperature hysteresis
#define DITHER_PERIOD	.11f

#define P_MAX_HEATER	1300.0f		// Water max heating power (W)
#define P_MAX_GROUP		100.0f		// Group max heating power (W)
#define T_HEATER_MAX	125.0f		// Max heater temp (C)

#define K_T_GROUP		2.0f		// W/C
#define KD_T_GROUP		3.5f		// W/C/s
#define KI_T_GROUP		0.1f		// 1/s

#define K_T_WATER		25.0f
#define KD_T_WATER		60.0f
#define KI_T_WATER		0.03f
#define K_T_HEATER		50.0f
#define KI_T_HEATER		0.0f
#endif
