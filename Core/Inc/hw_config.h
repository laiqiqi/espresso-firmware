#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#include "math_ops.h"

/* Timer and PWM */
#define TIM_PWM			htim3				// PWM/ISR timer handle
#define TIM_CH_U		TIM_CHANNEL_1		// Terminal U timer channel
#define TIM_CH_V		TIM_CHANNEL_2		// Terminal V timer channel
#define TIM_CH_W		TIM_CHANNEL_3		// Terminal W timer channel
#define INVERT_DTC		0					// PWM inverting (1) or non-inverting (0)

/* ISRs */
#define PWM_ISR			TIM1_UP_TIM10_IRQn	// PWM Timer ISR
//#define CAN_ISR			CAN1_RX0_IRQn		// CAN Receive ISR

/* ADC */

#define ADC_CH_MAIN		hadc1
#define ADC_CH_PR		hadc1
#define ADC_CH_TW		hadc2
#define ADC_CH_TH		0
#define ADC_CH_TG		hadc3

/* DRV Gate drive */
#define ENABLE_PIN 		GPIOA, GPIO_PIN_11  // Enable gate drive pin.
#define DRV_SPI			hspi1				// DRV SPI handle
#define DRV_CS			GPIOA, GPIO_PIN_4	// DRV CS pin

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

/* Filtesr */
#define ALPHA_P			1.0f
/* Pressure Control */
#define DT					.00072f			// Loop period
#define K_P					.15e-6f			// Pressure control loop gain
#define KD_P				.05*1e-4f
#define KV_P				.002f;			// Speed loop gain, N-m/rad/s
#define TI_P				.025f			// Pressure integrator time constant
#define W_LEAD				250.0f			// Lead compensator max phase frequency
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

#define P_MAX_HEATER	1300.0f		// Water max heating power (W)
#define P_MAX_GROUP		100.0f		// Group max heating power (W)

#endif
