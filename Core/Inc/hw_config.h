#ifndef HW_CONFIG_H
#define HW_CONFIG_H

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
#define PUMP_DISP		.3f			// Pump displacement (ml/revolution)
#define K_LEAK			.1f			// Pump leak rate (ml/s/Pa)
#define KT				.1f			// Pump motor torque constant


#define R_RTD 			100.0f
#define R_REF_RTD		430.0f
#define RTD_A			3.9083e-3f
#define RTD_B			-5.775e-7f

#define R_NTC			100000.0f
#define R_REF_NTC 		100000.0f
#define NTC_T1 			298.0f
#define NTC_BETA 		3950.0f

#define P_SCALE			333.982174f	// Pa per A/D count

/* Pressure Control */
#define K_P					.001f			// Pressure control proportional gain
#define KV_P				.002f;			// Speed loop gain, N-m/rad/s
#define KI_P				.00005f			// Pressure integral gain
#define W_MAX				1000.0f			// Max speed
#define LEAK				25.3f			// Rad/s/bar
#define PSI_PER_BAR			14.5038f		// bar to psi
#define PA_PER_BAR			100000.0f		// bar to pascals
#define BAR_PER_PA			.00001f			// pascals to bar
#define M3_PER_ML			.000001f		// mL to cubic meter
#define PRESSURE_TO_TORQUE	PUMP_DISP*M3_PER_ML*PA_PER_BAR	// Torque to pressure conversion with a perfect pump

#define P_MAX_HEATER	1300.0f		// Water max heating power (W)
#define P_MAX_GROUP		100.0f		// Group max heating power (W)

#endif
