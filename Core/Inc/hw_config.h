#ifndef HW_CONFIG_H
#define HW_CONFIG_H


/* CAN */
#define CAN_H			hcan1		// CAN handle
#define CAN_ID			0x0			// CAN ID
#define M_ID			0x1			// Motor CAN ID

/* Other hardware-related constants */
#define PUMP_DISP		.1f			// Pump displacement (ml/revolution)
#define K_LEAK			.1f			// Pump leak rate (ml/s/Pa)
#define KT				.1f			// Pump motor torque constant




#endif
