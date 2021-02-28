/*
 * structs.h
 *
 *  Created on: Mar 5, 2020
 *      Author: Ben
 */

#ifndef STRUCTS_H
#define STRUCTS_H




#include <stdint.h>
#include "spi.h"
#include "gpio.h"
#include "adc.h"
#include "tim.h"
#include "can.h"
#include "espresso.h"


/* Global Structs */
extern CANTxMessage can_tx;
extern CANRxMessage can_rx;
extern PFTCStruct pftc;


#endif
