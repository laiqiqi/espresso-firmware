/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "hw_config.h"
#include "math_ops.h"
#include "espresso.h"
#include "structs.h"
#include "usb_comm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

PFTCStruct pftc;

CANTxMessage can_tx;
CANRxMessage can_rx;
USBDataStruct usb_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* Turn on ADCs */
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start(&hadc3);

  /* Zero Sensors */
  zero_sensors(&pftc);

  /* CAN setup */
  can_rx_init(&can_rx);
  can_tx_init(&can_tx);
  HAL_CAN_Start(&CAN_H); //start CAN
  //__HAL_CAN_ENABLE_IT(&CAN_H, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* Turn on PWM */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);



  /* Enable the pump over CAN */
  HAL_Delay(1000);
  pump_enable();
  HAL_Delay(100);
  /* Start main interrupt */
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int count = 0;
  int delay = 300;
  float off2 = 6.0f*PA_PER_BAR;
  float off = .00f;
  float amp = .5f*PA_PER_BAR;
  int timer = 0;

  //printf("%f  %f  %f\r\n", A_LEAD, B_LEAD, C_LEAD);

  while (1)
  {
	  //HAL_Delay(10);

	  //pack_cmd(&can_tx, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f);	// Pack commands
	  //HAL_CAN_AddTxMessage(&CAN_H, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send response
	  //HAL_CAN_GetRxMessage(&CAN_H, CAN_RX_FIFO0, &can_rx.rx_header, can_rx.data);	// Read CAN
	  //unpack_reply(can_rx, &motor_state);	// Unpack commands
	  //for(int i = 0; i<3; i++){printf("%f  ", motor_state[i]);}
	  //printf("%.2f  %.2f  %.2f  %.2f  %.2f %.2f", pftc.t_water, pftc.t_heater, pftc.t_group, pftc.pressure*PSI_PER_PA, pftc.vel_pump, pftc.torque_pump);
	  //printf("%.2f\r\n", pftc.pressure_des);
	  //pftc.torque_des_pump = off2;
/*
	  pftc.pressure_des = off2;
	  int sw = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
	  if(sw || count != 0){
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET );
		  count++;
		  if(count > delay){
			  float t = (float)timer*.002f;
			  float cmd = (float)sw*(off + amp*sinf(.5f*2.0f*PI_F*t*t)) + off2;
			  //float cmd = (float)sw*6.0f*PA_PER_BAR;
			  //pftc.torque_des_pump = cmd;
			  //pftc.k_vel_pump = .01f;
			  pftc.pressure_des = cmd;
			  count = delay;
			  timer ++;
		  }
		  if(!sw){
			  count -= 2;
		  }

		  //printf("%.2f %.4f %.4f %.2f\r\n", pftc.pressure, pftc.pressure_des, pftc.torque_pump, pftc.vel_pump);
		  pftc.flag = 1;
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET );
	  }
	  else{pftc.flag = 0;}
*/

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIM = 4;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 96;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLSAIP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
