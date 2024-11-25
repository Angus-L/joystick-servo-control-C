/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
uint32_t old_time=0;
uint16_t range = 0.03;
int count1 = 0;
int count2 = 0;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) == SET && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==SET)//detect both button push at the same time
		{
				changePulseX(90);
				count1 = 1;
				count2 = 1;
		}



		else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) != RESET && HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) == RESET)//only pressed one button,as when get in this function something is pushed
		{

			count1 = 0;
			count2 = 0;
			if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET)
			{

				{changePulseX(109);}//set the new pulse for duty cycle

				__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
				HAL_GPIO_EXTI_Callback(GPIO_PIN_0);
			}
		}
		else//when it get to this else it means nothing is pushed but still something is detected when release so ignore it
		{
			if((HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == RESET && count1 ==1) || (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) == RESET) && count2==1)//last action pressed two button
					//time - old_time < range && count1 != 0)//set range to ignore hardware keep pushing
			{
				//count1 =0;
				//count2 =0;
				return;
			}
		}
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */


		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == SET && HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) == SET)//both pressed
		{
				changePulseX(90);
				count1 = 1;//only update the count when both is pressed so only will return when the previous move is press at the two same time.
				count2 = 1;
		}



		else if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) != RESET && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == RESET)//only pressed one button
		{
			count1 = 0;
			count2 = 0;
			if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13) != RESET)
			{


			changePulseX(70);//set the new pulse for duty cycle
			}

			__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
			HAL_GPIO_EXTI_Callback(GPIO_PIN_13);
		}
		else//detect something when released, ignore it.
		{	if((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) == RESET && count2 == 1) || (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == RESET)&&count1==1)// || HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) != SET)
					//time-old_time < range && count2 !=0)//set range to ignore hardware keep pushing
			{
				//count1=0;
				//count2=0;
				return;
			}
	}
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
