/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */
uint32_t whichDigit = 0;
uint32_t counter = 0;
void dec2BCD(int i){
	int x1 = i & 1;
	int x2 = (i & 2) >> 1;
	int x3 = (i & 4) >> 2;
	int x4 = (i & 8) >> 3;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, x4);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, x3);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, x2);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, x1);
	
}

void digit_interupt(void){
	switch(whichDigit){
		case 0:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
			while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3));
			dec2BCD( (counter & 0x000F) >> 0 );
		break;
		case 1:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
			while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2));
			dec2BCD( (counter & 0x00F0) >> 4 );
		break;
		case 2:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
			while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1));
			dec2BCD( (counter & 0x0F00) >> 8 );
		break;
		case 3:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
			while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0));
			dec2BCD( (counter & 0xF000) >> 12 );
		break;
	}
}


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

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
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
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
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
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
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
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
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
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
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1))
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1,0);
	else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1,1);
		while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)){
			if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0))
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1, 0);
		}
	}
  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	digit_interupt();
	//whichDigit = (whichDigit + 1) % 4;
	whichDigit = 0;
  /* USER CODE END TIM3_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
	counter = (counter + 1) % 2000;
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

// test1\MDK-ARM\test1\test1.hex

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
