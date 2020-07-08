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
#include "LiquidCrystal.h"
#define RED 2
#define YELLOW 1
#define GREEN 0
#define MAXTIME 20
#define MIDTIME 5

// test1\MDK-ARM\test1\test1.hex
uint32_t counter1 = MAXTIME; // init

char light1 = GREEN, light2 = RED, running = 0, stateChanged = 1;
void dec2BCD(uint32_t i){
	uint32_t x1 = i  & 1;
	uint32_t x2 = i  & 2;
	uint32_t x3 = i  & 4;
	uint32_t x4 = i  & 8;
	
	if (x1 > 0) x1 = 1;
	if (x2 > 0) x2 = 1;
	if (x3 > 0) x3 = 1;
	if (x4 > 0) x4 = 1;
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, x4);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, x3);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, x2);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, x1);
	
}

void digit_interupt(void){
	
	uint32_t delay = 4;
	uint32_t tmp1 = counter1;
	uint32_t tmp2 = counter1;
	if (light1 == GREEN) tmp1 = counter1 - MIDTIME;
	else if (light2 == GREEN) tmp2 = counter1 - MIDTIME;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0 , 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 , 1);
	dec2BCD( tmp1 % 10);
	tmp1 /= 10;
	HAL_Delay(delay);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0 , 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_0 , 1);
	dec2BCD( tmp1 % 10);
	HAL_Delay(delay);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0 , 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3 | GPIO_PIN_1 | GPIO_PIN_0 , 1);
	dec2BCD( tmp2 % 10);
	tmp2 /= 10;
	HAL_Delay(delay);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0 , 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_0 , 1);

	dec2BCD( tmp2 % 10);

}

void updateLight1(){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 , 0);
	setCursor(23,0);
	switch(light1){
		case GREEN:	 print("GREEN ");	 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5 , 1);	break;
		case YELLOW: print("YELLOW");	 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4 , 1);	break;
		case RED:		 print("RED   ");	 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3 , 1);	break;
	}
}

void updateLight2(){
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0 , 0);
	setCursor(25,1);
	switch(light2){
		case GREEN: 	print("GREEN ");	 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2 , 1);	break;
		case YELLOW:	print("YELLOW");	 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1 , 1);	break;
		case RED:			print("RED   ");	 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 , 1);	break;
	}
}


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;

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
	if (0==running) {
		running = 1; // Start the lights and counters and everything
		HAL_TIM_Base_Start_IT(&htim3); // Start timers
		HAL_TIM_Base_Start_IT(&htim4);
		stateChanged = 1;
	}
  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles EXTI line4 interrupt.
*/
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */
	
	// EXTI4: RESET LIGHT 1 TO RED
	light1 = RED;
	light2 = GREEN;
	counter1 = MAXTIME;
	stateChanged = 1;
	//updateLight1();
	//updateLight2();
	
  /* USER CODE END EXTI4_IRQn 1 */
}

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
	uint32_t pending = EXTI->PR;
	if (pending & (1 << 5) ){
		EXTI->PR = (1 << 5);
		light1 = YELLOW;
		light2 = RED;
		counter1 = MIDTIME;
	}
	if (pending & (1 << 6) ){
		EXTI->PR = (1 << 6);
		light1 = GREEN;
		light2 = RED;
		counter1 = MAXTIME;
	}
	if (pending & (1 << 7) ){
		EXTI->PR = (1 << 7);
		light1 = RED;
		light2 = GREEN;
		counter1 = MAXTIME;
		
	}
	if (pending & (1 << 8) ){
		EXTI->PR = (1 << 8);
		light1 = RED;
		light2 = YELLOW;
		counter1 = MIDTIME;
		
	}
	if (pending & (1 << 9) ){
		EXTI->PR = (1 << 9);
		light1 = GREEN;
		light2 = RED;
		counter1 = MAXTIME;
		
	}
	
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
	
	
	stateChanged = 1;
	
  /* USER CODE END EXTI9_5_IRQn 1 */
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
	//counter = (counter + 1) % 2000;
	// counter = (counter + 1) % 2000;
	digit_interupt();
	if (stateChanged == 1){
		stateChanged = 0;
		updateLight1();
		updateLight2();
	}
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
	//counter1 = (counter1 + 1) % MAXTIME;
	
	counter1--;
	if (counter1 == 0) counter1 = MAXTIME;
	
	if(counter1 == MIDTIME){
		if (light1 == GREEN) light1 = YELLOW;
		else if (light2 == GREEN) light2 = YELLOW;
		stateChanged = 1;
	} else if (counter1 == MAXTIME){
		light1 = (light1 + 1) % 3;
		light2 = (light2 + 1) % 3;
		stateChanged = 1;
	} else stateChanged = 0;
	
	// refresh if changes happened 
	// It is just for printing the last part of LCD
	
	
	//digit_interupt();
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
  /* USER CODE END TIM4_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
	extern unsigned char data[1];
	
	switch(data[0]){
		case 'r':
			light1 = RED;
			light2 = GREEN;
			counter1 = MAXTIME;
			break;
		case 'g':
			light1 = GREEN;
			light2 = RED;
			counter1 = MAXTIME;
			break;
		case 'y':
			light1 = YELLOW;
			light2 = RED;
			counter1 = MIDTIME;
			break;
		case '1':
			light1 = GREEN;
			light2 = RED;
			counter1 = MAXTIME;
			break;
		case '2':
			light1 = RED;
			light2 = GREEN;
			counter1 = MAXTIME;
			break;
		case '3':
			light1 = RED;
			light2 = YELLOW;
			counter1 = MIDTIME;
			break;
			
	}
	stateChanged = 1;
	
	HAL_UART_Receive_IT(&huart2,data,sizeof(data));
  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

// test1\MDK-ARM\test1\test1.hex

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
