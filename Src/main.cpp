
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "main.h"
#include "stm32f4xx_hal.h"

#include "PortSendQueue.hpp"
#include "uartCom.h"

#include "EigenLib/Eigen/Dense"
#include "EigenAddons.h"

#include "Robot.h"
#include "Action.h"

#include "ActionManager.h"

//#define DEBUG_MAIN

using namespace Eigen;

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
Robot r;
ActionManager manager;
bool startInterrupt = false;

int timerCntr = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM10_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/////////////////temporal for testing
void robotInit()
{
	r.addRegJoint(-90, 0, 0);
	r.addRegJoint(90, 0, 0);
	r.addRegJoint(0, 150, 0);
	r.addLocJoint(90, 93, 0);
	r.addLocJoint(-90, 0, 0);
	r.setTCPaLength(90);
			
	r.setThetaDeg(1, 90); 
	r.setThetaDeg(2, -90);
			
	r.addJointServoMinMax(0, 555, 150);
	r.addJointServoMinMax(1, 178, 593);
	r.addJointServoMinMax(1, 545, 154);
	r.addJointServoMinMax(2, 560, 160);
	r.addJointServoMinMax(3, 570, 165);
	r.addJointServoMinMax(4, 160, 570);
	
	r.setJointConstructionMinMax(0, -90, 90);
	r.setJointConstructionMinMax(1, 20, 150);
	r.setJointConstructionMinMax(2, -135, -45);
	r.setJointConstructionMinMax(3, -90, 90);
	r.setJointConstructionMinMax(4, -70, 70);
			
	r.setJointConversionMinMax(0, -90, 90);
	r.setJointConversionMinMax(1, 0, 180);
	r.setJointConversionMinMax(2, -180, 0);
	r.setJointConversionMinMax(3, -90, 90);
	r.setJointConversionMinMax(4, -90, 90);
	
#ifdef DEBUG_MAIN
	pcPort << "Koniec robotInit()\n";
#endif // DEBUG_MAIN

	
}

void managerInit()
{
	manager.setRobotPtr(&r);
	
	manager.start();
}

void setActions()
{
	Eigen::Vector3d v0, v1;
			
	/*v0 << 50, 150, 100;
	v1 << 50, 150, -100;
	
	manager.addStraightLineMovAction(r.getTCPlocation(), v0);
	manager.addStraightLineMovAction(v0, v1);
	
	v0 << 100, 200, 100;
	
	manager.addStraightLineMovAction(v1, v0);
		
	v1 << 200, 100, 0;
	
	manager.addStraightLineMovAction(v0, v1);
	
	v0 << 1, 300, 0;
	
	manager.addStraightLineMovAction(v1, v0);
	
	v1 << 200, 100, 200;
	
	manager.addStraightLineMovAction(v0, v1);
	*/
	
	v0 << 183, 200, 0;
	v1 << 183, 100, 0;
	
	manager.addConstTCPOrientAction(r.getTCPlocation(), v0);
	manager.addConstTCPOrientAction(v0, v1);
	
#ifdef DEBUG_MAIN
	pcPort << "koniec setActions()\n";
#endif // DEBUG_MAIN

}

/////////////////!temporal for testing

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM10) {
		timerCntr++;
		if (timerCntr == 1000)
			timerCntr = 0;
		
		if (flags.isSet(ARDUINO_MOV_FIN) && startInterrupt && (timerCntr % 20) == 0)
		{
			//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			manager.nextStep();
		}
		//pcPort << "Koniec przerwania timera\n";
	}
	
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	portInit();	
	MX_TIM10_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim10);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	int i = 0;
	while (1)
	{
		//if(timerCntr == 999)
			//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);			  
		
		if (flags.isSet(ARDUINO_CONNECTED) && !HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin))
		{
			arduinoPort << 'A';
			
			
			///////////////////////////////////// temporal for testing
			while(!flags.isSet(ARDUINO_MOV_FIN));
				
			
			arduinoPort << "B350\n382\n347\n355\n364\n362\nC";
			
			flags.reset(ARDUINO_MOV_FIN);
			
			flags.set(PC_LOOP);
			
			pcPort << "Poczatek\n";
			
			robotInit();
			setActions();
			managerInit();
			
			startInterrupt = true;
			
			///////////////////////////////////// !temporal for stesting
			
		}
		
		if (manager.isCheckCalculations())
		{
			manager.calculations();
		}
		
		if (flags.isSet(PC_SAVE_RCV_VAL))
		{
			/*MatrixXd m0(2,2), m1(2,2);
			MatrixXd m2(3,4), m3;
			m0 << 1, 2,
				-3, 11;
			m1 << -5, 12,
				10, 13;
			
			m2 << 1, 0, 3, 4,
				0, 0, 0, 0,
				123, 0, 117, 8;
			
			
			pcPort << m0 << "\n" << m1 << "\n" << m2 << "\n";
			
			m3 = m2.transpose();
			
			pcPort << "transpoze:\n" << m3 << "\n";
			
			m3 = pseudoInverse(m2);
			
			pcPort << "pseudoinverse:\n" << m3 << "\n";
			
			//pcPort << m0(0, 0) << "  " << m0(0, 1) << "\n" << m0(1, 0) << "  " << m0(1, 1) << "\n";
			//pcPort << m1(0, 0) << "  " << m1(0, 1) << "\n" << m1(1, 0) << "  " << m1(1, 1) << "\n";
			*/
			pcPort << "Poczatek\n";
			Robot r;
			
			r.addRegJoint(-90,  0, 0);
			r.addRegJoint(90, 0, 0);
			r.addRegJoint(0, 100, 0);
			r.addLocJoint(90, 0, 0);
			r.addLocJoint(-90, 0, 50);
			r.setTCPaLength(20);
			
			r.setThetaDeg(1, 90); 
			r.setThetaDeg(4, -90);

			r.addJointServoMinMax(0, 150, 555);
			r.addJointServoMinMax(1, 593, 178);
			r.addJointServoMinMax(1, 154, 545);
			r.addJointServoMinMax(2, 160, 560);
			r.addJointServoMinMax(3, 165, 570);
			r.addJointServoMinMax(4, 160, 570);
	
			r.setJointConstructionMinMax(0, -90, 90);
			r.setJointConstructionMinMax(1, 20, 150);
			r.setJointConstructionMinMax(2, -45, 45);
			r.setJointConstructionMinMax(3, -90, 90);
			r.setJointConstructionMinMax(4, -160, -20);
			
			r.setJointConversionMinMax(0, -90, 90);
			r.setJointConversionMinMax(1, 0, 180);
			r.setJointConversionMinMax(2, -90, 90);
			r.setJointConversionMinMax(3, -90, 90);
			r.setJointConversionMinMax(4, -180, 0);
			
			//for (int i = 0; i < 6; i++)
				//pcPort << r.getJointZinGlobal(i) << '\n';
			
			pcPort << "Polozenie TCP na poczatku:\n" << r.getTCPlocation();
			
			Eigen::Vector3d v;
			
			v << 120, 50, 50;
			
			StraightLineMovAction a(r.getTCPlocation(), v);
			
			a.calculate(r);
			a.execute();
			
			pcPort << "Polozenie TCP na koncu:\n" << r.getTCPlocation();
			
			pcPort << "Katy:\n";
			
			for (int i = 0; i < r.getDOF(); i++)
				pcPort << r.getJointThetaRad(i)/DEG_TO_RAD << '\n';
			
			pcPort << "Odebrane: " << pcPort.getBuffer()->toInt() << '\n';
			flags.reset(PC_SAVE_RCV_VAL);
			pcPort.getBuffer()->clear();
			//HAL_Delay(1000);
		}
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

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage 
	*/
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 72;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks 
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time 
	*/
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick 
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 19;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 3599;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}



/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : I3_Pin I4_Pin I5_Pin I0_Pin 
	                         I1_Pin I2_Pin */
	GPIO_InitStruct.Pin = I3_Pin | I4_Pin | I5_Pin | I0_Pin 
	                        | I1_Pin | I2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : I6_Pin */
	GPIO_InitStruct.Pin = I6_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(I6_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	 /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
