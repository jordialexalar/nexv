/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */
#define ANO2000  946684800
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ModbusDataMap.h"
#include "can_power_member.h"
#include "specific_delta.h"
#include "state_machine_nodes.h"
#include "state_machine_master.h"
#include "i2c_peripherals.h"
#include "ModbusSerial.h"
#include "capture_peripherals.h"

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
unsigned long partialEpoch = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */
short counterCAN=0;
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

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (0)
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
  * @brief This function handles CAN1 RX0 interrupt.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
  if (timerLedCANA > COMS_LEDS_TIMER)
  {
    timerLedCANA = 0;
    if (onPowerSaving) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
    else HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_8);
  }
  isr_can_internal();
  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
  //timerUpdateTemp++;
  timerBlinkLed++;
  partialEpoch++;

  //systemEpoch = (((Uint32)objectDictionary[EPOCH_TIME_HIGH])<<16) + (Uint32)objectDictionary[EPOCH_TIME_LOW];

    if (partialEpoch >= 1000)
	{
#if ACTUALIZA_SYSTEMEPOCH

		if ((switchStatus & SW_MASTER_MASK) == SW_MASTER_MASK)
			{
					auxSystemEpoch =
					(((Uint32) objectDictionary[EPOCH_TIME_HIGH]) << 16)
						+ (Uint32) objectDictionary[EPOCH_TIME_LOW];
				if (auxSystemEpoch < ANO2000)
				{
					auxSystemEpoch = ANO2000;
				}

				if ((systemEpoch > auxSystemEpoch + 10)
					|| (systemEpoch < auxSystemEpoch - 10))
				{
					systemEpoch = auxSystemEpoch;

				}
			}
#else
		if ((switchStatus & SW_MASTER_MASK) == SW_MASTER_MASK)
		{
			if (auxSystemEpoch < ANO2000)
			{
				auxSystemEpoch = ANO2000;
				systemEpoch = auxSystemEpoch;
			}

		}

#endif
	partialEpoch = 0;
	systemEpoch++;
	}

  //objectDictionary[EPOCH_TIME_HIGH] = (systemEpoch >> 16) & 0x0000FFFF;
  //objectDictionary[EPOCH_TIME_LOW] = systemEpoch & 0x0000FFFF;
  timerSimul1++;
  timerEnablesGlobal++;
  timerEnablesRelayClosed++;
  timerEnablesRelayOpened++;
  timerConfigurationRelays++;
  timerDecreasePower++;
  timerCanLowLatency1++;
  timerCanLowLatencyDelta++;
  timerCanMediumLatency++;
  timerCanLongLatency++;
  timerConfiguringMaster++;
  timerUpdateTemp++;
  timerTempAcquisition++;
  if (timerWarmUp < MAX_TIME_WARMUP)
	  timerWarmUp++;
  if (timerEstabRele >= 0)
	  timerEstabRele++;

  timerThermalManagementDeltaExec++;
  timerHBMaster++;
  timerHBcan++;
  timerLedRS485A++;
  timerLedRS485B++;
  timerLedCANA++;
  timerLedCANB++;
  timerModbusRequest++;
  // timerModbusSlaveError++;
  // timerModbusMasterError++;
  timerDelayForSocketConf++;
  timerCanMediumLatencyErrorsMaster++;
  timerTachoAcquisition++;
  timerMasterTransmission++;
  timerMasterTransmission2++;
  timerMasterTransmission3++;
  timerBetweenDeltaActivation++;
  timerLedFRAM++;
  timerCurrentCompensation++;
  timerVentilador++;

  if (timerTraza >= 0)
	  timerTraza++;
  if (timerTraza3 >= 0)
	  timerTraza3++;

  timerTraza2++;
  if (timerTransmision >= 0)
	  timerTransmision++;
  timerTraza22++;
#if STANDBY_ENABLE
  if (timerStandby >= 0)
	  timerStandby++;
#endif

  if (timerDebugMeter  >= 0){
	  timerDebugMeter++;
  }

  if (timerReset >= 0)
	  timerReset++;
  timerInEmcy++;

  clockTickAllSM();
  clockTickNetwork();

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  if (timerLedRS485A > COMS_LEDS_TIMER)
  {
    timerLedRS485A = 0;
    if (onPowerSaving) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
    else HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_9);
  }
  //if (huart1.gState == HAL_UART_STATE_BUSY_RX) receiving = 1;

  //msgModbusRx[msgModbusRxBuffIndex] = tinnyRxBuffer[1];
  //msgModbusRxBuffIndex++;
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8) == GPIO_PIN_RESET)
  {
    msgModbusRxSlave[msgModbusRxBuffIndexSlave] = tinnyRxBufferSlave[0];
    msgModbusRxBuffIndexSlave++;
    HAL_UART_Receive_IT(&huart1, (uint8_t *)tinnyRxBufferSlave, 1);//MB_BUFFER_SIZE<<1); // Worst case, reserve all memory
  }
  else
  {

  }
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  if (timerLedRS485B > COMS_LEDS_TIMER)
  {
    timerLedRS485B = 0;
    if (onPowerSaving) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
    else HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
  }
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) == GPIO_PIN_RESET) //Received a word
  {
    msgModbusRxMaster[msgModbusRxBuffIndexMaster] = tinnyRxBufferMaster[0];
    msgModbusRxBuffIndexMaster++;
    HAL_UART_Receive_IT(&huart2, (uint8_t *)tinnyRxBufferMaster, 1);//MB_BUFFER_SIZE<<1); // Worst case, reserve all memory
  }
 /* else
  {
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4, GPIO_PIN_RESET); // Prepare for receive
    HAL_UART_Receive_IT(&huart2, (uint8_t *)tinnyRxBufferMaster, 1);//MB_BUFFER_SIZE<<1); // Worst case, reserve all memory
  }*/
  /* USER CODE END USART2_IRQn 1 */
}


/**
  * @brief This function handles USART2 global interrupt.
  */
void USART3_IRQHandler(void)
{

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART2_IRQn 1 */

}

/**
  * @brief This function handles CAN2 RX0 interrupt.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */
  if (timerLedCANB > COMS_LEDS_TIMER)
  {
    timerLedCANB = 0;
    if (onPowerSaving) HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
    else HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);

  }

  while ((hcan2.Instance->RF0R & 0x03) > 0)
  {
    isr_can_delta();
  }

  counterCAN++;
  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
