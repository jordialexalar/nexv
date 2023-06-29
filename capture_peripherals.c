/*
 * capture_peripherals.c
 *
 *  Created on: 6 mar. 2019
 *      Author: josepmaria.fernandez
 */

#include "main.h"
#include "capture_peripherals.h"

void redirectInputClock (TIM_HandleTypeDef *htim, unsigned int output)
{
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};

  HAL_TIM_Base_Stop(htim);

  switch (output)
  {
  case 1:
    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
    sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
    sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sSlaveConfig.TriggerFilter = 0;
    if (HAL_TIM_SlaveConfigSynchronization(htim, &sSlaveConfig) != HAL_OK)
    {
      Error_Handler();
    }
    break;
  case 2:
    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
    sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
    sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sSlaveConfig.TriggerFilter = 0;
    if (HAL_TIM_SlaveConfigSynchronization(htim, &sSlaveConfig) != HAL_OK)
    {
      Error_Handler();
    }
    break;
  default:
    break;
  }

  //Reset the counter:
 // HAL_TIM_S
  htim->Instance->CNT = 0;
  HAL_TIM_Base_Start(htim);
}

uint32_t timerTachoAcquisition = 0;
uint32_t lastCapture1 = 0;
uint32_t lastCapture2 = 0;
//Tachometer manager
void tachometerManager(void)
{
  static unsigned char tachoInput = 1;

  if (timerTachoAcquisition >= 1000)
  {
    timerTachoAcquisition = 0;
    switch(tachoInput)
    {
    case 1:
      lastCapture1 = htim3.Instance->CNT;
      redirectInputClock(&htim3,2);
      tachoInput = 2;
      break;
    case 2:
      lastCapture2 = htim3.Instance->CNT;
      redirectInputClock(&htim3,1);
      tachoInput = 1;
      break;
    default:
      break;
    }
  }
}
