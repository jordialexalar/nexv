#include "main.h"
#include "ModbusTimer.h"
#include "ModbusLog.h"
#include "ModbusSettings.h"

//struct CPUTIMER_VARS CpuTimer1;

void timer_resetTimer()
{
  htim6.Instance->CNT = 0;
  htim6.Instance->SR = htim6.Instance->SR & 0xFE;
  return;
}


bool timer_expiredTimer(Timer *self)
{
  return (htim6.Instance->SR & 0x01);
}


void timer_setTimerReloadPeriod(Timer *self, Uint32 setTime)
{
	self->stop();
	self->reloadTime = setTime;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 71;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = setTime;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  timer_resetTimer(); // Added afterwards
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
}


void timer_init(Timer *self, Uint32 setTime)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 71;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = setTime;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}


void timer_stop()
{
  HAL_TIM_Base_Stop(&htim6);
}

void timer_start()
{
  HAL_TIM_Base_Start(&htim6);
}

Timer construct_Timer(){
	Timer timer;

	timer.timerEnabled = false;
	timer.reloadTime = 0;

	timer.resetTimer = timer_resetTimer;
	timer.expiredTimer = timer_expiredTimer;
	timer.setTimerReloadPeriod = timer_setTimerReloadPeriod;
	timer.init = timer_init;
	timer.stop = timer_stop;
	timer.start = timer_start;

	return timer;
}

/**** Functions for Master ****/

void timer_resetTimerMaster()
{
  __HAL_TIM_SET_COUNTER(&htim7,0);
  __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
  //htim7.Instance->CNT = 0;
  //htim7.Instance->SR = htim7.Instance->SR & 0xFE;
  return;
}

bool timer_expiredTimerMaster(Timer *self)
{
  bool expiration = 0;
  if __HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE) expiration = 1;
  return (expiration);
}

void timer_setTimerReloadPeriodMaster(Timer *self, Uint32 setTime)
{
  self->stop();
  self->reloadTime = setTime/10;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 719;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = self->reloadTime;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  timer_resetTimerMaster();// Must be here. S ERRATA
}


void timer_initMaster(Timer *self, Uint32 setTime){
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 719;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = setTime/10;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  timer_resetTimerMaster();// Must be here. S ERRATA
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void timer_stopMaster()
{
  HAL_TIM_Base_Stop(&htim7);
}

void timer_startMaster()
{
  HAL_TIM_Base_Start(&htim7);
  timer_resetTimerMaster(); // Must be here. S ERRATA
}

Timer construct_TimerMaster(){
  Timer timer;

  timer.timerEnabled = false;
  timer.reloadTime = 0;

  timer.resetTimer = timer_resetTimerMaster;
  timer.expiredTimer = timer_expiredTimerMaster;
  timer.setTimerReloadPeriod = timer_setTimerReloadPeriodMaster;
  timer.init = timer_initMaster;
  timer.stop = timer_stopMaster;
  timer.start = timer_startMaster;

  return timer;
}


