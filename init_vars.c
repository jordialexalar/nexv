/*
 * init_vars.c
 *
 *  Created on: 8 ene. 2019
 *      Author: josepmaria.fernandez
 */
#include "specific_delta.h"
#include "ModbusSlave.h"
#include "main.h"
#include "ModbusMaster.h"
#include "state_machine_master.h"
#include "i2c_peripherals.h"

unsigned char switchStatus = 0;
unsigned long timerBlinkLed = 0;

unsigned int onPowerSaving = 0;
unsigned long timerLedRS485A;
unsigned long timerLedRS485B;
unsigned long timerLedCANA;
unsigned long timerLedCANB;


void initVars(void)
{
  int i;
  for (i=0;i<MAXIMUM_NUMBER_OF_CONVERTERS;i++) {
      deltaConverter[i] = construct_DeltaData();
#if SIMUL_DELTAS
      deltasSIMUL[i].msgIdSimul = 1;
#endif
  }

  // Init Modbus
  // mb_M = construct_ModbusSlave();
  mb = construct_ModbusSlave();
  mb_master = construct_ModbusMaster();

  //Object dictionary initialization
  objectDictionary[DUMMY0] = 43981;  /* ABCD */
  objectDictionary[MFG_DEVICE_NAME_HIGH] = 2477;  /* 0x9AD */
  objectDictionary[MFG_DEVICE_NAME_LOW] = 0;
  objectDictionary[MFG_HARDWARE_VERSION_HIGH] = 0;
  objectDictionary[MFG_HARDWARE_VERSION_LOW] = 0;
  objectDictionary[MFG_SOFTWARE_VERSION_HIGH] = VERSION_HIGH;
  objectDictionary[MFG_SOFTWARE_VERSION_LOW] = VERSION_LOW;

  objectDictionary[NUM_INSTALLED_DELTAS] = 14;

  objectDictionarySpecialObject4919 = 0;

  /* 1_0..0_4 */
  /* objectDictionarySpecialObject4e1e[1] objectDictionarySpecialObject4e1e[0] */
  /* objectDictionarySpecialObject4e1e[1] = 1<<8;
  objectDictionarySpecialObject4e1e[0] = 4; */

  objectDictionarySpecialObject4e1e[1] = VERSION_HIGH;
  objectDictionarySpecialObject4e1e[0] = VERSION_LOW;
  objectDictionarySpecialObject4e19 = 0x501;
  objectDictionarySpecialObject4930 = 0;
  objectDictionarySpecialObject4931 = TIMER_8M / 1000;
  objectDictionarySpecialObject4932 = 7; // deltas installed

  objectDictionary[INIT_STATE] = 0;
  objectDictionary[SERVICIO] = 0;

  // Init Modbus
  mb = construct_ModbusSlave();
  mb_master = construct_ModbusMaster();

  // Configure Network
  initNetwork();

  // Read data from FRAM
  constructFramRegs();

//  all = 0;outputs.
  globalState = INITIAL_STATE_A1;
}

