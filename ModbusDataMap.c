/*
 * ModbusVarMap.c
 *
 *  Created on: 08/10/2014
 *      Author: bds
 */
#include "ModbusSettings.h"
#include MB_DATA_MAP
#include "ModbusLog.h"
#include "i2c_peripherals.h"
#include "state_machine_master.h"
#include "specific_delta.h"

#if MB_COILS_ENABLED
ModbusCoilsMap construct_ModbusCoilsMap(){
	ModbusCoilsMap coilsMap = {0};
	return coilsMap;
}
#endif

#if MB_INPUTS_ENABLED
ModbusInputsMap construct_ModbusInputsMap(){
	ModbusInputsMap inputsMap = {0};
	return inputsMap;
}
#endif

#if MB_INPUT_REGISTERS_ENABLED
ModbusInputRegistersMap construct_ModbusInputRegistersMap(){
	ModbusInputRegistersMap registersMap = {0};
	return registersMap;
}
#endif

#if MB_HOLDING_REGISTERS_ENABLED
ModbusHoldingRegistersMap construct_ModbusHoldingRegistersMap(){
	ModbusHoldingRegistersMap registersMap = {0};
	return registersMap;
}
#endif

#if MB_INPUT_REGISTERS_ENABLED || MB_HOLDING_REGISTERS_ENABLED
Uint16 objectDictionary[MAXIMUM_OD_OBJECTS];
Uint16 requesterValues000[MAXIMUM_BLOCK_OBJECTS];
Uint16 requesterValues700[MAXIMUM_BLOCK_OBJECTS];
Uint16 objectDictionarySpecialObject4919;
Uint16 objectDictionarySpecialObject4e1e[2];
Uint16 objectDictionarySpecialObject4e19;
Uint16 objectDictionarySpecialObject4930;
Uint16 objectDictionarySpecialObject4931;
Uint16 objectDictionarySpecialObject4932;

/* Update values (when they are nod made by CAN functions)*/
void modbus_UpdateValues(void)
{
  // Detected Deltas
  int i, detectedDeltas = 0;
  for (i = 0; i < objectDictionary[NUM_INSTALLED_DELTAS]; i++)
  {
    if (objectDictionary[DELTA25_CAN_ID_1 + DELTA_ADDRESSES_PER_CONVERTER*i] > 0)
    {
      detectedDeltas++;
    }
  }

  // Cabinet ambient parameters
  objectDictionary[TEMPERATURE_CABINET] = (Uint16)Temperature;
  objectDictionary[HUMIDITY_CABINET] = (Uint16)Humidity;

  //objectDictionary[INIT_STATE] = 0;


  objectDictionary[RELES] = networkParams.node1.socketContactorStatus |
      (networkParams.node2.socketContactorStatus <<1) | ((networkParams.node1.bridgeContactorStatus |
      networkParams.node2.bridgeContactorStatus)<<2);
}

    void modbus_UpdateErrors(void){
        //Temporary connected to test errors
 /*       EventWord dummyError1;
        EventWord dummyError2;

        dummyError1.eventLocation = createEvent(5, 1, 1, INFO, 23);
        dummyError1.epochTime = systemEpoch;
        dummyError1.uniqueIdentifier = 0;

        dummyError2.eventLocation = createEvent(5, 2, 3, DEBUG, 6);
        dummyError2.epochTime = systemEpoch;
        dummyError2.uniqueIdentifier = 0;

        objectDictionary[INDEX] = 2;
        objectDictionary[ERROR_LOCATION_1] = (dummyError1.eventLocation.hardware << 12) | (dummyError1.eventLocation.subsystem << 8) | \
                (dummyError1.eventLocation.subsubsystem << 4) | (dummyError1.eventLocation.severity) ;
        objectDictionary[ERROR_CODE_1] = dummyError1.eventLocation.eventCode;
        objectDictionary[ERROR_EPOCH_HIGH_1] = dummyError1.epochTime >> 16;
        objectDictionary[ERROR_EPOCH_LOW_1] = dummyError1.epochTime & 0xFFFF;
        objectDictionary[ERROR_IDENTIFICATION_1] = dummyError1.uniqueIdentifier;
        objectDictionary[ERROR_LOCATION_2] = (dummyError2.eventLocation.hardware << 12) | (dummyError2.eventLocation.subsystem << 8) | \
                (dummyError2.eventLocation.subsubsystem << 4) | (dummyError2.eventLocation.severity) ;
        objectDictionary[ERROR_CODE_2] = dummyError2.eventLocation.eventCode;
        objectDictionary[ERROR_EPOCH_HIGH_2] = dummyError2.epochTime >> 16;
        objectDictionary[ERROR_EPOCH_LOW_2] = dummyError1.epochTime & 0xFFFF;
        objectDictionary[ERROR_IDENTIFICATION_2] = dummyError2.uniqueIdentifier;

        objectDictionary[NUM_INSTALLED_DELTAS] = 12;
        objectDictionary[NUM_DETECTED_DELTAS] = 12;*/

      int i, convertersInstalled = 0, eventsListed = 0, padding = 0;
      objectDictionary[NUM_INSTALLED_DELTAS] = 14;

      for (i = DELTA25_CAN_ID_1; i < MAXIMUM_OD_OBJECTS; i += DELTA_ADDRESSES_PER_CONVERTER)
      {
        if (objectDictionary[i] > 0) convertersInstalled++;
      }

      objectDictionary[NUM_DETECTED_DELTAS] = convertersInstalled;

      sortEvents(eventBufferM);

      for (i = 0; i < MAXIMUM_ERRORS_LISTED; i++)
      {
        if(eventIsVoid(&eventBufferM[i]) == false)//Means that there is an event
        {
          padding = EVENTS_ADDRESSES_PER_EVENT * eventsListed;
          objectDictionary[ERROR_LOCATION_1 + padding] = (eventBufferM[i].eventLocation.hardware << 12) | (eventBufferM[i].eventLocation.subsystem << 8) | \
                  (eventBufferM[i].eventLocation.subsubsystem << 4) | (eventBufferM[i].eventLocation.severity) ;
          objectDictionary[ERROR_CODE_1 + padding] = eventBufferM[i].eventLocation.eventCode;
          objectDictionary[ERROR_EPOCH_HIGH_1 + padding] = eventBufferM[i].epochTime >> 16;
          objectDictionary[ERROR_EPOCH_LOW_1 + padding] = eventBufferM[i].epochTime & 0xFFFF;
          objectDictionary[ERROR_IDENTIFICATION_1 + padding] = eventBufferM[i].uniqueIdentifier;
          eventsListed++;
        }
        else break;
      }
      objectDictionary[INDEX] = eventsListed;


      objectDictionary[SERVICIO] = service;

      if (((networkParams.node1.stateNumber == 2 && networkParams.node2.stateNumber == 2) || (timerWarmUp > TIMER_3M)) && objectDictionary[INIT_STATE] == 0)
      {
    	  objectDictionary[INIT_STATE] = 1;

  #if	DEBUG
  		env_debug("Nodos en produccion");

  		env_debug("\n");
  #endif
    }
    }
#endif
