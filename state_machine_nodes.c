/*
 * state_machine_nodes.c
 *
 *  Created on: 9 ene. 2019
 *      Author: josepmaria.fernandez
 */
#include "main.h"
#include "can_power_member.h"
#include "specific_delta.h"
#include "state_machine_nodes.h"
#include "state_machine_master.h"
#include "state_machine_commons.h"
#include "i2c_peripherals.h"
#include "ModbusSlave.h"
#include "ModbusMaster.h"
#include "EventList.h"
#include "capture_peripherals.h"

// Global defines
#define ALLOW_RELAY_MONITORING 0 /* Double check this option before programming*/

#define  CAUDAL_DELTA_POR_RPM     0.016  // (169.9 / 10500.0)
#define  PARAMETRO_AJUSTE         0.00071 * 1.20 // (1 /  (2*700)) * (1.20 Valor segun pruebas realizadas en laboratorio)

#define MIN_OPTIMAL_VALUE         38.0

extern int timer_traza;
extern void ftoa(float n, char *res, int afterpoint);
char aux1[20];

unsigned int pol,pol_ant = 0;
int deltasReady(void);
// Public variable declaration:
Uint32 timerTraza3 = -1;
Uint32 timerSimul1 = 0;
Uint32 timerEnablesGlobal = 0;
Uint32 timerConfigurationRelays = 0;
Uint32 timerEnablesRelayClosed = 0;
Uint32 timerEnablesRelayOpened = 0;
Uint32 timerDecreasePower = 0;
Uint32 timerCanLowLatency1 = 0;
Uint32 timerCanLowLatencyDelta = 0;
Uint32 timerCanMediumLatency = 0;
Uint32 timerCanLongLatency = 0;
Uint32 timerUpdateTemp = TIMER_5S;
Uint32 timerThermalManagementDeltaExec = 0;
Uint32 timerHBcan = 0;
Uint32 timerDelayForSocketConf = 0;
Uint32 timerBetweenDeltaActivation = 0;
Uint32 timerCurrentCompensation = 0;
Uint32 timerInEmcy = 0;

Uint32 timerModbusRequest = 0;
Uint32 timerModbusSlaveError = 0;
Uint32 timerModbusMasterError = 0;
// Private variable declaration:
MachineStatesGemma globalState              = INITIAL_STATE_A1;
MachineStatesGemma globalState_ant          = STANDBY_A2;

char *globalState_str[]={"INITIAL_STATE_A1","PREPARATION_F2","PRODUCTION_F1","REQUESTED_STOP_A3","EMERGENCY_STOP_D1","REPARE_FAILURE_D2","STANDBY","ERROR_COMM","WAIT_STANDBY","SERVICE"};

ConfigureOutputContactors outputContactorsConfiguration              = CONF_OPEN_EVERYTHING;
ConfigureOutputContactors outputContactorsConfiguration_ant          = CONF_RELAYS_FATAL_ERROR;

char *outputContactorsConfiguration_str[]={"CONF_OPEN_EVERYTHING1","CONF_WAIT_ALL_OPEN","CONF_CLOSE_DESIDERED","CONF_WAIT_DESIDERED","CONF_PROCESS_FINISHED","CONF_RELAYS_FATAL_ERROR"};

int oldoutput  = -1;

int envia_siempre = 0;
int offsetPosition = 0;
int cargando;

EventWord DummyEvent;
Uint16 socketContactorOutput = 0;
Uint16 bridgeContactorOutput = 0;
Uint16 acRelayOutput = 0;
float decreaseCurrent = 0.0;
union inputUnion inputs;
union outputUnion outputs;

ModbusSlave mb;
ModbusMaster mb_master;





unsigned char tempVar = 0;
unsigned char firstPass = 1;

Uint32 timerEstabRele = 0;
extern int deltaErrorComm;
extern int deltaLastReceivedMessage;

extern unsigned int    discoveredModules;
int ventilador_act = 0;

int jx = 0;
/* This state machine is based on the Gemma guide */
void stateMachineNodes(void)
{
#if  SIMUL_DELTAS
	if (timerWarmUp > TIMER_10S){
		   can_delta_SIMUL(jx);
		   jx++;
		   if (jx == maxNumberOfConvertersWorking)
			   jx = 0;
	}

#endif


  //First, check inputs
  //===================
  readInputsNodes();
  if (supervisor_node == 0)
    mb.loopStates(&mb); //Modbus state machine (also considered as an input)

  //Second, check alarms
  //====================
  if (timerWarmUp > TIMER_3S) checkAlarmInput();

  updateEvents(eventBuffer);
  checkOtherEvents(eventBuffer);

  //Third, state machine evolution
  //==============================
  updateStateMachineNodes();

  //Fourth, update outputs
  //======================
  updateOutputsNodes();
  manageTransmissionsNode();
#if DEBUG_SHOWEVENT
  if (own_node == 5){
	  if (timerTraza3 == -1)
		  timerTraza3 = 0;

	  if (timerTraza3 > TIMER_5S)
	      {
		  /*
	      int dat = 0;
	      dat  = (int) Temperature;

		  env_debug("Temperatura:");
		  env_debug_int(dat);

	      */
	      /*
	      ftoa(Temperature,aux,1);
	      env_debug(aux);
		  env_debug("\n");
	      	timerTraza = 0;
	      	*/
	      timerTraza3 = 0;

	      showEvent(eventBuffer);


	      }
  }

#endif
}



void checkAlarmInput(void)
{
	/*
#define SURGE_PROTECTION  0
#define CLICKSON  1
#define TAMPER  2
#define HEATER_AND_FANS 3
#define VCC_FAULT 4
#define DISPENSER_EMCY  5
	*/
	int input_alarms = 0x0;
	int input_alarms_ant = 0x0;

	/*
  if (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14) == GPIO_PIN_RESET){
    createEvent(own_node, SUB_SYSTEM_CABINET, SUB_SUB_SYSTEM_EMCY, SEVERITY_ERROR, DISPENSER_EMCY);
    input_alarms = input_alarms | 0x10;
  }
*/
#if MODELO_PU350


/*
	  if (MAGNETO_DELTAS){
	    createEvent(own_node, SUB_SYSTEM_CABINET, SUB_SUB_SYSTEM_EMCY, SEVERITY_ERROR, MAGN_DELTAS);
	    input_alarms = input_alarms | 0x20;
	  }*/

	  if (MAGNETO_VENTILADORES){
	    createEvent(own_node, SUB_SYSTEM_CABINET, SUB_SUB_SYSTEM_EMCY, SEVERITY_ERROR, FANS1);
	    input_alarms = input_alarms | 0x40;
	  }


	 if (MAGNETO_CONTROL){
	    createEvent(own_node, SUB_SYSTEM_CABINET, SUB_SUB_SYSTEM_EMCY, SEVERITY_ERROR, CRITICAL_ALARMS);
	    input_alarms = input_alarms | 0x80;
	  }


	if (DISPENSER_EMERGENCY_ON){
	  createEvent(own_node, SUB_SYSTEM_CABINET, SUB_SUB_SYSTEM_EMCY, SEVERITY_ERROR, DISPENSER_EMCY);
	 input_alarms = input_alarms | 0x01;
	}


	if (POWER_EMERGENCY_ON){
	  createEvent(own_node, SUB_SYSTEM_CABINET, SUB_SUB_SYSTEM_EMCY, SEVERITY_ERROR, DISPENSER_POWER_EMCY);
	  input_alarms = input_alarms | 0x02;
	}

/*
	if (MAGNETO_SPD){
	createEvent(own_node, SUB_SYSTEM_CABINET, SUB_SUB_SYSTEM_EMCY, SEVERITY_ERROR, MAGN_SPD);
	input_alarms = input_alarms | 0x04;
	}
*/

	if (MAGNETO_AC){
	  createEvent(own_node, SUB_SYSTEM_CABINET, SUB_SUB_SYSTEM_EMCY, SEVERITY_ERROR, MAGN_AC);
	  input_alarms = input_alarms | 0x08;
	}
/*
	if (MAGNETO_SAI){
	  createEvent(own_node, SUB_SYSTEM_CABINET, SUB_SUB_SYSTEM_EMCY, SEVERITY_ERROR, MAGN_SAI);
	  input_alarms = input_alarms | 0x08;
	}
*/
#endif
#if MODELO_NEXVIA
	  if (FANS2_EMERGENCY_ON){
	    createEvent(own_node, SUB_SYSTEM_CABINET, SUB_SUB_SYSTEM_EMCY, SEVERITY_ERROR, FANS1);
	    input_alarms = input_alarms | 0x20;
	  }

	  if (FANS1_EMERGENCY_ON){
	    createEvent(own_node, SUB_SYSTEM_CABINET, SUB_SUB_SYSTEM_EMCY, SEVERITY_ERROR, FANS2);
	    input_alarms = input_alarms | 0x40;
	  }


	 if (ALARM5_ON){
	    createEvent(own_node, SUB_SYSTEM_CABINET, SUB_SUB_SYSTEM_EMCY, SEVERITY_ERROR, CRITICAL_ALARMS);
	    input_alarms = input_alarms | 0x80;
	  }


  if (ALARM4_ON){
    createEvent(own_node, SUB_SYSTEM_CABINET, SUB_SUB_SYSTEM_EMCY, SEVERITY_ERROR, NO_CRITICAL_ALARMS);
   input_alarms = input_alarms | 0x01;
  }


  if (DISPENSER_EMERGENCY_ON){
    createEvent(own_node, SUB_SYSTEM_CABINET, SUB_SUB_SYSTEM_EMCY, SEVERITY_ERROR, DISPENSER_EMCY);
    input_alarms = input_alarms | 0x02;
  }

  /*
   *  if (ALARM2_ON){
      createEvent(own_node, SUB_SYSTEM_CABINET, SUB_SUB_SYSTEM_EMCY, SEVERITY_ERROR, DELTAS);
      input_alarms = input_alarms | 0x04;
    }
  */

  if (ALARM1_ON){
    createEvent(own_node, SUB_SYSTEM_CABINET, SUB_SUB_SYSTEM_EMCY, SEVERITY_ERROR, FANS1);
    input_alarms = input_alarms | 0x08;
  }


#endif



/*

#if	DEBUG
  if (input_alarms_ant != input_alarms){
	  input_alarms_ant = input_alarms;

		sprintf(bufferstr, "Alarm error: %i \n", (int) input_alarms);
		      env_debug(bufferstr);
		env_debug("\n");
  }
#endif
*/
  int networkOnFault = 0;
  int i = 0;
    for (i = 0; i < MAXIMUM_ERRORS_LISTED; i++)
    {
      // if ((eventBufferM[i].eventLocation.hardware != 0  || && eventBufferM[i].eventLocation.severity == SEVERITY_ERROR) ||
	  if ((eventBuffer[i].eventLocation.severity == SEVERITY_ERROR) ||
        eventBuffer[i].eventLocation.severity == SEVERITY_FATAL)
        {
          networkOnFault = 1;
        }
      if (networkOnFault) nodesHaveErrorsFlag = 1;
      else nodesHaveErrorsFlag = 0;
    }
}


void readInputsNodes(void)
{
  // Variable declarations
  float  currentB = 0.0;
  NodeParams *thisNodeParams;

  thisNodeParams = findNodeById(own_node); //Load parameters of this node, will depend on the node number


  if (thisNodeParams == &voidNode) //Be sure that the node is valid. If not, stop the power block and generate an event to inform
      createEvent(own_node, SUB_SYSTEM_CAN_COMMUNICATIONS, SUB_SUB_SYSTEM_GENERIC, SEVERITY_FATAL, NOT_MAPPED_NODE);
  // Read hardware inputs (input pins)
  inputs.bit.in1 = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_7);
  inputs.bit.in2 = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_8);
  inputs.bit.in3 = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9);
  inputs.bit.in4 = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10);
  inputs.bit.in5 = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11);
  inputs.bit.in6 = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12);
  inputs.bit.in7 = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13);
  inputs.bit.in8 = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14);
  //todo The asignation is left for the future
  thisNodeParams -> socketContactorStatus = inputs.bit.in3 ;
  thisNodeParams -> bridgeContactorStatus = inputs.bit.in4 ;

  // Read inputs coming from Delta:
  // Further variable declarations and initializations
  DeltaModule  *thisConverterB;
  NodeParams powerGlobalParamsShadow;
  int i;
  int calculatedStrings = 0;

  // Note the use of shadowed structures. The NodeParams are shadowed because some interruptions may
  // access to these variables by the time they are being used in math expressions.
  powerGlobalParamsShadow.outputVoltage = 0.0;
  powerGlobalParamsShadow.outputCurrent = 0.0;
  powerGlobalParamsShadow.availableVoltage = 1000.0;
  powerGlobalParamsShadow.availableCurrent = 0.0;
  powerGlobalParamsShadow.temperature1 = -120;
  powerGlobalParamsShadow.temperature2 = -120;
  powerGlobalParamsShadow.temperature3 = -120;
  powerGlobalParamsShadow.temperature4 = -120;
  powerGlobalParamsShadow.humidity1 = 0.0;


  for (i = 1; i <= (maxNumberOfConvertersWorking); i++){
    // For each converter, get the corresponding branch: the top and bottom. If converters are connected in series
    // current will flow through both. For this reason, if one Delta module has an error or it is not present, all
    // the branch must stop. Note that a proper numbering is crucial for this function, the location of the converters
    // inside the cabinet must be checked. Every branch is called "string" as in solar applications.

      thisConverterB = getModuleFromPosition(i);

      if (thisConverterB != &nullConverter){ // This line checks the presence of the modules
          calculatedStrings++;
          powerGlobalParamsShadow.outputVoltage += thisConverterB -> outputVoltage; //Total voltage is the sum of both modules
          // In case of current, the best reading is a mean between all modules in one string. There is also a correction made because of the use of integers
          // to measure current. There is a rounding issue in Delta's modules.

          currentB = ((thisConverterB -> outputCurrent + 1.01) > (thisConverterB -> currentSetPoint )) ? thisConverterB -> currentSetPoint : thisConverterB -> outputCurrent;
          powerGlobalParamsShadow.outputCurrent += (currentB); // Mean calculation
          // The available current will be the minimum available current per string. Forcing more current per converter will cause a module destruction.
          powerGlobalParamsShadow.availableCurrent +=  thisConverterB -> availableCurrent;
          // Finally, temperatures

          powerGlobalParamsShadow.temperature1 = MAXIMUM(thisConverterB -> temperaturePrimary, powerGlobalParamsShadow.temperature1);

          powerGlobalParamsShadow.temperature2 = MAXIMUM(thisConverterB -> temperatureSecondary, powerGlobalParamsShadow.temperature2);

          powerGlobalParamsShadow.temperature3 = MAXIMUM(thisConverterB -> temperatureAir, powerGlobalParamsShadow.temperature3);
      }
      else
      {
#if	DEBUG89

			env_debug("convertidores insuficientes ... \n");

#endif
      }
  }
  powerGlobalParamsShadow.outputVoltage /= (float)calculatedStrings; // Mean calculation of voltage read for all strings
  powerGlobalParamsShadow.temperature4 = Temperature; // Get the temperature from the nodes board (take a look into i2c peripherals)
  powerGlobalParamsShadow.humidity1 = Humidity; // Get humidity from nodes board
  // Update values from shadow to final location
  thisNodeParams -> outputVoltage = powerGlobalParamsShadow.outputVoltage;
  thisNodeParams -> outputCurrent = powerGlobalParamsShadow.outputCurrent;
  thisNodeParams -> availableVoltage = powerGlobalParamsShadow.availableVoltage;
  // There is a trick here: available current can be also limited by an alarm, emergency or transitions. For this reason, the available current can be decreased
  // under demand.
  float aux = powerGlobalParamsShadow.availableCurrent - decreaseCurrent;
  if (aux < 0.0) aux = 0.0;
  if (globalState == PREPARATION_F2 || globalState == INITIAL_STATE_A1) aux = 0.0;
  thisNodeParams -> availableCurrent = aux;
  thisNodeParams -> temperature1 = powerGlobalParamsShadow.temperature1;
  thisNodeParams -> temperature2 = powerGlobalParamsShadow.temperature2;
  thisNodeParams -> temperature3 = powerGlobalParamsShadow.temperature3;
  thisNodeParams -> temperature4 = Temperature;
  thisNodeParams -> humidity1 = powerGlobalParamsShadow.humidity1;
  thisNodeParams -> stateNumber = globalState;

  readPowerMeter();

  // Read fan revolutions. After that, load the values in the memory structure.
  tachometerManager();
  thisNodeParams -> rpm1 = lastCapture1;
  thisNodeParams -> rpm2 = lastCapture2;

  return; // No more
}

void updateOutputsNodes(void)
{
  // Initialize variables

  DeltaModule *thisConverter;

  objectDictionary[SERVICIO] = service;

  can_UpdateMsgs();

  // Outputs are compared with masks because two complementary softwares can be running in the same board at the same time.
// In case of emergency in Dispenser enables are set to 0
  if (DISPENSER_EMERGENCY_ON){
	  outputs.all = activeOutputRelaysVector & 0x0000; //0011 1111b
	  pol = (unsigned int) activeOutputRelaysVector & 0x0000;
  }
  else{
	  outputs.all = activeOutputRelaysVector & 0x007F; //0011 1111b
	  pol = (unsigned int) activeOutputRelaysVector & 0x007F;
  }

  if (pol != pol_ant){
      pol_ant = pol;
#if DEBUG
      sprintf(bufferstr, "Valor relays: %x \n", pol);

      env_debug(bufferstr);

      env_debug("\n");
#endif
  }


  outputs.bit.socketContactor = (socketContactorOutput == 1) ? 1 : 0;
  outputs.bit.bridgeContactor = (bridgeContactorOutput == 1) ? 1 : 0;
  outputs.bit.acRelay = (acRelayOutput == 1) ? 1 : 0;

  // Apply enable outputs

  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,(GPIO_PinState)outputs.bit.enable1);
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,(GPIO_PinState)outputs.bit.enable2);
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,(GPIO_PinState)outputs.bit.enable3);
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,(GPIO_PinState)outputs.bit.enable4);
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,(GPIO_PinState)outputs.bit.enable5);
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,(GPIO_PinState)outputs.bit.enable6);
  /* enable a�adido  */
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,(GPIO_PinState)outputs.bit.enable7);


  // HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,(GPIO_PinState)outputs.bit.bridgeContactor);
  // HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,(GPIO_PinState)outputs.bit.socketContactor);

#if	DEBUG
  if (oldoutput != outputs.bit.acRelay){
	  oldoutput = outputs.bit.acRelay;
	  if (outputs.bit.acRelay == 1)
			env_debug("Activo ac rele \n");
	  else
			env_debug("Desactivo ac rele \n");
  }

#endif


  // HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,(GPIO_PinState)outputs.bit.acRelay);  /* ventilador */
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,0);
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,0);
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,0);
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,0);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,(GPIO_PinState)outputs.bit.acRelay);

  //System Leds manager. This function acts as a visual heart beat. If the system hangs the LED becomes steady.
  if (timerBlinkLed > TIMER_1S)
  {
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
    if (timerBlinkLed > (TIMER_1S + TIMER_50MS))
    {
      timerBlinkLed = 0;
      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
    }
  }

  // Indicates an emergency error
  if (globalState == EMERGENCY_STOP_D1)
  {
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
  }
  else  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);

  // Set the fan requirements only for warm-up. After that, return the ventilation control to the program.
  if (timerWarmUp < TIMER_30S && globalState == INITIAL_STATE_A1) thermalManagementDelta(FORCE_COOLING_LEVEL_MAX);
  else thermalManagementDelta(AUTO_COOLING);

  // Modbus section, all parameters from the node are transferred to the modbus dictionary. The master node will collect all the data
  // and will place it on its own dictionary.

  unsigned int j = 0;
  unsigned int h = 0;
  unsigned int hs = 0;
  unsigned int nodePadding = 0;

  // First of all, according to the node, check where should be placed the data in the dictionary. It should be noted that
  // all data is correlative and the size of the dictionary related to the power modules is always the same.
  // Although the data is placed in the modbus dictionary, it will be transferred by CAN to the master, because it is critical.
  // Get the address padding:
  if (own_node == networkParams.node1.nodeNumber)  nodePadding = 0;
  if (own_node == networkParams.node2.nodeNumber)  nodePadding = 742;


  while ( j < maxNumberOfConvertersWorking)
  {

    thisConverter = getModuleFromPosition(j+1);

    // objectDictionary[i++] = thisConverter->statusWord;
    objectDictionary[nodePadding + DELTA25_CAN_ID_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = thisConverter->identifier;
    objectDictionary[nodePadding + DELTA25_GROUP_ID_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = own_node;
    objectDictionary[nodePadding + DELTA25_PHYSICAL_ID_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = thisConverter->phyAddr;
    objectDictionary[nodePadding + DELTA25_REMOTE_STATUS_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = 0;
    objectDictionary[nodePadding + DELTA25_DC_OUTPUT_VOLTAGE_SETPOINT_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = (__u16)thisConverter->voltageSetPoint * 10.0;		// [0.1 V]

    objectDictionary[nodePadding + DELTA25_DC_OUTPUT_CURRENT_SETPOINT_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = (__u16)thisConverter->currentSetPoint * 10.0;		// [0.1 A]


    objectDictionary[nodePadding + DELTA25_DC_OUTPUT_POWER_SETPOINT_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = (__u16)thisConverter->powerSetPoint/100.0;
    objectDictionary[nodePadding + DELTA25_MODE_SERIAL_PARALEL_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = 0;
    objectDictionary[nodePadding + DELTA25_DC_OUTPUT_VOLTAGE_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = (__u16)thisConverter->outputVoltage * 10.0;	    // [0.1 V]
    objectDictionary[nodePadding + DELTA25_DC_OUTPUT_CURRENT_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = (__u16)thisConverter->outputCurrent * 10.0; // [0.1 A]
    objectDictionary[nodePadding + DELTA25_DC_OUTPUT_POWER_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = (__u16)thisConverter->outputPower / 100.0;
    objectDictionary[nodePadding + DELTA25_DC_FLAGS_MONITOR_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = 0;
    objectDictionary[nodePadding + DELTA25_FAN_SPEED1_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = (__u16)thisConverter->rpm1;		// [rpm]
    objectDictionary[nodePadding + DELTA25_FAN_SPEED2_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = (__u16)thisConverter->rpm2;		// [rpm]		// [rpm]
    objectDictionary[nodePadding + DELTA25_TEMPERATURE_AMBIENT_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = thisConverter->temperatureAir * 10.0;		// [0.1 ºC]
    objectDictionary[nodePadding + DELTA25_TEMPERATURE_PFC1_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = thisConverter->tempPFC1 * 10.0;		// [0.1 ºC]
    objectDictionary[nodePadding + DELTA25_TEMPERATURE_PFC2_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = thisConverter->tempPFC2 * 10.0;		// [0.1 ºC]
    objectDictionary[nodePadding + DELTA25_TEMPERATURE_OUTLET_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = thisConverter->tempOutlet * 10.0;		// [0.1 ºC]
    objectDictionary[nodePadding + DELTA25_INPUT_VOLTAGE_VR_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = (__u16)thisConverter->Vr;		// [0.1 V]
    objectDictionary[nodePadding + DELTA25_INPUT_VOLTAGE_VS_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = (__u16)thisConverter->Vs;	// [0.1 V]
    objectDictionary[nodePadding + DELTA25_INPUT_VOLTAGE_VT_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = (__u16)thisConverter->Vt;	// [0.1 V]
    objectDictionary[nodePadding + DELTA25_INPUT_VOLTAGE_FREQ_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = thisConverter->Freq;
    objectDictionary[nodePadding + DELTA25_INPUT_CURRENT_IR_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = thisConverter->Ir;             		// [0.1 A]
    objectDictionary[nodePadding + DELTA25_INPUT_CURRENT_IS_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = thisConverter->Is;   		// [0.1 A]
    objectDictionary[nodePadding + DELTA25_INPUT_CURRENT_IT_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = thisConverter->It;	// [0.1 A]
    objectDictionary[nodePadding + DELTA25_INPUT_POWER_PR_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = thisConverter->Pr;         		// [0.1 kW]
    objectDictionary[nodePadding + DELTA25_INPUT_POWER_PS_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = thisConverter->Ps; 		// [0.1 kW]
    objectDictionary[nodePadding + DELTA25_INPUT_POWER_PT_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = thisConverter->Pt;       		// [0.1 kW]
    objectDictionary[nodePadding + DELTA25_POSITION_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = thisConverter->position;
    // objectDictionary[nodePadding + DELTA25_ENERGIA_HIGH_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = 0;
    // objectDictionary[nodePadding + DELTA25_ENERGIA_LOW_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = 0;
    objectDictionary[nodePadding + DELTA25_POWER_UP_CYCLES_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = 0;
    objectDictionary[nodePadding + DELTA25_ERROR_DATA_WORD_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = thisConverter->errorDataWord;
    objectDictionary[nodePadding + DELTA25_REMAINING_CURRENT_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = 0;
    objectDictionary[nodePadding + DELTA25_ESTADO_DEL_CONVERTIDOR_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = thisConverter->statusWord;
    objectDictionary[nodePadding + DELTA25_RESERVADO0_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = 0;
    objectDictionary[nodePadding + DELTA25_RESERVADO1_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = 0;
    objectDictionary[nodePadding + DELTA25_RESERVADO2_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = 0;
    h = 0;
    hs = 0;

    while (h < 32){
        objectDictionary[nodePadding + DELTA25_SERIAL_NUMBER_1_W1_1 + (DELTA_ADDRESSES_PER_CONVERTER *j) + h] = (__u16)((thisConverter->serialNumber[hs] & 0xFFFF0000) >> 16);

        objectDictionary[nodePadding + DELTA25_SERIAL_NUMBER_1_W2_1 + (DELTA_ADDRESSES_PER_CONVERTER *j) + h] = (__u16)(thisConverter->serialNumber[hs] & 0x0000FFFF);
        h = h + 2;
        hs++;
    }

    h = 0;
    hs = 0;
    while (h < 24){
        objectDictionary[nodePadding + DELTA25_FW_VERSION_PFC_BOOT_W1_1 + (DELTA_ADDRESSES_PER_CONVERTER *j) + h] = (__u16)((thisConverter->version[hs] & 0xFFFF0000) >> 16);

        objectDictionary[nodePadding + DELTA25_FW_VERSION_PFC_BOOT_W2_1 + (DELTA_ADDRESSES_PER_CONVERTER *j) + h] = (__u16)(thisConverter->version[hs] & 0x0000FFFF);
        h = h + 2;
        hs++;
    }



    objectDictionary[nodePadding + DELTA25_SLEEP_TIME_HIGH_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = (__u16)((thisConverter->sleepTime & 0xFFFF0000) >> 16);
    objectDictionary[nodePadding + DELTA25_SLEEP_TIME_LOW_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = (__u16)(thisConverter->sleepTime & 0x0000FFFF);
    objectDictionary[nodePadding + DELTA25_STANDBY_TIME_HIGH_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = (__u16)((thisConverter->standbyTime & 0xFFFF0000) >> 16);
    objectDictionary[nodePadding + DELTA25_STANDBY_TIME_LOW_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = (__u16)(thisConverter->standbyTime & 0x0000FFFF);


    objectDictionary[nodePadding + DELTA25_PLOAD_00_25_TIME_HIGH_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = (__u16)((thisConverter->pload25 & 0xFFFF0000)  >> 16);
    objectDictionary[nodePadding + DELTA25_PLOAD_00_25_TIME_LOW_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] =(__u16)(thisConverter->pload25 & 0x0000FFFF);
    objectDictionary[nodePadding + DELTA25_PLOAD_25_50_TIME_HIGH_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = (__u16)((thisConverter->pload50 & 0xFFFF0000)  >> 16);
    objectDictionary[nodePadding + DELTA25_PLOAD_25_50_TIME_LOW_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = (__u16)(thisConverter->pload50 & 0x0000FFFF);
    objectDictionary[nodePadding + DELTA25_PLOAD_50_75_TIME_HIGH_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] =(__u16)((thisConverter->pload75 & 0xFFFF0000)  >> 16);
    objectDictionary[nodePadding + DELTA25_PLOAD_50_75_TIME_LOW_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = (__u16)(thisConverter->pload75 & 0x0000FFFF);

    objectDictionary[nodePadding + DELTA25_PLOAD_75_00_TIME_HIGH_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = (__u16)((thisConverter->pload100 & 0xFFFF0000) >> 16);
    objectDictionary[nodePadding + DELTA25_PLOAD_75_00_TIME_LOW_1 + (DELTA_ADDRESSES_PER_CONVERTER *j)] = (__u16)(thisConverter->pload100 & 0x0000FFFF);
    // objectDictionary[nodePadding + i++] = 7;
    j++;
  }
   return;
}

void updateStateMachineNodes(void)
{
  // Variable declaration
  int i, reconfigurationProcess, switchEnableProcess;
  int allDeltasDetected = 0;
  NodeParams *thisNodeParams;
  thisNodeParams = findNodeById(own_node);
  if (thisNodeParams == &voidNode)
      createEvent(own_node, SUB_SYSTEM_CAN_COMMUNICATIONS, SUB_SUB_SYSTEM_GENERIC, SEVERITY_FATAL, NOT_MAPPED_NODE);

  // These two functions act as a heart beat control. When the system communications are broken
  networkWatchNodes();
  nodeWatch();

  //** Updates the state machine of every single converter


  for (i=0;i<maxNumberOfConvertersWorking;i++)
  {
	  deltaConverter[i].yi = i;
      deltaConverter[i].machine.stateMachine(&deltaConverter[i]);
#if SIMUL_DELTAS
      deltaConverter[i].machine.pong = 1;
#endif
  }

  if (globalState != globalState_ant){
	  globalState_ant = globalState;
	  printf("Node state: %s\n",globalState_str[globalState]);
#if	DEBUG
		env_debug("......Node state:");
		env_debug(globalState_str[globalState]);
		env_debug("\n");
#endif
  }

  if (service > 0) globalState = SERVICE_STATE;
  switch (globalState)
  {
    case INITIAL_STATE_A1:
      cargando = 0;
      thisNodeParams -> availableCurrent = 0;
      timerInEmcy = 0;
      activeOutputRelaysVectorLast = 0; /** < Starting point for relays */
      activeOutputRelaysVector = 0;

      // How many Deltas are online?
      for (i=0;i<maxNumberOfConvertersWorking;i++)
      {
        if (deltaConverter[i].identifier > 0) allDeltasDetected++;
      }

      if (timerWarmUp > TIMER_15S)  acRelayOutput = 1; //Delay startup from reset


      if (timerWarmUp > TIMER_60S)
      {
        printf("Going to OPERATION REQUEST by timeout\n");
        //powerGlobalParams.command = OPERATION_REQUEST;
        globalState = PREPARATION_F2;
        firstPass = 0;
      }
      // Drive directly to operation request if all Deltas are detected
//      if (allDeltasDetected == maxNumberOfConvertersWorking)
      if (discoveredModules == maxNumberOfConvertersWorking)
      {
        //powerGlobalParams.command = OPERATION_REQUEST;
        printf("All deltas detected, going to OPERATION REQUEST!\n");
#if	DEBUG
		env_debug("All deltas detected, going to OPERATION REQUEST!");


		env_debug("\n");
#endif
        globalState = PREPARATION_F2;
        firstPass = 0;
      }


      break;
    case PREPARATION_F2:
        /** Converter identification, system configuration */
        timerInEmcy = 0;
        thisNodeParams -> availableCurrent = 0;
        thisNodeParams -> followedSocket = 0;

#if   EPROMS_IN_DELTAS
        switchEnableProcess = deltasReady();
        activeOutputRelaysVector = 0x0000FFFF;
#else
        switchEnableProcess = switchEnables();
#endif

        if (switchEnableProcess > 0) {
            reconfigurationProcess = reconfigureOutputs();
            if ( reconfigurationProcess > 0)
            {

              globalState = PRODUCTION_F1;
              thisNodeParams -> followedSocket = thisNodeParams ->followedSocketRequest;
              printf("Equipment transition: INITIATING PRODUCTION STATE\n");
              createEvent(own_node, SUB_SYSTEM_STATE_MACHINE, SUB_SUB_SYSTEM_TRANSITIONS, SEVERITY_INFO, INFO_NODE_READY);
#if DEBUG
				    env_debug("Equipment transition: INITIATING PRODUCTION STATE\n");
		    		env_debug("\n");
#endif
            }
            if ( reconfigurationProcess < 0)
            {
              globalState = EMERGENCY_STOP_D1; //Contacts soldered, relay malfunction. Is critical
              printf("Equipment malfunction, failed conf.: EMERGENCY REQUEST\n");
              createEvent(own_node, SUB_SYSTEM_STATE_MACHINE, SUB_SUB_SYSTEM_TRANSITIONS, SEVERITY_ERROR, ERROR_RECONFIGURATION_EXITED_W_ERROR);
#if DEBUG
				    env_debug("Equipment malfunction, failed conf.: EMERGENCY REQUEST\n");
		    		env_debug("\n");
#endif
            }
        }
        if (switchEnableProcess < 0)
        {
          globalState = EMERGENCY_STOP_D1; //Contacts soldered, relay malfunction. Is critical
          printf("Equipment malfunction, failed enables: EMERGENCY REQUEST\n");
          createEvent(own_node, SUB_SYSTEM_STATE_MACHINE, SUB_SUB_SYSTEM_TRANSITIONS, SEVERITY_ERROR, ERROR_NO_DELTAS_DETECTED);
#if DEBUG
				    env_debug("Equipment malfunction, failed enables: EMERGENCY REQUEST\n");
		    		env_debug("\n");
#endif
        }
        break;
    case PRODUCTION_F1:
    	if (empieza_carga == 0){
       #if DEBUG
    		 for (i = 0; i < maxNumberOfConvertersWorking; i++){
    	            sprintf(bufferstr, "Delta: %d id: %d    Phisical add.: %d      pos: %d ", (int)deltaConverter[i].yi, (int)deltaConverter[i].identifier, (int)deltaConverter[i].phyAddr, (int)deltaConverter[i].position);

		    		env_debug(bufferstr);

    		    		env_debug("\n");
    		    		env_debug("********************************************************");
    		    		env_debug("\n");
    		 }
         #endif
    	}
    	empieza_carga = 1;
        timerInEmcy = 0;
        applyCommands();
        if (thisNodeParams -> currentConfiguration == STANDBY &&  thisNodeParams -> configurationSetPoint  == STANDBY)
        	thisNodeParams -> currentConfiguration = NOT_CONNECTED; /* para forzar a ir a STANDBY */


        if (thisNodeParams -> currentConfiguration != thisNodeParams -> configurationSetPoint)
        {
        	if (thisNodeParams -> configurationSetPoint > 0){
        		cargando = 1;

        	}
        	else{
        		if (cargando == 1){
                    offsetPosition ++;
                    if (offsetPosition > maxNumberOfConvertersWorking - 1){
                    	offsetPosition = 0;
                    }
        		}
        		cargando = 0;

        	}
            globalState = REQUESTED_STOP_A3;
            printf("REQUESTING STOP there is a new configuration\n");
#if DEBUG
				    env_debug("REQUESTING STOP there is a new configuration\n");
		    		env_debug("\n");
#endif
        }
        else if ( thisNodeParams -> configurationSetPoint == SUPPORT && bridgeContactorOutput == 0){
        	printf("no puede ser REQUESTING STOP  new configuration\n");
#if DEBUG
				    env_debug("no puede ser REQUESTING STOP  new configuration\n");
		    		env_debug("\n");
#endif
        	globalState = REQUESTED_STOP_A3;
        }

        if (thisNodeParams -> currentConfiguration == SOCKET )
        {
            if (thisNodeParams -> socketContactorRequest == 1 && socketContactorOutput == 0)
              socketContactorOutput = thisNodeParams -> socketContactorRequest;
            if (thisNodeParams -> socketContactorRequest == 0 && socketContactorOutput == 1)
              globalState = REQUESTED_STOP_A3;

            bridgeContactorOutput = 0; //Yes, always, the bridge relay is controlled be the supporting converter
        }
        if (errorCommunication > 0 || DISPENSER_EMERGENCY_ON || nodesHaveErrorsFlag)
          {
/*            socketContactorOutput = 0;
            bridgeContactorOutput = 0;*/
#if DEBUG

				    env_debug("errores...\n");
		    		env_debug("\n");
#endif
            globalState = REQUESTED_STOP_A3;
          }
        break;
    case REQUESTED_STOP_A3:
        timerInEmcy = 0;
        //applyCommands();
        if (decreasePowerSoftly())
        {
          if (masterHasErrorsFlag || networkIsCriticalFlag || nodesHaveErrorsFlag || DISPENSER_EMERGENCY_ON || errorCommunication > 0)
          {
            reconfigurationProcess = reconfigureOutputs();
            if ( reconfigurationProcess > 0 || DISPENSER_EMERGENCY_ON || errorCommunication > 0)
				 globalState = EMERGENCY_STOP_D1; // Be sure that everything is opened
          }
          else if (thisNodeParams -> configurationSetPoint == STANDBY){
        	  thisNodeParams -> currentConfiguration = thisNodeParams -> configurationSetPoint;
        	  acRelayOutput = 0;
        	  globalState = STANDBY_A2;
          }
          else
        	  globalState = PREPARATION_F2;
        }

        break;

    case EMERGENCY_STOP_D1:
      for (i = 0; i < maxNumberOfConvertersWorking; i++)
      {
        deltaConverter[i].commandWord = SWITCH_OFF_COMMAND | DEEP_SLEEP_COMMAND; //All converters to off state
      }

	    thisNodeParams -> command = DISCONNECTION_REQUEST;
      if (errorCommunication == 0 && masterHasErrorsFlag == 0 && nodesHaveErrorsFlag == 0 && networkIsCriticalFlag == 0 && timerInEmcy > TIMER_1S  && !DISPENSER_EMERGENCY_ON )
      {
		#if	DEBUG
			env_debug("voy a REPARE_FAILURE_D2 state:");

			env_debug("\n");
		#endif
        globalState = REPARE_FAILURE_D2;
      }
      if (thisNodeParams -> outputCurrent <= 10.0 && timerInEmcy > TIMER_500MS)
      {
#if	DEBUG7
    	  env_debug("apaga los dos reles <= 10.0 y timerInEmcy > TIMER_500MS  \n");

#endif
        socketContactorOutput = 0;
        bridgeContactorOutput = 0;
        thisNodeParams -> currentConfiguration = NOT_CONNECTED;
      }
      thisNodeParams -> followedSocket = 0;
      break;

    case REPARE_FAILURE_D2:
      globalState = PREPARATION_F2;
      break;

    case STANDBY_A2:
      deltaLastReceivedMessage = 0;
      deltaErrorComm = 0;
      empieza_carga = 0;
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
        if ((thisNodeParams -> currentConfiguration != thisNodeParams -> configurationSetPoint) && errorCommunication == 0)
         {
#if	DEBUG
		env_debug("voy a PREPARATION_F2 state:");

		env_debug(globalState_str[globalState]);
		env_debug("\n");
#endif
		timerEstabRele = 0;
		acRelayOutput = 1;

		globalState = WAIT_STANDBY_A2;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);

#if	SIMUL_DELTAS
        	 for (i=0;i<maxNumberOfConvertersWorking;i++)
        	  {

        	      deltaConverter[i].machine.estadoActualConvertidor = 0;
        	      deltasSIMUL[i].msgIdSimul = 1;
        	      nodeList[i] = 0;
        	  }
#endif
         }
        else
          {

           if (DISPENSER_EMERGENCY_ON || errorCommunication > 0)
            {
				#if	DEBUG
						env_debug("voy a EMERGENCY");

						env_debug("\n");
				#endif

          	     acRelayOutput = 1;

          	     globalState = EMERGENCY_STOP_D1;
             }

          }


      break;

    case ERROR_COMM:
  /*  *******************
   * Este estado ya no se usa, los errores de comunicaciones se derivan al estado de emergencia,
   * se mantiene el estado para compatibilidad con el software de MOde4
   *
   */


      break;

    case WAIT_STANDBY_A2:
    	deltaLastReceivedMessage = 0;
        if (timerEstabRele > TIMER_30S)
        {
        	if  ((deltasReady() == 1 ) | (timerEstabRele > TIMER_30S)){
            	empieza_carga = 0;
                globalState = PREPARATION_F2;
                timerEstabRele = -1;

        	}
        }
        break;

    case SERVICE_STATE:
        if (service == 1){
   	       if (decreasePowerSoftly())
   	        {
   		           globalState = SERVICE_STATE;
					#if	DEBUG
							env_debug("Power decreased...");
							env_debug("\n");
					#endif
					service = 2;
   	        }
         }

        if (service == 2){
			  for (i = 0; i < maxNumberOfConvertersWorking; i++)
			  {
				deltaConverter[i].commandWord = SWITCH_OFF_COMMAND | DEEP_SLEEP_COMMAND; //All converters to off state
			  }
				#if	DEBUG
						env_debug("Deltas apagados...");
						env_debug("\n");
				#endif
             service = 3;
      }
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);

        acRelayOutput = 0;
		socketContactorOutput = 0;
		bridgeContactorOutput = 0;
		thisNodeParams -> currentConfiguration = NOT_CONNECTED;
        thisNodeParams -> followedSocket = 0;
      break;

    default:
        //stay
        break;
  };
}


/** This function used to know if Eproms/deltas are in the right position */
int deltasReady(void)
{

    int i,j;
	int found = 0;
	int deltasReady  = 0;

	for (j = 1; j < maxNumberOfConvertersWorking + 1; j++)
	{

		 for (i = 0; i < maxNumberOfConvertersWorking; i++)
		 {
#if EPROMS_IN_DELTAS
			if ((deltaConverter[i].phyAddr == j) && (deltaConverter[i].position == j))
			{
			  found ++;
			  break;
			}
#else
			if ((deltaConverter[i].position == j))
			{
			  found ++;
			  break;
			}
#endif

		 }
	}
    if (found == discoveredModules){
    	deltasReady = 1;
    }

	return(deltasReady);

}

/** This function is a bit messy, it switches the output relays one by one and skips identified modules */
int switchEnables(void)
{
  static EnableCycleMachineState deltaEnableState  = EN_CONFIGURE;

  int i, positionFound = 0, forceLeave = 0;
  unsigned long fullTrays = 0, tryThisPosition = 0, shiftRelays = 0, equivalentTray = 0;

  for (i = 0; i < maxNumberOfConvertersWorking; i++)
  {
    if (serialNodeRegister[i].tray != 0) fullTrays++;
  }

  switch (deltaEnableState){
  case EN_CONFIGURE:
    timerEnablesGlobal = 0;
    timerEnablesRelayClosed = 0;
    timerEnablesRelayOpened = TIMER_2S;
    deltaEnableState++;
    //break;
  case EN_IDENTIFY_RELAY:
    //timerEnablesRelayOpened = 0;
    timerEnablesRelayClosed = 0;
    activeOutputRelaysVector = 0;

    if (timerEnablesRelayOpened >= TIMER_2S)
    {
      if  (activeOutputRelaysVectorLast == 0 || activeOutputRelaysVectorLast >= (0x00000001 << (maxNumberOfConvertersWorking-1))) \
                      tryThisPosition = 1;
      else tryThisPosition = activeOutputRelaysVectorLast << 1;

      for (shiftRelays = 0; shiftRelays < maxNumberOfConvertersWorking; shiftRelays++)
      {
          if ((tryThisPosition >> shiftRelays) & 0x00000001){
        	  equivalentTray = shiftRelays + 1;
          }
      }

      for (i = 0; i < maxNumberOfConvertersWorking; i++)
      {
          if (serialNodeRegister[i].tray == equivalentTray){
        	  positionFound = 1;
          }
      }

      if (positionFound == 0){
          deltaEnableState++;
          printf("Enabling tray %d \n", (int)equivalentTray);
#if DEBUG
  		env_debug("Enabling tray:");

  		env_debug_int((int)equivalentTray);
  		env_debug("\n");
#endif
      }
      activeOutputRelaysVectorLast = tryThisPosition;
    }
    if (timerEnablesGlobal > TIMER_60S || fullTrays == maxNumberOfConvertersWorking){
      deltaEnableState = EN_ALL_ON;
    }
    break;

  case EN_WAIT_IDENTIFICATION:
    timerEnablesRelayOpened = 0;
    //timerEnablesRelayClosed = 0;

    activeOutputRelaysVector = activeOutputRelaysVectorLast;

    for (shiftRelays = 0; shiftRelays < maxNumberOfConvertersWorking; shiftRelays++)
    {
        if ((activeOutputRelaysVector >> shiftRelays) & 0x00000001) equivalentTray = shiftRelays + 1;
    }

    for (i = 0; i < maxNumberOfConvertersWorking; i++)
    {
        if (serialNodeRegister[i].tray == equivalentTray)
        {
          positionFound = 1;
          printf(" Found converter in %d \n",(int)equivalentTray);
#if DEBUG
    		env_debug(" Found converter in:");

    		env_debug_int((int)equivalentTray);
    		env_debug("\n");
#endif
        }
    }

    if (timerEnablesRelayClosed >= TIMER_5S || positionFound){
        deltaEnableState--;
    }
    if (timerEnablesGlobal > TIMER_60S || fullTrays == maxNumberOfConvertersWorking){
        deltaEnableState = EN_ALL_ON;
    }
    break;
  case EN_ALL_ON:
    if (timerEnablesRelayClosed > TIMER_80MS) {
      timerEnablesRelayClosed = 0;
      if  (activeOutputRelaysVector == 0 || activeOutputRelaysVector >= (0x00000001 << (maxNumberOfConvertersWorking-1))) {
          activeOutputRelaysVector |= 1;
      }
      activeOutputRelaysVector |= activeOutputRelaysVector << 1;
      if ((~activeOutputRelaysVector) == 0) forceLeave = 1;
    }
    break;
  case EN_IDENTIFICATION_DONE:
   activeOutputRelaysVector = 0x0000FFFF;
   if (fullTrays == 0)  {
     deltaEnableState = EN_CONFIGURE;
     //generate an event!
   return(-1);




   }
   else return(fullTrays);
//         unreachable break;
  default:
    deltaEnableState = EN_CONFIGURE;
    break;
  }
  if ((timerEnablesGlobal > (TIMER_60S + TIMER_1S)) || forceLeave){
    deltaEnableState = EN_IDENTIFICATION_DONE;
    activeOutputRelaysVector = 0x0000FFFF;
    if (fullTrays == 0)  return(-1);
    else return(fullTrays);
  } else return (0);
}


int reconfigureOutputs(void)
{
  static ConfigureOutputContactors outputContactorsConfiguration    = CONF_OPEN_EVERYTHING;

  /*EventLocation event;*/
  int i;
  int retval = 0; //Reconfiguration in progress

  NodeParams *thisNodeParams;
  thisNodeParams = findNodeById(own_node);
  if (thisNodeParams == &voidNode)
      createEvent(own_node, SUB_SYSTEM_CAN_COMMUNICATIONS, SUB_SUB_SYSTEM_GENERIC, SEVERITY_FATAL, NOT_MAPPED_NODE);

  if (outputContactorsConfiguration != outputContactorsConfiguration_ant){
	  outputContactorsConfiguration_ant = outputContactorsConfiguration;
  	 // printf("Reconfigure state: %s\n",outputContactorsConfiguration_str[outputContactorsConfiguration]);
#if	DEBUG
  		env_debug("...Reconfigure state:");

  		env_debug(outputContactorsConfiguration_str[outputContactorsConfiguration]);
  		env_debug("\n");
#endif
    }

  switch (outputContactorsConfiguration){
    case(CONF_OPEN_EVERYTHING):
      timerConfigurationRelays = 0;
      timerDelayForSocketConf = 0;
      for (i=0;i<maxNumberOfConvertersWorking;i++)
      {
        deltaConverter[i].commandWord = SWITCH_OFF_COMMAND | DEEP_SLEEP_COMMAND; //All converters to off state (redundant)
      }
      if (thisNodeParams -> outputCurrent <= 10.0)
      {
#if	DEBUG
    	  env_debug("apaga los dos reles <= 10.0 \n");
#endif
        socketContactorOutput = 0;
        bridgeContactorOutput = 0;
        outputContactorsConfiguration = CONF_WAIT_ALL_OPEN;
      }
      break;
    case(CONF_WAIT_ALL_OPEN):
      if (ALLOW_RELAY_MONITORING == 1)
      {
        if (thisNodeParams -> socketContactorStatus == 0 && thisNodeParams -> bridgeContactorStatus == 0 && \
                  thisNodeParams -> outputCurrent <= 10.0 && timerConfigurationRelays> TIMER_500MS)
        {
          if (thisNodeParams -> configurationSetPoint == SOCKET)
          {
            outputContactorsConfiguration = CONF_CLOSE_DESIDERED;
            thisNodeParams -> currentConfiguration = NOT_CONNECTED;
          }
          else if (timerDelayForSocketConf > TIMER_500MS)
          {
            outputContactorsConfiguration = CONF_CLOSE_DESIDERED;
            thisNodeParams -> currentConfiguration = NOT_CONNECTED;
          }
        }
      }
      if (ALLOW_RELAY_MONITORING == 0) //Pass anyway
      {
        if (thisNodeParams -> outputCurrent <= 10.0 && timerConfigurationRelays> TIMER_200MS)
        {
          if (thisNodeParams -> configurationSetPoint == SOCKET)
          {
            outputContactorsConfiguration = CONF_CLOSE_DESIDERED;
            thisNodeParams -> currentConfiguration = NOT_CONNECTED;
          }
          else if (timerDelayForSocketConf > TIMER_500MS)
          {
            outputContactorsConfiguration = CONF_CLOSE_DESIDERED;
            thisNodeParams -> currentConfiguration = NOT_CONNECTED;
          }
        }
      }

      //Check timeouts. If relays are not being monitored, this statement will not be achievable.
      if (timerConfigurationRelays > TIMER_2S) //In two seconds all relays are still open
      {
          if (thisNodeParams -> socketContactorStatus)
              /*event =*/ createEvent(own_node, SUB_SYSTEM_CONTACTORS, SUB_SUB_SYSTEM_DC_SOCKET, SEVERITY_WARNING, SOCKET_CONTACTOR_DOES_NOT_OPEN);
          if (thisNodeParams -> bridgeContactorStatus)
              /*event =*/ createEvent(own_node, SUB_SYSTEM_CONTACTORS, SUB_SUB_SYSTEM_DC_SOCKET, SEVERITY_WARNING, BRIDGE_CONTACTOR_DOES_NOT_OPEN);
          //outputContactorsConfiguration = CONF_RELAYS_FATAL_ERROR;
#if	DEBUG
          env_debug("apaga los dos reles despues de 2 secs\n");
#endif
          socketContactorOutput = 0;
          bridgeContactorOutput = 0;
          retval = -1;
      }
      break;
    case(CONF_CLOSE_DESIDERED):
      timerConfigurationRelays = 0;
      if (thisNodeParams -> configurationSetPoint == SUPPORT)
      {
        socketContactorOutput = 0;
#if	DEBUG
       env_debug("enciende rele de support\n");
#endif
        bridgeContactorOutput = 1;
      }
      else if (thisNodeParams -> configurationSetPoint == SOCKET)
      {
        socketContactorOutput = thisNodeParams -> socketContactorRequest;
#if	DEBUG
        env_debug("enciende rele de socket\n");
#endif
        bridgeContactorOutput = 0;
      }
      else
      {
#if	DEBUG
    	env_debug("apaga los dos reles...\n");
#endif
        socketContactorOutput = 0;
        bridgeContactorOutput = 0;
      }
      outputContactorsConfiguration = CONF_WAIT_DESIDERED;
      break;
    case(CONF_WAIT_DESIDERED):
      if (thisNodeParams -> socketContactorStatus == socketContactorOutput && thisNodeParams -> bridgeContactorStatus == bridgeContactorOutput && timerConfigurationRelays > TIMER_500MS)
      {
        outputContactorsConfiguration = CONF_PROCESS_FINISHED;
        thisNodeParams -> currentConfiguration = thisNodeParams -> configurationSetPoint;
#if	DEBUG
        env_debug("iguala current a config 500  :");
        if (thisNodeParams -> configurationSetPoint == 0)
 		   env_debug("  NOT_CONNECTED");
         else if (thisNodeParams -> configurationSetPoint == 1)
  		   env_debug("  SOCKET");
         else if (thisNodeParams -> configurationSetPoint == 2)
             		   env_debug("  SUPPORT");
         else
   		   env_debug("   STANDBY");

         env_debug("\n");
#endif
      }
      if (ALLOW_RELAY_MONITORING == 0 && timerConfigurationRelays > TIMER_200MS) //Pass anyway
      {
#if	DEBUG
    	  env_debug("iguala current a config 200   :");

          if (thisNodeParams -> configurationSetPoint == 0)
   		   env_debug("  NOT_CONNECTED");
           else if (thisNodeParams -> configurationSetPoint == 1)
    		   env_debug("  SOCKET");
           else if (thisNodeParams -> configurationSetPoint == 2)
               		   env_debug("  SUPPORT");
           else
     		   env_debug("   STANDBY");

           env_debug("\n");
#endif
        outputContactorsConfiguration = CONF_PROCESS_FINISHED;
        thisNodeParams -> currentConfiguration = thisNodeParams -> configurationSetPoint;
      }
      if (timerConfigurationRelays > TIMER_2S) // No feedback in two seconds
      {
        if (thisNodeParams -> socketContactorStatus != socketContactorOutput)
            /*event = */createEvent(own_node, SUB_SYSTEM_CONTACTORS, SUB_SUB_SYSTEM_DC_SOCKET, SEVERITY_FATAL, SOCKET_CONTACTOR_DOES_NOT_CLOSE);
        if (thisNodeParams -> bridgeContactorStatus != bridgeContactorOutput)
            /*event = */createEvent(own_node, SUB_SYSTEM_CONTACTORS, SUB_SUB_SYSTEM_DC_SOCKET, SEVERITY_FATAL, BRIDGE_CONTACTOR_DOES_NOT_CLOSE);
        // outputContactorsConfiguration = CONF_RELAYS_FATAL_ERROR;
#if	DEBUG
        env_debug("apaga los dos reles por deseo  \n");
#endif
        socketContactorOutput = 0;
        bridgeContactorOutput = 0;
        retval = -1;
      }
      break;
    case(CONF_PROCESS_FINISHED):
      retval = 1;
      outputContactorsConfiguration = CONF_OPEN_EVERYTHING;
      break;
    case(CONF_RELAYS_FATAL_ERROR):
#if	DEBUG
		  env_debug("apaga los dos reles por error \n");
#endif
      socketContactorOutput = 0;
      bridgeContactorOutput = 0;
      retval = -1;
      break;
  }
  return (retval);
}

int decreasePowerSoftly(void)
{
  int retVal = 0;
  int i;
  float decreaseRatio;

  if (masterHasErrorsFlag || networkIsCriticalFlag) decreaseRatio = 200.0; //20.0
  else decreaseRatio = 200.0; //2.0

  // Check that the network is properly mapped
  NodeParams *thisNodeParams;
  thisNodeParams = findNodeById(own_node);
  if (thisNodeParams == &voidNode)
      createEvent(own_node, SUB_SYSTEM_CAN_COMMUNICATIONS, SUB_SUB_SYSTEM_GENERIC, SEVERITY_FATAL, NOT_MAPPED_NODE);

  if (timerDecreasePower >= TIMER_100MS)
  {
    timerDecreasePower = 0;
    decreaseCurrent += decreaseRatio;
#if	DEBUG
    env_debug("decrease current ...\n");
#endif
  }

  if (thisNodeParams -> availableCurrent > thisNodeParams -> outputCurrent)
  {
    thisNodeParams -> availableCurrent = thisNodeParams -> outputCurrent;
  }

  thisNodeParams -> availableCurrent -= decreaseCurrent; //Was calculated previously.
  if (thisNodeParams -> availableCurrent < 0.0) //Current command is equal to zero
  {
    thisNodeParams -> availableCurrent = 0.0;
    decreaseCurrent = 0.0;
   /* Nodeparam all set to 0 , just in case any value can activate deltas */
	// thisNodeParams -> configurationSetPoint = NOT_CONNECTED;
    thisNodeParams -> voltageSetPoint = 0.0;
    thisNodeParams -> currentSetPoint = 0.0;
    thisNodeParams -> command = DISCONNECTION_REQUEST;
    // thisNodeParams -> followedSocketRequest = 0;
    // thisNodeParams -> followedSocket = 0;
    // thisNodeParams -> lastReceivedMsg = 0;
#if	DEBUG
    env_debug("apago deltas ...\n");
#endif

    for (i = 0; i < maxNumberOfConvertersWorking; i++)
    {
      deltaConverter[i].commandWord = SWITCH_OFF_COMMAND | DEEP_SLEEP_COMMAND; //All converters to off state
    }
    retVal = 1;
    // return retVal;
  }
  else{
	  //Apply commands in decrease mode:
	  DeltaModule  *thisConverterB;
	  NodeParams powerGlobalParamsShadow;
	  //  float loadRatio = 0.0;
	  NodeParams *complementaryNode;
	  NodeParams complementaryNodeSC; //Shadow copy

	  thisNodeParams = findNodeById(own_node);
	  if (thisNodeParams == &voidNode)
	      createEvent(own_node, SUB_SYSTEM_CAN_COMMUNICATIONS, SUB_SUB_SYSTEM_GENERIC, SEVERITY_FATAL, NOT_MAPPED_NODE);

	  float remainingCurrentSetPoint = thisNodeParams -> currentSetPoint;
	  float availableCurrentCumulative = 0.0;

	  if (thisNodeParams -> configurationSetPoint == SUPPORT)
	  {
	    complementaryNode = findNodeById(thisNodeParams -> connectedToSupport);
	    // It is better to work using shadow copies because these register may change during execution.
	    complementaryNodeSC.availableCurrent = complementaryNode->availableCurrent;
	    complementaryNodeSC.outputCurrent = complementaryNode->outputCurrent;
	    complementaryNodeSC.currentSetPoint = complementaryNode->currentSetPoint;

	    if (complementaryNode == &voidNode || complementaryNode->nodeNumber == own_node)
	      createEvent(own_node, SUB_SYSTEM_CAN_COMMUNICATIONS, SUB_SUB_SYSTEM_GENERIC, SEVERITY_FATAL, MULTIPLE_NODE_DEFINITION);

	    if (complementaryNodeSC.availableCurrent < thisNodeParams -> currentSetPoint)
	    {
	      remainingCurrentSetPoint = thisNodeParams -> currentSetPoint - complementaryNodeSC.availableCurrent;
	    }
	    else remainingCurrentSetPoint = 0.0;
	  }

	  /*Just check integrity*/
	  if (thisNodeParams -> voltageSetPoint > thisNodeParams -> maxVoltageRequest) thisNodeParams -> voltageSetPoint = thisNodeParams -> maxVoltageRequest;
	  if (remainingCurrentSetPoint < 0.0) remainingCurrentSetPoint = 0.0;
	  if (remainingCurrentSetPoint > thisNodeParams -> availableCurrent) remainingCurrentSetPoint = thisNodeParams -> availableCurrent;

	  powerGlobalParamsShadow.availableCurrent = 0.0;

	  // Commands for switching off the converters
	  if  (thisNodeParams -> voltageSetPoint < 10.0 || thisNodeParams -> command == STOP_REQUEST ||\
	    thisNodeParams -> command == WAIT_REQUEST || thisNodeParams -> command == DISCONNECTION_REQUEST )
	  {
	    for (i = 0; i < maxNumberOfConvertersWorking; i++)
	    {
	      deltaConverter[i].commandWord = SWITCH_OFF_COMMAND | DEEP_SLEEP_COMMAND; //All converters to off state
	    }
	    for (i = 1; i <= (maxNumberOfConvertersWorking); i++)
	    {
	      //Rules for the corresponding converters:

	      thisConverterB = getModuleFromPosition(i);

	      if (thisConverterB != &nullConverter)
	      {
	        powerGlobalParamsShadow.availableCurrent =  thisConverterB -> availableCurrent;
	        availableCurrentCumulative += powerGlobalParamsShadow.availableCurrent;

	        thisConverterB -> voltageSetPoint = 0;
	       }
	    }
	  }
	  else{
			// Check strings one by one
			for (i = 1; i <= (maxNumberOfConvertersWorking); i++)
			{
			  //Rules for the corresponding converters:

			  thisConverterB = getModuleFromPosition(i);

			  if ( thisConverterB != &nullConverter)
			  {
				//thisConverterT -> commandWord = SWITCH_ON_COMMAND; //Action not required at this point
				//thisConverterB -> commandWord = SWITCH_ON_COMMAND;
				powerGlobalParamsShadow.availableCurrent = thisConverterB -> availableCurrent;
				availableCurrentCumulative += powerGlobalParamsShadow.availableCurrent;
				thisConverterB -> currentSetPoint = (remainingCurrentSetPoint > powerGlobalParamsShadow.availableCurrent) ? powerGlobalParamsShadow.availableCurrent : remainingCurrentSetPoint;

				remainingCurrentSetPoint -= thisConverterB -> currentSetPoint;
				thisConverterB -> voltageSetPoint = thisNodeParams -> voltageSetPoint;


				/* Special command for current difference compensation between readings and output */


				/* Special case for high power. This will allow charge with resistors.*/
				if (thisNodeParams -> outputVoltage > 200.0)
				{
				  thisConverterB -> voltageSetPoint = MINIMUM((thisNodeParams -> voltageSetPoint) , ((thisNodeParams -> outputVoltage + 10.0))) ;

				}
			  }
			}
			//thisNodeParams -> availableCurrent = availableCurrentCumulative;
		  }

  }

  return retVal;
}

/* This function sends the corresponding set points to every Delta. It also checks the current availability
 * for complementary nodes connected to the same socket. */
void applyCommands(void)
{
    float valor_optimo = 0.0;
	float sum = 0.0;
	int newPosition = 0;
  // Declarations related to the current node
  DeltaModule  *thisConverterB;
  NodeParams powerGlobalParamsShadow;
  NodeParams *thisNodeParams;
  int i;
  // Declarations related to the complementary node
  NodeParams *complementaryNode;
  NodeParams complementaryNodeSC; //Shadow copy

  thisNodeParams = findNodeById(own_node); //Returns the structure of the own node
  if (thisNodeParams == &voidNode) // And of course, it should be present
      createEvent(own_node, SUB_SYSTEM_CAN_COMMUNICATIONS, SUB_SUB_SYSTEM_GENERIC, SEVERITY_FATAL, NOT_MAPPED_NODE);

  float remainingCurrentSetPoint = thisNodeParams -> currentSetPoint;
  float availableCurrentCumulative = 0.0;

  // Special case when the node is acting as a support node (when it is not connected directly to the output)
  if (thisNodeParams -> configurationSetPoint == SUPPORT)
  {
    complementaryNode = findNodeById(thisNodeParams -> connectedToSupport); // Find the node which is supporting to get its data
    // It is better to work using shadow copies because these register may change during execution.
    complementaryNodeSC.availableCurrent = complementaryNode->availableCurrent;
    complementaryNodeSC.outputCurrent = complementaryNode->outputCurrent;
    complementaryNodeSC.currentSetPoint = complementaryNode->currentSetPoint;
    // of course, if there is no complementary node means that there is a configuration mismatch
    if (complementaryNode == &voidNode || complementaryNode->nodeNumber == own_node)
      createEvent(own_node, SUB_SYSTEM_CAN_COMMUNICATIONS, SUB_SUB_SYSTEM_GENERIC, SEVERITY_FATAL, MULTIPLE_NODE_DEFINITION);
    // All the previous lines where made to get the following value: how many current
    if (complementaryNodeSC.availableCurrent < thisNodeParams -> currentSetPoint)
    {
      remainingCurrentSetPoint = thisNodeParams -> currentSetPoint - complementaryNodeSC.availableCurrent;
    }
    else remainingCurrentSetPoint = 0.0;
  }

  /*Just check integrity*/
  if (thisNodeParams -> voltageSetPoint > thisNodeParams -> maxVoltageRequest) thisNodeParams -> voltageSetPoint = thisNodeParams -> maxVoltageRequest;
  if (remainingCurrentSetPoint < 0.0) remainingCurrentSetPoint = 0.0;
  if (remainingCurrentSetPoint > thisNodeParams -> availableCurrent) remainingCurrentSetPoint = thisNodeParams -> availableCurrent;

  powerGlobalParamsShadow.availableCurrent = 0.0;

  // Commands for switch off the converters
  if  (thisNodeParams -> voltageSetPoint < 10.0 || thisNodeParams -> command == STOP_REQUEST ||\
    thisNodeParams -> command == WAIT_REQUEST || thisNodeParams -> command == DISCONNECTION_REQUEST )
  {
    for (i = 0; i < MAXIMUM_NUMBER_OF_CONVERTERS; i++)
    {
      deltaConverter[i].commandWord = SWITCH_OFF_COMMAND | DEEP_SLEEP_COMMAND; //All converters to off state
    }
    for (i = 1; i <= (maxNumberOfConvertersWorking); i++)
    {
      //Rules for the corresponding converters:

      thisConverterB = getModuleFromPosition(i);

      if (thisConverterB != &nullConverter)
      {
        powerGlobalParamsShadow.availableCurrent =  thisConverterB -> availableCurrent;
        availableCurrentCumulative += powerGlobalParamsShadow.availableCurrent;

        thisConverterB -> voltageSetPoint = 0;

        thisConverterB -> currentSetPoint = 0;
       }
    }
  }
  else{

	    // trying to find a better currentSetPoint
#if  APLICAR_VALOR_OPTIMO
	 if ((remainingCurrentSetPoint / deltaConverter[0].availableCurrent) > 0 && (remainingCurrentSetPoint / deltaConverter[0].availableCurrent) < maxNumberOfConvertersWorking){
		  if (((int)remainingCurrentSetPoint % (int)((deltaConverter[0].availableCurrent))) < MIN_OPTIMAL_VALUE){
			  valor_optimo =  remainingCurrentSetPoint  / (int)((remainingCurrentSetPoint / deltaConverter[0].availableCurrent)+1);
		  }

	  }
#endif
	// valor_optimo = 0.0;
    for (i = 1; i <= (maxNumberOfConvertersWorking); i++)
    {
        // change the first delta to work
  	    newPosition = offsetPosition + i;
   	    if (newPosition > maxNumberOfConvertersWorking){
   	    	newPosition = newPosition - maxNumberOfConvertersWorking;
   	    }
      //Rules for the corresponding converters:

      thisConverterB = getModuleFromPosition(newPosition);

      if (thisConverterB != &nullConverter)
      {
        if ((thisConverterB -> commandWord != SWITCH_ON_COMMAND) && timerBetweenDeltaActivation > TIMER_20MS)
        {

          thisConverterB -> commandWord = SWITCH_ON_COMMAND;

          timerBetweenDeltaActivation = 0;
        }
        powerGlobalParamsShadow.availableCurrent = thisConverterB -> availableCurrent;
        availableCurrentCumulative += powerGlobalParamsShadow.availableCurrent;
        if (valor_optimo > 0.0){
            thisConverterB -> currentSetPoint = (remainingCurrentSetPoint > valor_optimo) ? valor_optimo : remainingCurrentSetPoint;
        }
        else{
            thisConverterB -> currentSetPoint = (remainingCurrentSetPoint > powerGlobalParamsShadow.availableCurrent) ? powerGlobalParamsShadow.availableCurrent : remainingCurrentSetPoint;
        }

        remainingCurrentSetPoint -= thisConverterB -> currentSetPoint;

        thisConverterB -> voltageSetPoint = thisNodeParams -> voltageSetPoint;

        thisConverterB -> powerSetPoint = 25000;

        /* Special case for high power */
 /*       if (thisNodeParams -> outputVoltage > 200.0)
        {
          // thisConverterT -> voltageSetPoint = MINIMUM((thisNodeParams -> voltageSetPoint * 0.5) , ((thisNodeParams -> outputVoltage + 10.0)* 0.5)) ;
          thisConverterB -> voltageSetPoint = MINIMUM((thisNodeParams -> voltageSetPoint) , (thisNodeParams -> outputVoltage + 10.0));
        }  */
      }
      else
      {

        thisConverterB -> commandWord = SWITCH_OFF_COMMAND | DEEP_SLEEP_COMMAND;
      }

    }
    if (timerCurrentCompensation >= TIMER_10S)
    {
      timerCurrentCompensation = 0;
#if DEBUG
      /*
      printf("Converter 1 SP: %i, Reading: %i, Avail: %i \n", (int)(deltaConverter[0].currentSetPoint*10.0), (int)(deltaConverter[0].outputCurrent*10.0), (int)(deltaConverter[0].availableCurrent*10.0)); HAL_Delay(1);
      printf("Converter 2 SP: %i, Reading: %i, Avail: %i \n", (int)(deltaConverter[1].currentSetPoint*10.0), (int)(deltaConverter[1].outputCurrent*10.0), (int)(deltaConverter[1].availableCurrent*10.0)); HAL_Delay(1);
      printf("Converter 3 SP: %i, Reading: %i, Avail: %i \n", (int)(deltaConverter[2].currentSetPoint*10.0), (int)(deltaConverter[2].outputCurrent*10.0), (int)(deltaConverter[2].availableCurrent*10.0)); HAL_Delay(1);
      printf("Converter 4 SP: %i, Reading: %i, Avail: %i \n", (int)(deltaConverter[3].currentSetPoint*10.0), (int)(deltaConverter[3].outputCurrent*10.0), (int)(deltaConverter[3].availableCurrent*10.0)); HAL_Delay(1);
      printf("Converter 5 SP: %i, Reading: %i, Avail: %i \n", (int)(deltaConverter[4].currentSetPoint*10.0), (int)(deltaConverter[4].outputCurrent*10.0), (int)(deltaConverter[4].availableCurrent*10.0)); HAL_Delay(1);
      printf("Converter 6 SP: %i, Reading: %i, Avail: %i \n", (int)(deltaConverter[5].currentSetPoint*10.0), (int)(deltaConverter[5].outputCurrent*10.0), (int)(deltaConverter[5].availableCurrent*10.0));
 */


      for (i = 0; i < (maxNumberOfConvertersWorking); i++)
       {
    	  env_debug("ID: ");
    	  env_debug_int((int)(deltaConverter[i].identifier));
    	  env_debug(" Converter Pos: ");
    	  env_debug_int((int)(deltaConverter[i].position));
          env_debug_float(" SPC:",(deltaConverter[i].currentSetPoint), 2);
          env_debug_float(" Read Cur.:",(deltaConverter[i].outputCurrent), 2);
          env_debug_float(" Av. Cur.:",(deltaConverter[i].availableCurrent), 2);
          env_debug_float(" SPV:",(deltaConverter[i].voltageSetPoint), 2);
          env_debug_float(" Read Vol.:",(deltaConverter[i].outputVoltage), 2);

          env_debug_float(" Word:",(deltaConverter[i].errorDataWord), 2);
#if SIMUL_DELTAS
          deltaConverter[i].pings = 0;
#else
          env_debug_float(" Pings:",(deltaConverter[i].pings), 2);
#endif


          env_debug(" \n");

          sum = sum + deltaConverter[i].outputCurrent;
       }


      env_debug_float("...........Total Read Cur.:",(sum), 2);
      env_debug_float("     Temp.:",Temperature, 2);

      env_debug(" \n");
      env_debug("***********************************");
      env_debug(" \n");
#endif
    }

    thisNodeParams -> availableCurrent = availableCurrentCumulative;
  }

}

void can_UpdateMsgs(void)
{
  static unsigned int lastErrorSent = 0;
  static unsigned int roundRobinLowLatency = 0, roundRobinLongLatency = 0;
  unsigned int i;

  NodeParams *thisNodeParams;
  thisNodeParams = findNodeById(own_node);
  if (thisNodeParams == &voidNode)
      createEvent(own_node, SUB_SYSTEM_CAN_COMMUNICATIONS, SUB_SUB_SYSTEM_GENERIC, SEVERITY_FATAL, NOT_MAPPED_NODE);

  if (own_node == supervisor_node + 1){
	  getConfigurationMessage(&(thisNodeParams ->followedSocketRequest), &(thisNodeParams -> configurationSetPoint), &(thisNodeParams -> powerLimit), &(idMangueraActiva));

  }

  if (timerCanLowLatency1 > TIMER_10MS)
  {
    switch (roundRobinLowLatency)
    {
      case 0:
		  if (publishLowLatencyMeasures(thisNodeParams -> outputVoltage, thisNodeParams -> availableVoltage,
				  thisNodeParams -> outputCurrent, thisNodeParams -> availableCurrent))
			  {
				timerCanLowLatency1 = 0;
				roundRobinLowLatency++;
			  }
        break;
      case 1:
		if ( publishState(thisNodeParams -> stateNumber, thisNodeParams -> currentConfiguration, thisNodeParams -> socketContactorStatus,
			   thisNodeParams -> followedSocket, thisNodeParams -> maxVoltageRequest, thisNodeParams -> maxCurrentRequest))
		{
		  timerCanLowLatency1 = 0;
		  roundRobinLowLatency++;
		}
		break;
      case 2:
		if (publishDeltaData()){
			timerCanLowLatency1 = 0;
			roundRobinLowLatency = 0;
		}

      default:
        roundRobinLowLatency = 0;
        break;
    }
    return; //If there is a transmission, there is nothing more to do
  }

  int messageSent = 0;
  //Time to send the medium latency message
  if (timerCanMediumLatency > TIMER_200MS)
  {
    for (i = 0; i < MAXIMUM_ERRORS_LISTED; i++)
    {
      lastErrorSent++;
      if (lastErrorSent >= MAXIMUM_ERRORS_LISTED) lastErrorSent = 0;
      // Publish event if and only if it belongs to the node and it is not empty
      if (eventIsVoid(&eventBuffer[lastErrorSent]) || eventBuffer[lastErrorSent].eventLocation.hardware != own_node)
      {
          //Skip, there is nothing to be sent
      }
      else
      {
        if (publishEvent(&eventBuffer[lastErrorSent]))
        {
          messageSent = 1;
          break;
        }
        else messageSent = -1;
      }
    }
    if (messageSent == 1 || messageSent == 0)
    {
      timerCanMediumLatency = 0;
      return;
    }
  }


  if (timerCanLongLatency > TIMER_1S)
  {
    switch(roundRobinLongLatency)
    {
      case 0:
        if (publishTemperatures(thisNodeParams -> temperature1, thisNodeParams -> temperature2, thisNodeParams -> temperature3, thisNodeParams -> temperature4,\
            thisNodeParams -> humidity1, thisNodeParams -> rpm1, thisNodeParams -> rpm2))
        {
          timerCanLongLatency = 0;
          roundRobinLongLatency++;
        }
        break;
      case 1:
        if ( publishActiveInductive(thisNodeParams -> midAcEnergyActive, thisNodeParams -> midAcEnergyInductive) )
        {
          timerCanLongLatency = 0;
          roundRobinLongLatency++;
        }
        break;
      case 2:
        if ( publishCapacitiveInstantaneousPower(thisNodeParams -> midAcEnergyCapacitive, thisNodeParams -> midAcPowerInstantaneous) )
        {
          timerCanLongLatency = 0;
          roundRobinLongLatency++;
        }
        break;
      case 3:
        if ( publishCurrentGlobalCurrentPh1(thisNodeParams -> midAcCurrentGlobal, thisNodeParams -> midAcCurrentPh1) )
        {
          timerCanLongLatency = 0;
          roundRobinLongLatency++;
        }
        break;
      case 4:
        if ( publishCurrentPh2CurrentPh3(thisNodeParams -> midAcCurrentPh2, thisNodeParams -> midAcCurrentPh3) )
        {
          timerCanLongLatency = 0;
          roundRobinLongLatency++;
        }
        break;
      case 5:
        if ( publishVoltageGlobalVoltagePh1(thisNodeParams -> midAcVoltageGlobal, thisNodeParams -> midAcVoltagePh1) )
        {
          timerCanLongLatency = 0;
          roundRobinLongLatency++;
        }
        break;
      case 6:
        if ( publishVoltagePh2VoltagePh3(thisNodeParams -> midAcVoltagePh2, thisNodeParams -> midAcVoltagePh3) )
        {
          timerCanLongLatency = 0;
          roundRobinLongLatency = 0;
        }
        break;
      default:
        roundRobinLongLatency = 0;
        break;
    }
  }

  if (timerCanLowLatencyDelta > TIMER_100MS)
  {
    timerCanLowLatencyDelta = 0;

  }
}

void thermalManagementDelta(SpecialCoolingCommands specialfanCommands)
{
  int i = 0;
  unsigned char commands = 0;
  float rpmMax = 0.0, period = 0.0, rpmMaxSum = 0.0;
  static CoolingLevels powerLevel;
  TIM_OC_InitTypeDef pwmConfigType = {0};

  if (onPowerSaving)
  {
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
    FAN_BOTTOM_OFF;
#if DEBUG
    if (ventilador_act == 1){
    	env_debug("desactivo ventilador por onPowerSaving\n");
    }
	ventilador_act = 0;

#endif
    powerLevel = COOLING_LEVEL_MIN;
    return; //Nothing more to do
  }
  if (timerThermalManagementDeltaExec < TIMER_5S) return;
  timerThermalManagementDeltaExec = 0;

  switch (specialfanCommands)
  {
  case AUTO_COOLING:
	/* calculation of the flow emitted by the deltas using the maximum value of the deltas   */
    for (i=0;i<maxNumberOfConvertersWorking;i++)
    {
      rpmMax = MAXIMUM(rpmMax,deltaConverter[i].rpm1);
      rpmMax = MAXIMUM(rpmMax,deltaConverter[i].rpm2);
      rpmMaxSum = rpmMaxSum + rpmMax * CAUDAL_DELTA_POR_RPM;
      commands = ( (deltaConverter[i].commandWord & SWITCH_ON_COMMAND) == SWITCH_ON_COMMAND && globalState == PRODUCTION_F1) ? 1:commands;
    }
/*
    if (rpmMax > 3000.0 && powerLevel == COOLING_LEVEL_MIN) powerLevel++;
    if (rpmMax > 5000.0 && powerLevel == COOLING_LEVEL_1) powerLevel++;
    if (rpmMax > 7000.0 && powerLevel == COOLING_LEVEL_2) powerLevel++;
    if (rpmMax > 9000.0 && powerLevel == COOLING_LEVEL_3) powerLevel++;
    if (rpmMax > 10000.0 && powerLevel == COOLING_LEVEL_4) powerLevel = COOLING_LEVEL_MAX;

    if (rpmMax < 2000.0 && powerLevel == COOLING_LEVEL_1) powerLevel = COOLING_LEVEL_MIN;
    if (rpmMax < 4000.0 && powerLevel == COOLING_LEVEL_2) powerLevel--;
    if (rpmMax < 6000.0 && powerLevel == COOLING_LEVEL_3) powerLevel--;
    if (rpmMax < 8000.0 && powerLevel == COOLING_LEVEL_4) powerLevel--;
    if (rpmMax < 9000.0 && powerLevel == COOLING_LEVEL_MAX) powerLevel--;
    */

    rpmMaxSum = rpmMaxSum * PARAMETRO_AJUSTE;
    /* Calculamos factor del porcentaje de periodo a aplicar */
    period = rpmMaxSum;
    if (period < 0.2)  period = 0.2;
    if (period > 1.0)  period = 1.0;
    break;
  case FORCE_COOLING_LEVEL_MAX:
    powerLevel = COOLING_LEVEL_MAX;
    commands = 1;
    period = 1.0;
    break;
  default:
    break;
  }

// Normal operating condition
/*
  switch (powerLevel)
  {
    case COOLING_LEVEL_MIN: period = 0.2; break;
    case COOLING_LEVEL_1: period = 0.3; break;
    case COOLING_LEVEL_2: period = 0.5; break;
    case COOLING_LEVEL_3: period = 0.6; break;
    case COOLING_LEVEL_4: period = 0.75; break;
    case COOLING_LEVEL_MAX: period = 1.0; break;
    default: period = 1.0; break;
  }
  */
  //Overwrite
//  if (specialfanCommands && FAN_FORCE_TO_MAXIMUM)

  pwmConfigType.OCMode = TIM_OCMODE_PWM1;
  pwmConfigType.Pulse = (uint32_t)( (float)htim2.Init.Period * period );//
  pwmConfigType.OCPolarity = TIM_OCPOLARITY_LOW;
  pwmConfigType.OCFastMode = TIM_OCFAST_DISABLE;

  HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim2,&pwmConfigType, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);

  HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim2,&pwmConfigType, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);



  if (commands)
  {
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
    FAN_BOTTOM_ON;
#if DEBUG
    if (ventilador_act == 0)
			env_debug("Activo ventilador \n");
	ventilador_act = 1;

#endif
  }
  else
  {
    if (powerLevel == COOLING_LEVEL_MIN)
	{
    	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
    	FAN_BOTTOM_OFF;
        #if DEBUG
    	    if (ventilador_act == 1)
    	    	env_debug("desactivo ventilador COOLING_LEVEL_MIN\n");

			ventilador_act = 0;
        #endif
	}
    else
	{
    	powerLevel--;
	}
  }
}


// Uint32 timerModbusRequest = 0;
void readPowerMeter(void)
{
  static char mbBlock = 0;
  NodeParams *thisNodeParams;
  thisNodeParams = findNodeById(own_node);
  if (thisNodeParams == &voidNode)
      createEvent(own_node, SUB_SYSTEM_CAN_COMMUNICATIONS, SUB_SUB_SYSTEM_GENERIC, SEVERITY_FATAL, NOT_MAPPED_NODE);

  if (/*mb_master.requester.isReady == true && */mb_master.state == MB_WAIT && timerModbusRequest > TIMER_2S)
  {
    timerModbusRequest = 0;
    mbBlock++;
    if (mbBlock >= 4) mbBlock = 1;
    //mbBlock = 1;
  }

  switch (mbBlock)
  {
  case 1:
    mb_master.requester.slaveAddress = 0x09;
    mb_master.requester.functionCode  = MB_FUNC_READ_HOLDINGREGISTERS;
    mb_master.requester.addr          = 0x000;
    mb_master.requester.totalData     = 12;
    mb_master.requester.generate(&mb_master);

    break;
  case 2:
    mb_master.requester.slaveAddress = 0x09;
    mb_master.requester.functionCode  = MB_FUNC_READ_HOLDINGREGISTERS;
    mb_master.requester.addr          = 0x732;
    mb_master.requester.totalData     = 18;//12;//42
    mb_master.requester.generate(&mb_master);

    break;
  case 3:
    mb_master.requester.slaveAddress = 0x09;
    mb_master.requester.functionCode  = MB_FUNC_READ_HOLDINGREGISTERS;
    mb_master.requester.addr          = 0x746;
    mb_master.requester.totalData     = 24;//12;//42
    mb_master.requester.generate(&mb_master);
    break;
  default:
    mbBlock = 1;
    break;
  }

  mb_master.loopStates(&mb_master);

#if SIMUL_METER
  if (timerDebugMeter >= (TIMER_30S))
  {
	  thisNodeParams -> midAcEnergyActive = (Uint32)(10 * (own_node));
	  thisNodeParams -> midAcEnergyInductive = (Uint32)20 + own_node;
	  thisNodeParams -> midAcEnergyCapacitive = (Uint32)30 + own_node;
	  /* Copy requested values in memory*/
	  thisNodeParams -> midAcVoltagePh1 = (Uint32)40 + own_node;
	  thisNodeParams -> midAcVoltagePh2 = (Uint32)50 + own_node;
	  thisNodeParams -> midAcVoltagePh3 = (Uint32)60 + own_node;
	  thisNodeParams -> midAcVoltageGlobal = (Uint32)70 + own_node;
	  thisNodeParams -> midAcCurrentPh1 = (Uint32)80 + own_node;
	  thisNodeParams -> midAcCurrentPh2 = (Uint32)90 + own_node;
	  thisNodeParams -> midAcCurrentPh3 = (Uint32)100 + own_node;
	  thisNodeParams -> midAcCurrentGlobal = (Uint32)110 + own_node;
	  thisNodeParams -> midAcPowerInstantaneous = (Uint32)120 + own_node;
  }



#else
  /* Copy requested values in memory*/
  thisNodeParams -> midAcEnergyActive = ((Uint32)requesterValues000[ACTIVE_ENERGY_CEM_C20_ADD+1]) + (((Uint32)requesterValues000[ACTIVE_ENERGY_CEM_C20_ADD])<<16);
  thisNodeParams -> midAcEnergyInductive = ((Uint32)requesterValues000[REACTIVE_ENERGY_Q1_CEM_C20_ADD+1]) + (((Uint32)requesterValues000[REACTIVE_ENERGY_Q1_CEM_C20_ADD])<<16);
  thisNodeParams -> midAcEnergyCapacitive = ((Uint32)requesterValues000[REACTIVE_ENERGY_Q4_CEM_C20_ADD+1]) + (((Uint32)requesterValues000[REACTIVE_ENERGY_Q4_CEM_C20_ADD])<<16);
  /* Copy requested values in memory*/
  thisNodeParams -> midAcVoltagePh1 = ((Uint32)requesterValues700[VOLTAGE_PH1_CEM_C20_ADD+1]) + (((Uint32)requesterValues700[VOLTAGE_PH1_CEM_C20_ADD])<<16);
  thisNodeParams -> midAcVoltagePh2 = ((Uint32)requesterValues700[VOLTAGE_PH2_CEM_C20_ADD+1]) + (((Uint32)requesterValues700[VOLTAGE_PH2_CEM_C20_ADD])<<16);
  thisNodeParams -> midAcVoltagePh3 = ((Uint32)requesterValues700[VOLTAGE_PH3_CEM_C20_ADD+1]) + (((Uint32)requesterValues700[VOLTAGE_PH3_CEM_C20_ADD])<<16);
  thisNodeParams -> midAcVoltageGlobal = (thisNodeParams -> midAcVoltagePh1 + thisNodeParams -> midAcVoltagePh2 + thisNodeParams -> midAcVoltagePh3)/3;
  thisNodeParams -> midAcCurrentPh1 = ((Uint32)requesterValues700[CURRENT_PH1_CEM_C20_ADD+1]) + (((Uint32)requesterValues700[CURRENT_PH1_CEM_C20_ADD])<<16);
  thisNodeParams -> midAcCurrentPh2 = ((Uint32)requesterValues700[CURRENT_PH2_CEM_C20_ADD+1]) + (((Uint32)requesterValues700[CURRENT_PH2_CEM_C20_ADD])<<16);
  thisNodeParams -> midAcCurrentPh3 = ((Uint32)requesterValues700[CURRENT_PH3_CEM_C20_ADD+1]) + (((Uint32)requesterValues700[CURRENT_PH3_CEM_C20_ADD])<<16);
  thisNodeParams -> midAcCurrentGlobal = (thisNodeParams -> midAcCurrentPh1 + thisNodeParams -> midAcCurrentPh2 + thisNodeParams -> midAcCurrentPh3)/3;
  thisNodeParams -> midAcPowerInstantaneous = ((Uint32)requesterValues700[ACTIVE_POWER_TOTAL_CEM_C20_ADD+1]) + (((Uint32)requesterValues700[ACTIVE_POWER_TOTAL_CEM_C20_ADD])<<16);


#endif


#if DEBUG
#if DEBUG_METER
  if (timerDebugMeter == -1)
  {
	  timerDebugMeter = 0;
  }

  if (timerDebugMeter >= (TIMER_30S + TIMER_5S))
  {

	  timerDebugMeter = 0;

      sprintf(bufferstr, "midAcEnergyActive:     %i \n", thisNodeParams -> midAcEnergyActive);
      env_debug(bufferstr);
      sprintf(bufferstr, "midAcEnergyInductive:  %i \n", thisNodeParams -> midAcEnergyInductive);
      env_debug(bufferstr);
      sprintf(bufferstr, "midAcEnergyCapacitive: %i \n", thisNodeParams -> midAcEnergyCapacitive);
      env_debug(bufferstr);

      sprintf(bufferstr, "midAcVoltagePh1:     %i \n", thisNodeParams -> midAcVoltagePh1);
      env_debug(bufferstr);
      sprintf(bufferstr, "midAcVoltagePh2:  %i \n", thisNodeParams -> midAcVoltagePh2);
      env_debug(bufferstr);
      sprintf(bufferstr, "midAcVoltagePh3: %i \n", thisNodeParams -> midAcVoltagePh3);
      env_debug(bufferstr);
      sprintf(bufferstr, "midAcCurrentPh1:     %i \n", thisNodeParams -> midAcCurrentPh1);
      env_debug(bufferstr);
      sprintf(bufferstr, "midAcCurrentPh2:  %i \n", thisNodeParams -> midAcCurrentPh2);
      env_debug(bufferstr);
      sprintf(bufferstr, "midAcCurrentPh3: %i \n", thisNodeParams -> midAcCurrentPh3);
      env_debug(bufferstr);

      sprintf(bufferstr, "midAcVoltageGlobal:  %i \n", thisNodeParams -> midAcVoltageGlobal);
      env_debug(bufferstr);
      sprintf(bufferstr, "midAcCurrentGlobal: %i \n", thisNodeParams -> midAcCurrentGlobal);
      env_debug(bufferstr);
      sprintf(bufferstr, "midAcPowerInstantaneous: %i\n", thisNodeParams -> midAcPowerInstantaneous);
      env_debug(bufferstr);

      env_debug("\n");



      env_debug("***********************************");
      env_debug(" \n");
  }
#endif
#endif

}

void nodeWatch(void)
{
  NodeParams *thisNodeParams;
  thisNodeParams = findNodeById(own_node);
  if (thisNodeParams == &voidNode)
      createEvent(own_node, SUB_SYSTEM_CAN_COMMUNICATIONS, SUB_SUB_SYSTEM_GENERIC, SEVERITY_FATAL, NOT_MAPPED_NODE);

  if (thisNodeParams -> lastReceivedMsg > TIME_HEARTBEAT_MS)
  {
#if	DEBUG
	   env_debug("  error node watch  !!!!!!\n");
#endif
	thisNodeParams -> configurationSetPoint = NOT_CONNECTED;
    thisNodeParams -> voltageSetPoint = 0.0;
    thisNodeParams -> currentSetPoint = 0.0;
    thisNodeParams -> command = DISCONNECTION_REQUEST;
    thisNodeParams -> followedSocketRequest = 0;
    thisNodeParams -> followedSocket = 0;
    thisNodeParams -> lastReceivedMsg = 0;
  }
}

void checkOtherEvents(EventWord listOfEvents[])
{
	int i;
	unsigned int powerSaving = 0;

	for (i = 0; i < MAXIMUM_ERRORS_LISTED; i++)
	{
		if (listOfEvents[i].eventLocation.hardware == supervisor_node &&
		  listOfEvents[i].eventLocation.subsystem == SUB_SYSTEM_CABINET &&
		  listOfEvents[i].eventLocation.subsubsystem == SUB_SUB_SYSTEM_EMCY &&
		  listOfEvents[i].eventLocation.eventCode == VCC_FAULT) powerSaving = 1;
	}

	onPowerSaving = powerSaving;

	if (onPowerSaving) thermalManagementDelta(AUTO_COOLING);
}


void manageTransmissionsNode(void)
{

  int queGet = 0;
  EventWord eventReceived;

/*
  if (own_node == supervisor_node )
      getStateMessage( &networkParams.node1.stateNumber, &networkParams.node1.currentConfiguration, &networkParams.node1.socketContactorStatus ,&networkParams.node1.followedSocket, \
                         &networkParams.node1.maxVoltageRequest, &networkParams.node1.maxCurrentRequest);
*/
  if (own_node == supervisor_node + 1 ){

	  for (queGet = 0;queGet < NUM_PUBLISH; queGet++){

		    if (mensajeDeSuper.mensaje_rec[queGet]  == 1 ){
			    mensajeDeSuper.mensaje_rec[queGet]  = 0;

			    switch(queGet){
			    case PUBLISH_EVENT:
			        getEventMessage(&eventReceived);
			        // updateLastErrorIdentifierM(&message);
			        registerEventWord(&eventReceived, eventBuffer);
			    	break;
			    }

		    }
	  }
  }


  if (timerMasterTransmission3 > TIMER_357MS)
  {
/*
	  if (mb_master.errorModbusMaster  != 0 || mb.errorModbusSlave != 0){

 		   env_debug("Error comunicacion modbus...");
           env_debug("\n");
	  }
*/


      if (publishErrorModbus(mb_master.errorModbusMaster , mb.errorModbusSlave, deltaErrorComm))
      {

        timerMasterTransmission3 = 0;
      }
  }

}


void getConfigurationMessage( __u8 *linkedNode, Output_Config *configuration, float *powerLimit, int *mang){


    if (mensajeDeSuper.mensaje_rec[PUBLISH_CONFIGURATION] == 0 ) return;

        mensajeDeSuper.mensaje_rec[PUBLISH_CONFIGURATION] = 0;

        *linkedNode = mensajeDeSuper.masterNode;
        *configuration = mensajeDeSuper.configuration;
        *powerLimit = mensajeDeSuper.powerLimit * 100.0;
        *mang       = mensajeDeSuper.manguera;

}





