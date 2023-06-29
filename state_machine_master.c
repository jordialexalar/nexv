/*
 * state_machine_master.c
 *
 *  Created on: 10 ene. 2019
 *      Author: josepmaria.fernandez
 */
#include <stdio.h>
#include <math.h>
#include "state_machine_master.h"
#include "main.h"
#include "EventList.h"

// The following lines are used only for node+master configuration
#include "specific_delta.h"
#include "ModbusSlave.h"
#include "ModbusMaster.h"

#define VALUE_FRM_ALREADY_INIT   31416
#define VALUE_HEATER_MIN  -10.0
#define VALUE_HEATER_MAX  -5.0


unsigned int input_alarmsM = 0x0;
unsigned int input_alarms = 0x0;
unsigned int input_alarms_ant = 0xff;
extern unsigned long timerVentilador;

extern int deltaLastReceivedMessage;
extern int deltaErrorComm;
extern float Temperature;
void ftoa(float n, char *res, int afterpoint);

Output_Config nodeaa = 1,nodebb = 1;
masterConfigurationStates masterConfigureNetwork = WAITING;
masterConfigurationStates masterConfigureNetwork_ant = FURTHER_CHECKS;
int dedondevengo = 0;

char *masterConfigureNetwork_str[]={"WAITING","INITIALIZE_TRANSITION","ORDER_TRANSITION1","WAIT_ACKNOWLEDGE","FURTHER_CHECKS","EMERGENCY","STANDBY_TRANSITION","SERVICE"};


mutex_var mutexPower = ALLOW_ACCESS;
NodeConfiguration nodeAconfigurationCommand;
NodeConfiguration nodeBconfigurationCommand;
unsigned long timerMasterTransmission;
unsigned long timerMasterTransmission2;
unsigned long timerMasterTransmission3;

unsigned long timerConfiguringMaster, timerHBMaster, timerCanMediumLatencyErrorsMaster;
unsigned long timerVentilador = 0;
long int timerTraza = -1;

unsigned long timerTraza2 = 0;
long timerTraza22 = -1;
long int timerStandby = -1;
long int limiteStandby = TIMER_8M;
long int timerDebugMeter = -1;


char aux[32];

void stateMachineMaster(void){


  own_node = supervisor_node;
  //First, check inputs
  //===================
  readInputsMaster();
  // mb_M.loopStates(&mb_M);.
  mb.loopStates(&mb);

  //Second, check alarms
  //====================
  updateEvents(eventBufferM);
  checkAlarmInputM();
  networkWatchMaster();

  //Third, state machine evolution
  //==============================
  updateStateMachineMaster(); //Global

  //Fourth, update outputs
  //======================
  updateOutputsMaster();
#if DEBUG
#if DEBUG_TEMP
  if (timerTraza > TIMER_2S)
      {
	  /*
      int dat = 0;
      dat  = (int) Temperature;

	  env_debug("Temperatura:");
	  env_debug_int(dat);

      */
      ftoa(Temperature,aux,1);
      env_debug(aux);
	  env_debug("\n");
      	timerTraza = 0;
      }
#endif

#if DEBUG_SHOWEVENT
  if (timerTraza == -1)
	  timerTraza = 0;

  if (timerTraza > TIMER_10S)
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
      timerTraza = 0;
      showEvent(eventBufferM);
      sprintf(bufferstr,"Indice: %i\n",(int) objectDictionary[INDEX]);

      env_debug(bufferstr);

	  env_debug("********************");
	  env_debug("\n");
      showEvent(eventBuffer);
	  env_debug("\n");
	  env_debug("\n");
	  env_debug("\n");


      }
#endif
#endif
  own_node = own_node + 1;
 /* pendiente esquema  calefactorx(); */

}


void updateStateMachineMaster(void)
{
  NodeParams *nodeA, *nodeB;
  unsigned int socketAactive = 0;
  unsigned int socketBactive = 0;
  static unsigned int socketAactiveAnt = 99;
  static unsigned int socketBactiveAnt = 99;
  static unsigned int nodeaConfigurationSetPoint = 99;
  static unsigned int nodebConfigurationSetPoint = 99;
  char sty[2];
  int salirDeStandby = 0;

  nodeA = findNodeById(networkParams.socketA.connectedToNode);
  nodeB = findNodeById(networkParams.socketB.connectedToNode);
  if(nodeA == &voidNode || nodeB == &voidNode)     Error_Handler(); //Something is really wrong -> Incorrect network map
  nodeA->powerLimit = MAX_POWER;
  nodeB->powerLimit = MAX_POWER;
  modbus_UpdateErrors();
  // modbus_UpdateValues();

  //All the required data is in memory, the state machine should be:
  //- Detect requirements
  //- Read status of the other nodes
  //- Verify availability

  //TODO: The socket state number must match

  socketAactive = (networkParams.socketA.stateNumber == 1 || networkParams.socketA.stateNumber == 2
      || networkParams.socketA.stateNumber == 3)?1:0;
  socketBactive = (networkParams.socketB.stateNumber == 1 || networkParams.socketB.stateNumber == 2
      || networkParams.socketB.stateNumber == 3)?1:0;

  if (errorCommunication > 0){
      socketAactive = 0;
      socketBactive = 0;
  }
#if STANDBY_ENABLE
  if (networkParams.socketA.stateNumber == 5 || networkParams.socketB.stateNumber == 5){
	  salirDeStandby = 1;
  }
#endif

  if (socketAactiveAnt != socketAactive){
	  socketAactiveAnt = socketAactive;
#if	DEBUG
		env_debug("SocketA:");
        if (socketAactive == 1){
		   env_debug(" activo");
        }
        else{
 		   env_debug("  no activo");
 		  if (errorCommunication > 0){
 		      socketAactive = 0;
 		      socketBactive = 0;
 		      env_debug("  por error de comunicacion");
 		  }
        }
		env_debug("\n");
#endif
  }

  if (socketBactiveAnt != socketBactive){
	  socketBactiveAnt = socketBactive;
#if	DEBUG
		env_debug("SocketB:");
        if (socketBactive == 1){
		   env_debug(" activo");
        }
        else{
 		   env_debug("  no activo");
  		  if (errorCommunication > 0){
  		      socketAactive = 0;
  		      socketBactive = 0;
  		      env_debug("  por error de comunicacion");
  		  }
        }
		env_debug("\n");
#endif
  }
  if (nodeaConfigurationSetPoint != nodeA->configurationSetPoint){
	  nodeaConfigurationSetPoint = nodeA->configurationSetPoint;
#if	DEBUG
		env_debug("Conf setpoint nodeA:");

        if (nodeA->configurationSetPoint == 0)
 		   env_debug("  NOT_CONNECTED");
         else if (nodeA->configurationSetPoint == 1)
  		   env_debug("  SOCKET");
         else if (nodeA->configurationSetPoint == 2)
             		   env_debug("  SUPPORT");
         else
   		   env_debug("   STANDBY");

         env_debug("\n");
   env_debug("nodeA followed socket:");
   sty[0] = nodeA->followedSocket + 48;
   sty[1] = 0x0;
   env_debug(sty);
   env_debug("\n");
   env_debug("networkParams.socketA.nodeNumber:");
   sty[0] = networkParams.socketA.nodeNumber+48;
   sty[1] = 0x0;
   env_debug(sty);
   env_debug("\n");
#endif
  }

  if (nodebConfigurationSetPoint != nodeB->configurationSetPoint){
	  nodebConfigurationSetPoint = nodeB->configurationSetPoint;
#if	DEBUG
		env_debug("Conf setpoint nodeB:");
        if (nodeB->configurationSetPoint == 0)
 		   env_debug("  NOT_CONNECTED");
         else if (nodeB->configurationSetPoint == 1)
  		   env_debug("  SOCKET");
         else if (nodeB->configurationSetPoint == 2)
             		   env_debug("  SUPPORT");
         else
   		   env_debug("   STANDBY");

         env_debug("\n");

   env_debug("\n");
   env_debug("nodeB followed socket:");
   sty[0] = nodeB->followedSocket+48;
   sty[1] = 0x0;
   env_debug(sty);
   env_debug("\n");
   env_debug("networkParams.socketB.nodeNumber:");
   sty[0] = networkParams.socketB.nodeNumber+48;
   sty[1] = 0x0;
   env_debug(sty);
   env_debug("\n");
#endif
  }

  if (masterConfigureNetwork != masterConfigureNetwork_ant){
	  masterConfigureNetwork_ant = masterConfigureNetwork;
 	  printf("Master state: %s\n",masterConfigureNetwork_str[masterConfigureNetwork]);
#if	DEBUG
 		env_debug("...Master state:");

 		env_debug(masterConfigureNetwork_str[masterConfigureNetwork]);
 		env_debug("\n");
#endif
   }
if (service > 0) masterConfigureNetwork = SERVICE;

  switch (masterConfigureNetwork){
  //A change is detected and starts the reconfiguration. In this first stage, the followed socket is the same
  case WAITING:

    if (socketAactive == 1 && socketBactive == 1){ // Both sockets require nodes
        if (nodeA->followedSocket != networkParams.socketA.nodeNumber){ //The node is not configured
            nodeA->configurationSetPoint = NOT_CONNECTED;
            //nodeA->followedSocket = networkParams.socketA.nodeNumber;
            //setConfiguration(nodeA->nodeNumber,  nodeA->followedSocket, nodeA->configurationSetPoint, nodeA->powerLimit);
            setConfiguration(nodeA->nodeNumber,  0, nodeA->configurationSetPoint, nodeA->powerLimit);
            masterConfigureNetwork = INITIALIZE_TRANSITION;
#if	DEBUG
    	env_debug("NOT_CONNECTED master 1...\n");
#endif

        }
        if (nodeB->followedSocket != networkParams.socketB.nodeNumber){
            nodeB->configurationSetPoint = NOT_CONNECTED;
            //nodeB->followedSocket = networkParams.socketB.nodeNumber;
            //setConfiguration(nodeB->nodeNumber, nodeB->followedSocket,  nodeB->configurationSetPoint, nodeB->powerLimit);
            setConfiguration(nodeB->nodeNumber, 0,  nodeB->configurationSetPoint, nodeB->powerLimit);
            masterConfigureNetwork = INITIALIZE_TRANSITION;
#if	DEBUG
    	env_debug("NOT_CONNECTED master 2...\n");
#endif
        }
    }
    else if (socketAactive == 1 && socketBactive == 0){ // socketA requires support
        if (nodeA->followedSocket != networkParams.socketA.nodeNumber){ //The node is not configured
            nodeA->configurationSetPoint = NOT_CONNECTED;
            //nodeA->followedSocket = networkParams.socketA.nodeNumber;
            //setConfiguration(nodeA->nodeNumber,nodeA->followedSocket , nodeA->configurationSetPoint , nodeA->powerLimit);
            setConfiguration(nodeA->nodeNumber,0 , nodeA->configurationSetPoint , nodeA->powerLimit);
            masterConfigureNetwork = INITIALIZE_TRANSITION;
            printf("Configuring output SA: yes SB: no\n");
#if	DEBUG
    	env_debug("NOT_CONNECTED master 3...\n");
#endif
            //return;
        }
        if (nodeB->followedSocket != networkParams.socketA.nodeNumber){ //Make the node B follow the socket A
            nodeB->configurationSetPoint = NOT_CONNECTED;
            //nodeB->followedSocket = networkParams.socketA.nodeNumber;
            //setConfiguration(nodeB->nodeNumber,nodeB->followedSocket , nodeB->configurationSetPoint, nodeB->powerLimit);
            setConfiguration(nodeB->nodeNumber,0 , nodeB->configurationSetPoint, nodeB->powerLimit);
            masterConfigureNetwork = INITIALIZE_TRANSITION;
#if	DEBUG
    	env_debug("NOT_CONNECTED master 4...\n");
#endif
        }
    }
    else if (socketAactive == 0 && socketBactive == 1){ // socketB requires support
        if (nodeB->followedSocket != networkParams.socketB.nodeNumber){
            nodeB->configurationSetPoint = NOT_CONNECTED;
           //nodeB->followedSocket = networkParams.socketB.nodeNumber;
            //setConfiguration(nodeB->nodeNumber, nodeB->followedSocket, nodeB->configurationSetPoint, nodeB->powerLimit);
            setConfiguration(nodeB->nodeNumber, 0, nodeB->configurationSetPoint, nodeB->powerLimit);
            masterConfigureNetwork = INITIALIZE_TRANSITION;
            //return;
#if	DEBUG
    	env_debug("NOT_CONNECTED master 5...\n");
#endif
        }
        if (nodeA->followedSocket != networkParams.socketB.nodeNumber){ //The node A is not following socket B
            nodeA->configurationSetPoint = NOT_CONNECTED;
            //nodeA->followedSocket = networkParams.socketB.nodeNumber;
            // setConfiguration(nodeA->nodeNumber,nodeA->followedSocket , nodeA->configurationSetPoint, nodeA->powerLimit);
            setConfiguration(nodeA->nodeNumber,0 , nodeA->configurationSetPoint, nodeA->powerLimit);
            masterConfigureNetwork = INITIALIZE_TRANSITION;
#if	DEBUG
    	env_debug("NOT_CONNECTED master 6...\n");
#endif
        }
    }
    else if (socketAactive == 0 && socketBactive == 0){ // All off
    	setConfiguration(nodeA->nodeNumber, 0 , nodeA->configurationSetPoint , nodeA->powerLimit);
        if (nodeA->followedSocket != 0){ //The node is not configured
            nodeA->configurationSetPoint = NOT_CONNECTED;
            //nodeA->followedSocket = 0;
            //setConfiguration(nodeA->nodeNumber, nodeA->followedSocket, nodeA->configurationSetPoint , nodeA->powerLimit);
         //   setConfiguration(nodeA->nodeNumber, 0 , nodeA->configurationSetPoint , nodeA->powerLimit);
            masterConfigureNetwork = INITIALIZE_TRANSITION;
#if	DEBUG
    	env_debug("NOT_CONNECTED master 7...\n");
#endif
        }
            setConfiguration(nodeB->nodeNumber, 0, nodeB->configurationSetPoint , nodeB->powerLimit);
        if (nodeB->followedSocket != 0){
            nodeB->configurationSetPoint = NOT_CONNECTED;
            //nodeB->followedSocket = 0;
            //setConfiguration(nodeB->nodeNumber, nodeB->followedSocket, nodeB->configurationSetPoint , nodeB->powerLimit);
  //          setConfiguration(nodeB->nodeNumber, 0, nodeB->configurationSetPoint , nodeB->powerLimit);
            masterConfigureNetwork = INITIALIZE_TRANSITION;
#if	DEBUG
    	env_debug("NOT_CONNECTED master 8...\n");
#endif
        }
        if (limiteStandby > 0){
            if (timerStandby == -1)
            	timerStandby = 0;


            if (timerStandby > limiteStandby){
            	masterConfigureNetwork = STANDBY_TRANSITION;
    		#if	DEBUG
    				env_debug("voy a StandBy");

    				env_debug("\n");
    		#endif

            	timerStandby = -1;
            }

        }
        else
        	timerStandby = -1;

    }

    timerConfiguringMaster = 0;
    break;
  case INITIALIZE_TRANSITION: //Wait until the relay transition is made
	  timerStandby = -1;
    if (nodeA -> configurationSetPoint == nodeA -> currentConfiguration &&
        nodeB -> configurationSetPoint == nodeB -> currentConfiguration &&
        timerConfiguringMaster > TIMER_100MS) //Fast way
    {
      masterConfigureNetwork = ORDER_TRANSITION;
      timerConfiguringMaster = 0;
    }
    if (timerConfiguringMaster > TIMER_2S) //Pass anyway
    {
      masterConfigureNetwork = ORDER_TRANSITION;
      timerConfiguringMaster = 0;
    }
  break;


  case ORDER_TRANSITION: //At this point, the node starts listening to the socket
	  dedondevengo = 0;
    if (socketAactive == 1 && socketBactive == 1){ // Both sockets require nodes
        if (nodeA->followedSocket != networkParams.socketA.nodeNumber){ //The node is not configured
            nodeA->configurationSetPoint = SOCKET;
            nodeA->followedSocket = networkParams.socketA.nodeNumber;
            setConfiguration(nodeA->nodeNumber,  nodeA->followedSocket, nodeA->configurationSetPoint, nodeA->powerLimit);
            masterConfigureNetwork = WAIT_ACKNOWLEDGE;
#if	DEBUG
    	env_debug("ORDER_TRANSITION 1...\n");
#endif
    	 return;
        }
        if (nodeB->followedSocket != networkParams.socketB.nodeNumber){
            nodeB->configurationSetPoint = SOCKET;
            nodeB->followedSocket = networkParams.socketB.nodeNumber;
            setConfiguration(nodeB->nodeNumber, nodeB->followedSocket,  nodeB->configurationSetPoint, nodeB->powerLimit);
            masterConfigureNetwork = WAIT_ACKNOWLEDGE;
#if	DEBUG
    	env_debug("ORDER_TRANSITION 2...\n");
#endif
    	 return;
        }
    }
    else if (socketAactive == 1 && socketBactive == 0){ // socketA requires support
        if (nodeA->followedSocket != networkParams.socketA.nodeNumber){ //The node is not configured
            nodeA->configurationSetPoint = SOCKET;
            nodeA->followedSocket = networkParams.socketA.nodeNumber;
            setConfiguration(nodeA->nodeNumber,nodeA->followedSocket , nodeA->configurationSetPoint , nodeA->powerLimit);
            masterConfigureNetwork = WAIT_ACKNOWLEDGE;
            return; //Must close socket first
        }
        if (nodeB->followedSocket != networkParams.socketA.nodeNumber){ //Make the node B follow the socket A
            nodeB->configurationSetPoint = SUPPORT;
            nodeB->followedSocket = networkParams.socketA.nodeNumber;
            setConfiguration(nodeB->nodeNumber,nodeB->followedSocket , nodeB->configurationSetPoint, nodeB->powerLimit);
            masterConfigureNetwork = WAIT_ACKNOWLEDGE;
        }
    }
    else if (socketAactive == 0 && socketBactive == 1){ // socketB requires support
        if (nodeB->followedSocket != networkParams.socketB.nodeNumber){
            nodeB->configurationSetPoint = SOCKET;
            nodeB->followedSocket = networkParams.socketB.nodeNumber;
            setConfiguration(nodeB->nodeNumber, nodeB->followedSocket, nodeB->configurationSetPoint, nodeB->powerLimit);
            masterConfigureNetwork = WAIT_ACKNOWLEDGE;
            return; //Must close socket first
        }
        if (nodeA->followedSocket != networkParams.socketB.nodeNumber){ //The node A is not following socket B
            nodeA->configurationSetPoint = SUPPORT;
            nodeA->followedSocket = networkParams.socketB.nodeNumber;
            setConfiguration(nodeA->nodeNumber,nodeA->followedSocket , nodeA->configurationSetPoint, nodeA->powerLimit);
            masterConfigureNetwork = WAIT_ACKNOWLEDGE;
        }
    }
    else if (socketAactive == 0 && socketBactive == 0){ // All off
    	dedondevengo = 1;
        // if (nodeA->followedSocket != 0){ //The node is not configured
            nodeA->configurationSetPoint = NOT_CONNECTED;
            nodeA->followedSocket = 0;
            setConfiguration(nodeA->nodeNumber, nodeA->followedSocket, nodeA->configurationSetPoint , nodeA->powerLimit);
            masterConfigureNetwork = WAIT_ACKNOWLEDGE;
#if	DEBUG
    	env_debug("NOT_CONNECTED master 9...\n");
#endif
       // }
       // if (nodeB->followedSocket != 0){
            nodeB->configurationSetPoint = NOT_CONNECTED;
            nodeB->followedSocket = 0;
            setConfiguration(nodeB->nodeNumber, nodeB->followedSocket, nodeB->configurationSetPoint , nodeB->powerLimit);
            masterConfigureNetwork = WAIT_ACKNOWLEDGE;
#if	DEBUG
    	env_debug("NOT_CONNECTED master 10...\n");
#endif
     //   }

    }
    timerConfiguringMaster = 0;
    break;
  case WAIT_ACKNOWLEDGE:
	if (dedondevengo == 0)  {
		   if (timerConfiguringMaster > TIMER_2S) masterConfigureNetwork = ORDER_TRANSITION;
		    if (nodeA -> configurationSetPoint == nodeA -> currentConfiguration && nodeB -> configurationSetPoint == nodeB -> currentConfiguration)
		      masterConfigureNetwork = ORDER_TRANSITION;
	}
	else {
		// dedondevengo = 0;
	    if (timerConfiguringMaster > TIMER_2S) masterConfigureNetwork = WAITING;
	    if (nodeA -> configurationSetPoint == nodeA -> currentConfiguration && nodeB -> configurationSetPoint == nodeB -> currentConfiguration)
	      masterConfigureNetwork = WAITING;
	}
	// dedondevengo = 0;
    break;

  case EMERGENCY:
    if (timerConfiguringMaster > TIMER_500MS)
    {
      timerConfiguringMaster = 0;
      nodeA->configurationSetPoint = NOT_CONNECTED;
      nodeA->followedSocket = 0;
      setConfiguration(nodeA->nodeNumber, nodeA->followedSocket, nodeA->configurationSetPoint , nodeA->powerLimit);

      nodeB->configurationSetPoint = NOT_CONNECTED;
      nodeB->followedSocket = 0;
      setConfiguration(nodeB->nodeNumber, nodeB->followedSocket, nodeB->configurationSetPoint , nodeB->powerLimit);

    }
    break;
  case STANDBY_TRANSITION:
	  if (socketAactive == 0 && socketBactive == 0 && salirDeStandby == 0){ // All off
			nodeA->configurationSetPoint = STANDBY;
			nodeB->configurationSetPoint = STANDBY;
			setConfiguration(nodeA->nodeNumber, 0 , nodeA->configurationSetPoint , nodeA->powerLimit);
			setConfiguration(nodeB->nodeNumber, 0, nodeB->configurationSetPoint , nodeB->powerLimit);
			masterConfigureNetwork = STANDBY_TRANSITION;
	        }
	  else{

		   nodeA->configurationSetPoint = NOT_CONNECTED;
		   nodeB->configurationSetPoint = NOT_CONNECTED;
		   salirDeStandby = 0;
#if	DEBUG
    	env_debug("NOT_CONNECTED master 12...\n");
#endif
		  masterConfigureNetwork = WAITING;
#if	DEBUG
		env_debug("voy a Waiting");

		env_debug("\n");
#endif
	    }
  break;

  case SERVICE:
       timerConfiguringMaster = 0;
       nodeA->configurationSetPoint = NOT_CONNECTED;
       nodeA->followedSocket = 0;

       nodeB->configurationSetPoint = NOT_CONNECTED;
       nodeB->followedSocket = 0;
       HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);


     break;


  default:
    masterConfigureNetwork = WAIT_ACKNOWLEDGE;
    break;
  }
}



void readInputsMaster(void){
//

    //Update Modbus (in case of being master and node at the same time)
    /*unsigned int padd ;
    unsigned int skipModbusAssignation = 0;*/

    //Move the data from the local node to the master. When master and local node are both in the same board
    //CAN communications are not read back at the time they are sent.
    /*if (own_node == networkParams.node1.nodeNumber) padd = NODE1PADDING;
    else if (own_node == networkParams.node2.nodeNumber) padd = NODE2PADDING;
    else skipModbusAssignation = 1;*/

/*    OBSOLETE while ( i < DELTA_ADDRESSES_PER_CONVERTER * MAXIMUM_NUMBER_OF_CONVERTERS && skipModbusAssignation == 0)
    {
      objectDictionary[padd + i] = deltaData2Supervisor[i];
      i++;
    }*/
}

void updateOutputsMaster (void){
    struct can_frame canFrame;

    if (timerVentilador > TIMER_20S)
    {
    	// HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
    	timerVentilador = TIMER_20S + TIMER_5S; /* para que no se desborde */
    }
    else {
       	// HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
    }

    if (timerHBMaster > 1000)
    {
      timerHBMaster = 0;
      canFrame.can_id = 0x3C0 + own_node;
      canFrame.can_dlc = 0;

      canTransmitStdMsg(&canFrame);
    }

    if (timerBlinkLed > TIMER_2S)
    {
      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
      if (timerBlinkLed > (TIMER_2S + TIMER_50MS))
      {
        timerBlinkLed = 0;
        printf(" Delivered %d \n",(int)objectDictionary[XCDV_FAMILY_AE_2_LOW]);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
      }
    }

    int i;
    static unsigned int lastErrorSentMaster = 0;
    if (timerCanMediumLatencyErrorsMaster > TIMER_200MS){
       timerCanMediumLatencyErrorsMaster = 0;
       for (i = 0; i < MAXIMUM_ERRORS_LISTED; i++){
         lastErrorSentMaster++;
         if (lastErrorSentMaster >= MAXIMUM_ERRORS_LISTED) lastErrorSentMaster = 0;
         // Publish event if and only if it belongs to the node and it is not empty
         if (eventIsVoid(&eventBufferM[lastErrorSentMaster]) || eventBufferM[lastErrorSentMaster].eventLocation.hardware != own_node)
         {
             //Skip, there is nothing to be sent
         }
         else{
           publishEvent(&eventBufferM[lastErrorSentMaster]);
           break;
         }
       }
     }

    if (timerWarmUp > TIMER_20S){  // Necesito haber leido FRAM)
    	managePowerMeasurements();
    }



    manageTransmissionsMaster();

    if (masterConfigureNetwork == EMERGENCY)
    {
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
    }
    else  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
}



void initNetwork (void){
  /* Configure this for future versions: */
  networkParams.socketA.nodeNumber = 1;
  networkParams.socketA.connectedToNode = 4;
  networkParams.socketB.nodeNumber = 2;
  networkParams.socketB.connectedToNode = 5;
  networkParams.node1.nodeNumber = 4;
  networkParams.node1.connectedToSocket = 1;
  networkParams.node1.connectedToSupport = 5;
  networkParams.node1.disabled = 0;
  networkParams.node2.nodeNumber = 5;
  networkParams.node2.connectedToSocket = 2;
  networkParams.node2.connectedToSupport = 4;
  networkParams.node2.disabled = 0;

  //Other initializations:
  clearSocketA();
  clearSocketB();
  clearNode1();
  clearNode2();
  clearEventBuffer(eventBufferM);
  clearEventBuffer(eventBuffer);
 // globalDummyEvent = createEvent(1, 2, 3, INFO, 4);
}


void clockTickNetwork(void)
{
#if	SIMUL_COMM_NODES
	  networkParams.node1.lastReceivedMsg = 0;
	  networkParams.node2.lastReceivedMsg = 0;
#endif
#if	SIMUL_COMM_METER
	  mb_master.lastReceivedMsgModbusMaster = 0;
#endif
#if	SIMUL_COMM_SOCKET
	  networkParams.socketA.lastReceivedMsg = 0;
	  networkParams.socketB.lastReceivedMsg = 0;
#endif
#if	SIMUL_DELTAS
	  deltaLastReceivedMessage = 0;
#endif
#if	SIMUL_COMM_MODBUS
	  mb.lastReceivedMsgModbusSlave = 0;
#endif
	  if (networkParams.node2.disabled == 1){
		  networkParams.node1.lastReceivedMsg = 0;
		  networkParams.node2.lastReceivedMsg = 0;
	  }
networkParams.socketA.lastReceivedMsg++;
// networkParams.socketB.lastReceivedMsg++;

networkParams.node1.lastReceivedMsg++;
networkParams.node2.lastReceivedMsg++;
mb_master.lastReceivedMsgModbusMaster++;
if (supervisor_node == 0)
	mb.lastReceivedMsgModbusSlave = 0;

mb.lastReceivedMsgModbusSlave++;

deltaLastReceivedMessage++;

  if (networkParams.socketA.nodeNumber  == idMangueraActiva){
	  networkParams.socketB.lastReceivedMsg = 0;
  }
  if (networkParams.socketB.nodeNumber  == idMangueraActiva){
	  networkParams.socketA.lastReceivedMsg = 0;
  }

}

void networkWatchNodes(void)
{

  errorCommunication = 0;

  if (networkParams.node1.lastReceivedMsg > TIME_HEARTBEAT_MS) {
 	  printf("Node 1, error heartbeat\n");
 	  clearNode1();
 	  errorCommunication = 1;
 	  networkParams.node1.errorComm = 1;
   }
   else{
 	  networkParams.node1.errorComm = 0;
   }
   if (networkParams.node2.lastReceivedMsg > TIME_HEARTBEAT_MS) {
	   if (timerWarmUp > TIMER_20S){
			printf("Node 2, error heartbeat\n");
			clearNode2();
			errorCommunication = 2;
			networkParams.node2.errorComm = 1;
	   }
	   else{
			networkParams.node2.disabled = 1;
	   }

    }
   else{
 	  networkParams.node2.errorComm = 0;
   }


  if (networkParams.socketA.lastReceivedMsg > TIME_HEARTBEAT_MS){
	 // printf("Socket A, error heartbeat\n");
	  errorCommunication = 3;
	  clearSocketA();
  }
  /*
  if (networkParams.socketB.lastReceivedMsg > TIME_HEARTBEAT_MS){
	 // printf("Socket B, error heartbeat\n");
	  errorCommunication = 4;
	  clearSocketB();
  }
*/

  if (mb_master.lastReceivedMsgModbusMaster > TIME_HEARTBEAT_MS){
	  if (mb_master.errorModbusMaster == 0)
		  printf("ModbusMaster, error heartbeat\n");
	  mb_master.errorModbusMaster = 1;
	  errorCommunication = 5;
  }
  else
  {
	  if (mb_master.errorModbusMaster == 1)
		  printf("ModbusMaster, ok\n");

	  mb_master.errorModbusMaster= 0;
  }
  if (mb.lastReceivedMsgModbusSlave > (TIME_HEARTBEAT_MS + 12)){
	  if (mb.errorModbusSlave == 0)
		  printf("ModbusSlave, error heartbeat\n");
	  mb.errorModbusSlave = 1;
	  errorCommunication = 6;
  }
  else
  {	  if (mb.errorModbusSlave == 1)
	  	  printf("ModbusSlave, OK \n");
	  mb.errorModbusSlave = 0;
  }
  if (globalState   != STANDBY_A2){
      if (deltaLastReceivedMessage > TIME_HEARTBEAT_MS){
	      if (deltaErrorComm == 0)
		      printf("Can delta, error heartbeat\n");
	      deltaErrorComm = 1;
	      errorCommunication = 7;
      }
      else{
	      if (deltaErrorComm == 1)
		      printf("Can delta, OK heartbeat\n");
	      deltaErrorComm = 0;
      }

  }
  else{
      deltaLastReceivedMessage = 0;
      deltaErrorComm = 0;
  }

}

void networkWatchMaster(void)
{
  errorCommunication = 0;
  //if (networkParams.socketA.lastReceivedMsg > TIME_HEARTBEAT_MS) clearSocketA();
  //if (networkParams.socketB.lastReceivedMsg > TIME_HEARTBEAT_MS) clearSocketB();
  if (networkParams.node1.lastReceivedMsg > TIME_HEARTBEAT_MS) {
	  printf("Node 1, error heartbeat\n");
	  clearNode1();
	  errorCommunication = 1;
	  networkParams.node1.errorComm = 1;
  }
  else{
	  networkParams.node1.errorComm = 0;
  }
  if (networkParams.node2.lastReceivedMsg > TIME_HEARTBEAT_MS) {
	  printf("Node 2, error heartbeat\n");
 	  clearNode2();
 	  errorCommunication = 2;
 	  networkParams.node2.errorComm = 1;
   }
  else{
	  networkParams.node2.errorComm = 0;
  }


    if (networkParams.socketA.lastReceivedMsg > TIME_HEARTBEAT_MS){
  	  printf("Socket A, error heartbeat\n");
  	  clearSocketA();
	  errorCommunication = 3;
	  networkParams.socketA.errorComm = 1;
    }
    else
    {
    	networkParams.socketA.errorComm = 0;
    }

    if (networkParams.socketB.lastReceivedMsg > TIME_HEARTBEAT_MS){
  	  printf("Socket B, error heartbeat\n");
  	  clearSocketB();
	  errorCommunication = 4;
	  networkParams.socketB.errorComm = 1;
   }
    else
    {
    	  networkParams.socketB.errorComm = 0;

    }


    if (mb.lastReceivedMsgModbusSlave > TIME_HEARTBEAT_MS){
	    if (mb.errorModbusSlave == 0)
		    printf("ModbusSlave, error heartbeat\n");
	    mb.errorModbusSlave = 1;
	    errorCommunication = 6;
    }
    else
    {	  if (mb.errorModbusSlave == 1)
		    printf("ModbusSlave, OK \n");
	    mb.errorModbusSlave = 0;
    }

}

void clearSocketA(void)
{

#if	DEBUG
    //	env_debug("NOT_CONNECTED master clearSocketA...\n");
#endif
  networkParams.socketA.outputVoltage = 0.0;
  networkParams.socketA.outputCurrent = 0.0;
  networkParams.socketA.stateNumber = 0;
  networkParams.socketA.command = DISCONNECTION_REQUEST;
  networkParams.socketA.maxVoltageRequest = 1000.0;
  networkParams.socketA.maxCurrentRequest = 200.0;
  networkParams.socketA.temperature1 = -120;
  networkParams.socketA.temperature2 = -120;
  networkParams.socketA.temperature3 = -120;
  networkParams.socketA.temperature4 = -120;
  networkParams.socketA.humidity1 = 0.0;
  networkParams.socketA.rpm1 = 0;
  networkParams.socketA.rpm2 = 0;
  networkParams.socketA.energyDc = 0;
  networkParams.socketA.voltageSetPoint = 0.0;
  networkParams.socketA.currentSetPoint = 0.0;
  networkParams.socketA.powerLimit = 150000.0;
  networkParams.socketA.contactorRequest = 0;
  networkParams.socketA.configurationSetPoint = NOT_CONNECTED;
  networkParams.socketA.followedNode = 0;
  //networkParams.socketA.lastReceivedMsg = 0;
}

void clearSocketB(void)
{

#if	DEBUG
    //	env_debug("NOT_CONNECTED master clearSocketB...\n");
#endif
  networkParams.socketB.outputVoltage = 0.0;
  networkParams.socketB.outputCurrent = 0.0;
  networkParams.socketB.stateNumber = 0;
  networkParams.socketB.command = DISCONNECTION_REQUEST;
  networkParams.socketB.maxVoltageRequest = 1000.0;
  networkParams.socketB.maxCurrentRequest = 200.0;
  networkParams.socketB.temperature1 = -120;
  networkParams.socketB.temperature2 = -120;
  networkParams.socketB.temperature3 = -120;
  networkParams.socketB.temperature4 = -120;
  networkParams.socketB.humidity1 = 0.0;
  networkParams.socketB.rpm1 = 0;
  networkParams.socketB.rpm2 = 0;
  networkParams.socketB.energyDc = 0;
  networkParams.socketB.voltageSetPoint = 0.0;
  networkParams.socketB.currentSetPoint = 0.0;
  networkParams.socketB.powerLimit = 150000.0;
  networkParams.socketB.contactorRequest = 0;
  networkParams.socketB.configurationSetPoint = NOT_CONNECTED;
  networkParams.socketB.followedNode = 0;
  //networkParams.socketB.lastReceivedMsg = 0;
}

void clearNode1(void)
{
#if	DEBUG
    	//env_debug("NOT_CONNECTED master clearNode1...\n");
#endif
  networkParams.node1.outputVoltage = 0.0;
  networkParams.node1.outputCurrent = 0.0;
  networkParams.node1.availableVoltage = 1000.0;
  networkParams.node1.availableCurrent = 246.0;
  networkParams.node1.stateNumber = 0;
  networkParams.node1.currentConfiguration = NOT_CONNECTED;
  networkParams.node1.command = DISCONNECTION_REQUEST;
  networkParams.node1.maxVoltageRequest = 0.0;
  networkParams.node1.maxCurrentRequest = 0.0;
  networkParams.node1.temperature1 = -120;
  networkParams.node1.temperature2 = -120;
  networkParams.node1.temperature3 = -120;
  networkParams.node1.temperature4 = -120;
  networkParams.node1.humidity1 = 0.0;
  networkParams.node1.rpm1 = 0;
  networkParams.node1.rpm2 = 0;

  networkParams.node1.midAcEnergyActive = 0;
  networkParams.node1.midAcEnergyInductive = 0;
  networkParams.node1.midAcEnergyCapacitive = 0;
  networkParams.node1.midAcPowerInstantaneous = 0;
  networkParams.node1.midAcCurrentGlobal = 0;
  networkParams.node1.midAcCurrentPh1 = 0;
  networkParams.node1.midAcCurrentPh2 = 0;
  networkParams.node1.midAcCurrentPh3 = 0;
  networkParams.node1.midAcVoltageGlobal = 0;
  networkParams.node1.midAcVoltagePh1 = 0;
  networkParams.node1.midAcVoltagePh2 = 0;
  networkParams.node1.midAcVoltagePh3 = 0;

  networkParams.node1.voltageSetPoint = 0.0;
  networkParams.node1.currentSetPoint = 0.0;
  networkParams.node1.powerLimit = 0.0; //
  networkParams.node1.configurationSetPoint = NOT_CONNECTED;
  networkParams.node1.socketContactorStatus = 0 ;
  networkParams.node1.socketContactorRequest = 0;
  networkParams.node1.bridgeContactorStatus = 0;
  networkParams.node1.bridgeContactorRequest = 0;
  networkParams.node1.followedSocket = 0;
  //networkParams.node1.lastReceivedMsg = 0;

  setConfiguration(networkParams.node1.nodeNumber, networkParams.node1.followedSocket, networkParams.node1.configurationSetPoint, 12500.0);
}

void clearNode2(void)
{

#if	DEBUG
    	//env_debug("NOT_CONNECTED master clearNode2...\n");
#endif
  networkParams.node2.outputVoltage = 0.0;
  networkParams.node2.outputCurrent = 0.0;
  networkParams.node2.availableVoltage = 1000.0;
  networkParams.node2.availableCurrent = 246.0;
  networkParams.node2.stateNumber = 0;
  networkParams.node2.currentConfiguration = NOT_CONNECTED;
  networkParams.node2.command = DISCONNECTION_REQUEST;
  networkParams.node2.maxVoltageRequest = 0.0;
  networkParams.node2.maxCurrentRequest = 0.0;
  networkParams.node2.temperature1 = -120;
  networkParams.node2.temperature2 = -120;
  networkParams.node2.temperature3 = -120;
  networkParams.node2.temperature4 = -120;
  networkParams.node2.humidity1 = 0.0;
  networkParams.node2.rpm1 = 0;
  networkParams.node2.rpm2 = 0;

  networkParams.node2.midAcEnergyActive = 0;
  networkParams.node2.midAcEnergyInductive = 0;
  networkParams.node2.midAcEnergyCapacitive = 0;
  networkParams.node2.midAcPowerInstantaneous = 0;
  networkParams.node2.midAcCurrentGlobal = 0;
  networkParams.node2.midAcCurrentPh1 = 0;
  networkParams.node2.midAcCurrentPh2 = 0;
  networkParams.node2.midAcCurrentPh3 = 0;
  networkParams.node2.midAcVoltageGlobal = 0;
  networkParams.node2.midAcVoltagePh1 = 0;
  networkParams.node2.midAcVoltagePh2 = 0;
  networkParams.node2.midAcVoltagePh3 = 0;

  networkParams.node2.voltageSetPoint = 0.0;
  networkParams.node2.currentSetPoint = 0.0;
  networkParams.node2.powerLimit = 0.0; //
  networkParams.node2.configurationSetPoint = NOT_CONNECTED;
  networkParams.node2.socketContactorStatus = 0 ;
  networkParams.node2.socketContactorRequest = 0;
  networkParams.node2.bridgeContactorStatus = 0;
  networkParams.node2.bridgeContactorRequest = 0;
  networkParams.node2.followedSocket = 0;
  //networkParams.node2.lastReceivedMsg = 0;

  setConfiguration(networkParams.node2.nodeNumber, networkParams.node2.followedSocket, networkParams.node2.configurationSetPoint, 12500.0);
}

void checkAlarmInputM(void)
{
	/*
#define SURGE_PROTECTION  0
#define CLICKSON  1
#define TAMPER  2
#define HEATER_AND_FANS 3
#define VCC_FAULT 4
#define DISPENSER_EMCY  5
	*/
input_alarmsM = 0x0;
int networkOnFault = 0;
/*
if (ALARM1_ON){
  createEvent(own_node, SUB_SYSTEM_CABINET, SUB_SUB_SYSTEM_EMCY, SEVERITY_ERROR, DELTAS);
  input_alarmsM = input_alarmsM | 0x04;
}
*/
int i = 0;
  for (i = 0; i < MAXIMUM_ERRORS_LISTED; i++)
  {
//    if ((eventBufferM[i].eventLocation.hardware == own_node && eventBufferM[i].eventLocation.severity == SEVERITY_ERROR) ||
     if ((eventBufferM[i].eventLocation.severity == SEVERITY_ERROR) ||
      eventBufferM[i].eventLocation.severity == SEVERITY_FATAL)
      {
        networkOnFault = 1;
        if (eventBufferM[i].eventLocation.subsystem == 0 || eventBufferM[i].eventLocation.subsubsystem == 0){
        	input_alarmsM = input_alarmsM | eventBufferM[i].eventLocation.eventCode;
        }
      }
    if (networkOnFault) masterConfigureNetwork = EMERGENCY;
    else if (masterConfigureNetwork == EMERGENCY && networkOnFault == 0) masterConfigureNetwork = WAITING;
  }
}


void managePowerMeasurements(void)
{
  unsigned long midAcEnergyActiveS1;
  unsigned long midAcEnergyActiveS2;
  unsigned long midAcPowerInstantaneousS1;
  unsigned long midAcPowerInstantaneousS2;
  unsigned long midAcCurrentGlobalS1;
  unsigned long midAcCurrentGlobalS2;
  unsigned long midAcCurrentPh1S1;
  unsigned long midAcCurrentPh1S2;
  unsigned long midAcCurrentPh2S1;
  unsigned long midAcCurrentPh2S2;
  unsigned long midAcCurrentPh3S1;
  unsigned long midAcCurrentPh3S2;
  if (mutexPower == PROTECTED) return;
  else mutexPower = PROTECTED;
  midAcEnergyActiveS1 = networkParams.node1.midAcEnergyActive;
  midAcEnergyActiveS2 = networkParams.node2.midAcEnergyActive;
  midAcPowerInstantaneousS1 = networkParams.node1.midAcPowerInstantaneous;
  midAcPowerInstantaneousS2 = networkParams.node2.midAcPowerInstantaneous;
  midAcCurrentGlobalS1 = networkParams.node1.midAcCurrentGlobal;
  midAcCurrentGlobalS2 = networkParams.node2.midAcCurrentGlobal;
  midAcCurrentPh1S1 =  networkParams.node1.midAcCurrentPh1;
  midAcCurrentPh1S2 =  networkParams.node2.midAcCurrentPh1;
  midAcCurrentPh2S1 =  networkParams.node1.midAcCurrentPh2;
  midAcCurrentPh2S2 =  networkParams.node2.midAcCurrentPh2;
  midAcCurrentPh3S1 =  networkParams.node1.midAcCurrentPh3;
  midAcCurrentPh3S2 =  networkParams.node2.midAcCurrentPh3;
//Independent mode
  objectDictionary[XCDV_FAMILY_AE_1_HIGH] =   networkParams.node1.midAcEnergyActive>>16;
  objectDictionary[XCDV_FAMILY_AE_1_LOW] =    networkParams.node1.midAcEnergyActive & 0xFFFF;
  objectDictionary[XCDV_FAMILY_IE_1_HIGH] =   networkParams.node1.midAcEnergyInductive>>16;
  objectDictionary[XCDV_FAMILY_IE_1_LOW] =    networkParams.node1.midAcEnergyInductive & 0xFFFF;
  objectDictionary[XCDV_FAMILY_CE_1_HIGH] =   networkParams.node1.midAcEnergyCapacitive >>16;
  objectDictionary[XCDV_FAMILY_CE_1_LOW] =    networkParams.node1.midAcEnergyCapacitive & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AP_1_HIGH] =   networkParams.node1.midAcPowerInstantaneous >> 16;
  objectDictionary[XCDV_FAMILY_AP_1_LOW] =    networkParams.node1.midAcPowerInstantaneous & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AI_1_HIGH] =   networkParams.node1.midAcCurrentGlobal >> 16;
  objectDictionary[XCDV_FAMILY_AI_1_LOW] =    networkParams.node1.midAcCurrentGlobal & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AI1_1_HIGH] =  networkParams.node1.midAcCurrentPh1 >> 16;
  objectDictionary[XCDV_FAMILY_AI1_1_LOW] =   networkParams.node1.midAcCurrentPh1 & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AI2_1_HIGH] =  networkParams.node1.midAcCurrentPh2 >> 16;
  objectDictionary[XCDV_FAMILY_AI2_1_LOW] =   networkParams.node1.midAcCurrentPh2 & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AI3_1_HIGH] =  networkParams.node1.midAcCurrentPh3 >> 16;
  objectDictionary[XCDV_FAMILY_AI3_1_LOW] =   networkParams.node1.midAcCurrentPh3 & 0xFFFF;
  objectDictionary[XCDV_FAMILY_VI_1_HIGH] =   networkParams.node1.midAcVoltageGlobal >> 16;
  objectDictionary[XCDV_FAMILY_VI_1_LOW ] =   networkParams.node1.midAcVoltageGlobal & 0xFFFF;
  objectDictionary[XCDV_FAMILY_VI1_1_HIGH] =  networkParams.node1.midAcVoltagePh1 >> 16;
  objectDictionary[XCDV_FAMILY_VI1_1_LOW] =   networkParams.node1.midAcVoltagePh1 & 0xFFFF;
  objectDictionary[XCDV_FAMILY_VI2_1_HIGH] =  networkParams.node1.midAcVoltagePh2 >> 16;
  objectDictionary[XCDV_FAMILY_VI2_1_LOW] =   networkParams.node1.midAcVoltagePh2 & 0xFFFF;
  objectDictionary[XCDV_FAMILY_VI3_1_HIGH] =  networkParams.node1.midAcVoltagePh3 >> 16;
  objectDictionary[XCDV_FAMILY_VI3_1_LOW] =   networkParams.node1.midAcVoltagePh3 & 0xFFFF;

  objectDictionary[XCDV_FAMILY_AE_2_HIGH] =   networkParams.node2.midAcEnergyActive>>16;
  objectDictionary[XCDV_FAMILY_AE_2_LOW] =    networkParams.node2.midAcEnergyActive & 0xFFFF;
  objectDictionary[XCDV_FAMILY_IE_2_HIGH] =   networkParams.node2.midAcEnergyInductive>>16;
  objectDictionary[XCDV_FAMILY_IE_2_LOW] =    networkParams.node2.midAcEnergyInductive & 0xFFFF;
  objectDictionary[XCDV_FAMILY_CE_2_HIGH] =   networkParams.node2.midAcEnergyCapacitive >>16;
  objectDictionary[XCDV_FAMILY_CE_2_LOW] =    networkParams.node2.midAcEnergyCapacitive & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AP_2_HIGH] =   networkParams.node2.midAcPowerInstantaneous >> 16;
  objectDictionary[XCDV_FAMILY_AP_2_LOW] =    networkParams.node2.midAcPowerInstantaneous & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AI_2_HIGH] =   networkParams.node2.midAcCurrentGlobal >>16;
  objectDictionary[XCDV_FAMILY_AI_2_LOW] =    networkParams.node2.midAcCurrentGlobal & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AI1_2_HIGH] =  networkParams.node2.midAcCurrentPh1 >> 16;
  objectDictionary[XCDV_FAMILY_AI1_2_LOW] =   networkParams.node2.midAcCurrentPh1 & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AI2_2_HIGH] =  networkParams.node2.midAcCurrentPh2 >> 16;
  objectDictionary[XCDV_FAMILY_AI2_2_LOW] =   networkParams.node2.midAcCurrentPh2 & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AI3_2_HIGH] =  networkParams.node2.midAcCurrentPh3 >> 16;
  objectDictionary[XCDV_FAMILY_AI3_2_LOW] =   networkParams.node2.midAcCurrentPh3 & 0xFFFF;
  objectDictionary[XCDV_FAMILY_VI_2_HIGH] =   networkParams.node2.midAcVoltageGlobal >> 16;
  objectDictionary[XCDV_FAMILY_VI_2_LOW ] =   networkParams.node2.midAcVoltageGlobal & 0xFFFF;
  objectDictionary[XCDV_FAMILY_VI1_2_HIGH] =  networkParams.node2.midAcVoltagePh1 >> 16;
  objectDictionary[XCDV_FAMILY_VI1_2_LOW] =   networkParams.node2.midAcVoltagePh1 & 0xFFFF;
  objectDictionary[XCDV_FAMILY_VI2_2_HIGH] =  networkParams.node2.midAcVoltagePh2 >> 16;
  objectDictionary[XCDV_FAMILY_VI2_2_LOW] =   networkParams.node2.midAcVoltagePh2 & 0xFFFF;
  objectDictionary[XCDV_FAMILY_VI3_2_HIGH] =  networkParams.node2.midAcVoltagePh3 >> 16;
  objectDictionary[XCDV_FAMILY_VI3_2_LOW] =   networkParams.node2.midAcVoltagePh3 & 0xFFFF;
  mutexPower = ALLOW_ACCESS;
//Combined mode
/*  objectDictionary[XCDV_FAMILY_AE_1_HIGH] = (midAcEnergyActiveS1 + midAcEnergyActiveS2)>>16;
  objectDictionary[XCDV_FAMILY_AE_1_LOW] =    (midAcEnergyActiveS1 + midAcEnergyActiveS2) & 0xFFFF;
  objectDictionary[XCDV_FAMILY_IE_1_HIGH] =   (networkParams.node1.midAcEnergyInductive + networkParams.node2.midAcEnergyInductive) >>16;
  objectDictionary[XCDV_FAMILY_IE_1_LOW] =    (networkParams.node1.midAcEnergyInductive + networkParams.node2.midAcEnergyInductive) & 0xFFFF;
  objectDictionary[XCDV_FAMILY_CE_1_HIGH] =   (networkParams.node1.midAcEnergyCapacitive + networkParams.node2.midAcEnergyCapacitive) >>16;
  objectDictionary[XCDV_FAMILY_CE_1_LOW] =    (networkParams.node1.midAcEnergyCapacitive + networkParams.node2.midAcEnergyCapacitive) & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AP_1_HIGH] =   (networkParams.node1.midAcPowerInstantaneous + networkParams.node2.midAcPowerInstantaneous) >> 16;
  objectDictionary[XCDV_FAMILY_AP_1_LOW] =    (networkParams.node1.midAcPowerInstantaneous + networkParams.node2.midAcPowerInstantaneous) & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AI_1_HIGH] =   (networkParams.node1.midAcCurrentGlobal + networkParams.node2.midAcCurrentGlobal) >> 16;
  objectDictionary[XCDV_FAMILY_AI_1_LOW] =    (networkParams.node1.midAcCurrentGlobal + networkParams.node2.midAcCurrentGlobal) & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AI1_1_HIGH] =  (networkParams.node1.midAcCurrentPh1 + networkParams.node2.midAcCurrentPh1)>> 16;
  objectDictionary[XCDV_FAMILY_AI1_1_LOW] =   (networkParams.node1.midAcCurrentPh1 + networkParams.node2.midAcCurrentPh1) & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AI2_1_HIGH] =  (networkParams.node1.midAcCurrentPh2 + networkParams.node2.midAcCurrentPh2) >> 16;
  objectDictionary[XCDV_FAMILY_AI2_1_LOW] =   (networkParams.node1.midAcCurrentPh2 + networkParams.node2.midAcCurrentPh2) & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AI3_1_HIGH] =  (networkParams.node1.midAcCurrentPh3 + networkParams.node2.midAcCurrentPh3) >> 16;
  objectDictionary[XCDV_FAMILY_AI3_1_LOW] =   (networkParams.node1.midAcCurrentPh3 + networkParams.node2.midAcCurrentPh3) & 0xFFFF;
  objectDictionary[XCDV_FAMILY_VI_1_HIGH] =   ((networkParams.node1.midAcVoltageGlobal + networkParams.node2.midAcVoltageGlobal) >> 17);//div 2
  objectDictionary[XCDV_FAMILY_VI_1_LOW ] =   ((networkParams.node1.midAcVoltageGlobal + networkParams.node2.midAcVoltageGlobal) >> 1) & 0xFFFF;//div 2
  objectDictionary[XCDV_FAMILY_VI1_1_HIGH] =  ((networkParams.node1.midAcVoltagePh1 + networkParams.node2.midAcVoltagePh1) >> 17);//div 2
  objectDictionary[XCDV_FAMILY_VI1_1_LOW] =   ((networkParams.node1.midAcVoltagePh1 + networkParams.node2.midAcVoltagePh1) >> 1) & 0xFFFF;//div 2
  objectDictionary[XCDV_FAMILY_VI2_1_HIGH] =  ((networkParams.node1.midAcVoltagePh2 + networkParams.node2.midAcVoltagePh2) >> 17);//div 2
  objectDictionary[XCDV_FAMILY_VI2_1_LOW] =   ((networkParams.node1.midAcVoltagePh2 + networkParams.node2.midAcVoltagePh2) >> 1) & 0xFFFF;//div 2
  objectDictionary[XCDV_FAMILY_VI3_1_HIGH] =  ((networkParams.node1.midAcVoltagePh3 + networkParams.node2.midAcVoltagePh3) >> 17);//div 2
  objectDictionary[XCDV_FAMILY_VI3_1_LOW] =   ((networkParams.node1.midAcVoltagePh3 + networkParams.node2.midAcVoltagePh3) >> 1) & 0xFFFF;//div 2

  objectDictionary[XCDV_FAMILY_AE_2_HIGH] =   (midAcEnergyActiveS1 + midAcEnergyActiveS2)>>16;
  objectDictionary[XCDV_FAMILY_AE_2_LOW] =    (midAcEnergyActiveS1 + midAcEnergyActiveS2) & 0xFFFF;
  objectDictionary[XCDV_FAMILY_IE_2_HIGH] =   (networkParams.node1.midAcEnergyInductive + networkParams.node2.midAcEnergyInductive) >>16;
  objectDictionary[XCDV_FAMILY_IE_2_LOW] =    (networkParams.node1.midAcEnergyInductive + networkParams.node2.midAcEnergyInductive) & 0xFFFF;
  objectDictionary[XCDV_FAMILY_CE_2_HIGH] =   (networkParams.node1.midAcEnergyCapacitive + networkParams.node2.midAcEnergyCapacitive) >>16;
  objectDictionary[XCDV_FAMILY_CE_2_LOW] =    (networkParams.node1.midAcEnergyCapacitive + networkParams.node2.midAcEnergyCapacitive) & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AP_2_HIGH] =   (networkParams.node1.midAcPowerInstantaneous + networkParams.node2.midAcPowerInstantaneous) >> 16;
  objectDictionary[XCDV_FAMILY_AP_2_LOW] =    (networkParams.node1.midAcPowerInstantaneous + networkParams.node2.midAcPowerInstantaneous) & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AI_2_HIGH] =   (networkParams.node1.midAcCurrentGlobal + networkParams.node2.midAcCurrentGlobal) >> 16;
  objectDictionary[XCDV_FAMILY_AI_2_LOW] =    (networkParams.node1.midAcCurrentGlobal + networkParams.node2.midAcCurrentGlobal) & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AI1_2_HIGH] =  (networkParams.node1.midAcCurrentPh1 + networkParams.node2.midAcCurrentPh1)>> 16;
  objectDictionary[XCDV_FAMILY_AI1_2_LOW] =   (networkParams.node1.midAcCurrentPh1 + networkParams.node2.midAcCurrentPh1) & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AI2_2_HIGH] =  (networkParams.node1.midAcCurrentPh2 + networkParams.node2.midAcCurrentPh2) >> 16;
  objectDictionary[XCDV_FAMILY_AI2_2_LOW] =   (networkParams.node1.midAcCurrentPh2 + networkParams.node2.midAcCurrentPh2) & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AI3_2_HIGH] =  (networkParams.node1.midAcCurrentPh3 + networkParams.node2.midAcCurrentPh3) >> 16;
  objectDictionary[XCDV_FAMILY_AI3_2_LOW] =   (networkParams.node1.midAcCurrentPh3 + networkParams.node2.midAcCurrentPh3) & 0xFFFF;
  objectDictionary[XCDV_FAMILY_VI_2_HIGH] =   ((networkParams.node1.midAcVoltageGlobal + networkParams.node2.midAcVoltageGlobal) >> 17);//div 2
  objectDictionary[XCDV_FAMILY_VI_2_LOW ] =   ((networkParams.node1.midAcVoltageGlobal + networkParams.node2.midAcVoltageGlobal) >> 1) & 0xFFFF;//div 2
  objectDictionary[XCDV_FAMILY_VI1_2_HIGH] =  ((networkParams.node1.midAcVoltagePh1 + networkParams.node2.midAcVoltagePh1) >> 17);//div 2
  objectDictionary[XCDV_FAMILY_VI1_2_LOW] =   ((networkParams.node1.midAcVoltagePh1 + networkParams.node2.midAcVoltagePh1) >> 1) & 0xFFFF;//div 2
  objectDictionary[XCDV_FAMILY_VI2_2_HIGH] =  ((networkParams.node1.midAcVoltagePh2 + networkParams.node2.midAcVoltagePh2) >> 17);//div 2
  objectDictionary[XCDV_FAMILY_VI2_2_LOW] =   ((networkParams.node1.midAcVoltagePh2 + networkParams.node2.midAcVoltagePh2) >> 1) & 0xFFFF;//div 2
  objectDictionary[XCDV_FAMILY_VI3_2_HIGH] =  ((networkParams.node1.midAcVoltagePh3 + networkParams.node2.midAcVoltagePh3) >> 17);//div 2
  objectDictionary[XCDV_FAMILY_VI3_2_LOW] =   ((networkParams.node1.midAcVoltagePh3 + networkParams.node2.midAcVoltagePh3) >> 1) & 0xFFFF;//div 2
*/
  //Experimental setup:

  static unsigned long  energyS1 = 0,  energyS1last = 0, energyS2 = 0,  energyS2last = 0, energyS1Acu = 0,  energyS2Acu = 0;
  static unsigned long  powerInstantaneousS1 = 0,  powerInstantaneousS1last = 0, powerInstantaneousS2 = 0,  powerInstantaneousS2last = 0;
  static unsigned long  currentGlobalS1 = 0,  currentGlobalS1last = 0, currentGlobalS2 = 0,  currentGlobalS2last = 0;
  static unsigned long  currentPh1S1 = 0,  currentPh1S1last = 0, currentPh1S2 = 0,  currentPh1S2last = 0;
  static unsigned long  currentPh2S1 = 0,  currentPh2S1last = 0, currentPh2S2 = 0,  currentPh2S2last = 0;
  static unsigned long  currentPh3S1 = 0,  currentPh3S1last = 0, currentPh3S2 = 0,  currentPh3S2last = 0;

  energyS1 = 0;
  energyS2 = 0;
  powerInstantaneousS1 = 0;
  powerInstantaneousS2 = 0;
  currentGlobalS1 = 0;
  currentGlobalS2 = 0;
  currentPh1S1 = 0;
  currentPh1S2 = 0;
  currentPh2S1 = 0;
  currentPh2S2 = 0;
  currentPh3S1 = 0;
  currentPh3S2 = 0;


  if (((Uint32) objectDictionary[XCDV_FAMILY_AE_TOTAL_SOCKET_INIT]) != VALUE_FRM_ALREADY_INIT){
	  energyS1Acu = 0;
	  energyS2Acu = 0;
	  //      energyS1Acu = midAcEnergyActiveS1;
	  //      energyS2Acu = midAcEnergyActiveS2;
#if DEBUG_ENERGIA
	  env_debug("Inicio contadores energia...");
	  env_debug("\n");
#endif

  }
  else{
	  energyS1Acu = (((Uint32) objectDictionary[XCDV_FAMILY_AE_TOTAL_SOCKET_A_HIGH]) <<16) + (Uint32)objectDictionary[XCDV_FAMILY_AE_TOTAL_SOCKET_A_LOW];
	  energyS2Acu = (((Uint32) objectDictionary[XCDV_FAMILY_AE_TOTAL_SOCKET_B_HIGH]) <<16) + (Uint32)objectDictionary[XCDV_FAMILY_AE_TOTAL_SOCKET_B_LOW];
  }
  if (networkParams.node1.currentConfiguration == NOT_CONNECTED && networkParams.node2.currentConfiguration == NOT_CONNECTED)
  {/*
	   energyS1 = midAcEnergyActiveS1;
       energyS2 = midAcEnergyActiveS2;
       powerInstantaneousS1 = midAcPowerInstantaneousS1;
       powerInstantaneousS2 = midAcPowerInstantaneousS2;
       currentGlobalS1 = midAcCurrentGlobalS1;
       currentGlobalS2 = midAcCurrentGlobalS2;
       currentPh1S1 = midAcCurrentPh1S1;
       currentPh1S2 = midAcCurrentPh1S2;
       currentPh2S1 = midAcCurrentPh2S1;
       currentPh2S2 = midAcCurrentPh2S2;
       currentPh3S1 = midAcCurrentPh3S1;
       currentPh3S2 = midAcCurrentPh3S2;
   */
	  energyS1 = 0;
	  energyS2 = 0;
	  powerInstantaneousS1 = 0;
	  powerInstantaneousS2 = 0;
	  currentGlobalS1 = 0;
	  currentGlobalS2 = 0;
	  currentPh1S1 = 0;
	  currentPh1S2 = 0;
	  currentPh2S1 = 0;
	  currentPh2S2 = 0;
	  currentPh3S1 = 0;
	  currentPh3S2 = 0;
  }


  // Depending on the assignation, energy will be integrated in one or other counter
 if (networkParams.node1.currentConfiguration == SOCKET){
	 energyS1 += (midAcEnergyActiveS1 - energyS1last);
	 powerInstantaneousS1 += (midAcPowerInstantaneousS1 - powerInstantaneousS1last);
	 currentGlobalS1 += (midAcCurrentGlobalS1 - currentGlobalS1last);
     currentPh1S1 += (midAcCurrentPh1S1 - currentPh1S1last);
     currentPh2S1 += (midAcCurrentPh2S1 - currentPh2S1last);
     currentPh3S1 += (midAcCurrentPh3S1 - currentPh3S1last);

 }


 if (networkParams.node1.currentConfiguration == SUPPORT){
	 energyS2 += (midAcEnergyActiveS1 - energyS1last);
	 powerInstantaneousS2 += (midAcPowerInstantaneousS1 - powerInstantaneousS1last);
	 currentGlobalS2 += (midAcCurrentGlobalS1 - currentGlobalS1last);
     currentPh1S2 += (midAcCurrentPh1S1 - currentPh1S1last);
     currentPh2S2 += (midAcCurrentPh2S1 - currentPh2S1last);
     currentPh3S2 += (midAcCurrentPh3S1 - currentPh3S1last);
 }


 if (networkParams.node2.currentConfiguration == SOCKET){
	 energyS2 += (midAcEnergyActiveS2 - energyS2last);
	 powerInstantaneousS2 += (midAcPowerInstantaneousS2 - powerInstantaneousS2last);
	 currentGlobalS2 += (midAcCurrentGlobalS2 - currentGlobalS2last);
     currentPh1S2 += (midAcCurrentPh1S2 - currentPh1S2last);
     currentPh2S2 += (midAcCurrentPh2S2 - currentPh2S2last);
     currentPh3S2 += (midAcCurrentPh3S2 - currentPh3S2last);
 }


 if (networkParams.node2.currentConfiguration == SUPPORT){
	 energyS1 += (midAcEnergyActiveS2 - energyS2last);
	 powerInstantaneousS1 += (midAcPowerInstantaneousS2 - powerInstantaneousS2last);
	 currentGlobalS1 += (midAcCurrentGlobalS2 - currentGlobalS2last);
     currentPh1S1 += (midAcCurrentPh1S2 - currentPh1S2last);
     currentPh2S1 += (midAcCurrentPh2S2 - currentPh2S2last);
     currentPh3S1 += (midAcCurrentPh3S2 - currentPh3S2last);
 }


 energyS1last = midAcEnergyActiveS1;
 energyS2last = midAcEnergyActiveS2;

 // powerInstantaneousS1last = midAcPowerInstantaneousS1;
 // powerInstantaneousS2last = midAcPowerInstantaneousS2;

 powerInstantaneousS1last = 0;
 powerInstantaneousS2last = 0;
/*
 currentGlobalS1last = midAcCurrentGlobalS1;
 currentGlobalS2last = midAcCurrentGlobalS2;

 currentPh1S1last = midAcCurrentPh1S1;
 currentPh1S2last = midAcCurrentPh1S2;
 currentPh2S1last = midAcCurrentPh2S1;
 currentPh2S2last = midAcCurrentPh2S2;
 currentPh3S1last = midAcCurrentPh3S1;
 currentPh3S2last = midAcCurrentPh3S2;
*/

 currentGlobalS1last = 0;
 currentGlobalS2last = 0;

 currentPh1S1last = 0;
 currentPh1S2last = 0;
 currentPh2S1last = 0;
 currentPh2S2last = 0;
 currentPh3S1last = 0;
 currentPh3S2last = 0;

  //TODO: What will happen with simultaneous charge?

  // energyS1 = midAcEnergyActiveS1 + midAcEnergyActiveS2;
  // energyS2 = midAcEnergyActiveS1 + midAcEnergyActiveS2;

  energyS1Acu += energyS1;
  energyS2Acu += energyS2;


/*
  energyS1Acu = 4500;
  energyS2Acu = 5500;

  powerInstantaneousS1 =1010;
  powerInstantaneousS2 =2010;
  currentGlobalS1 = 1020;
  currentGlobalS2 = 2020;

  currentPh1S1 = 1030;
  currentPh1S2 = 2030;
  currentPh2S1 = 1040;
  currentPh2S2 = 2040;
  currentPh3S1 = 1050;
  currentPh3S2 = 2050;
*/
  objectDictionary[XCDV_FAMILY_AE_TOTAL_SOCKET_INIT]   =   VALUE_FRM_ALREADY_INIT;
  objectDictionary[XCDV_FAMILY_AE_TOTAL_SOCKET_A_HIGH] =   (energyS1Acu >>16);
  objectDictionary[XCDV_FAMILY_AE_TOTAL_SOCKET_A_LOW]  =    energyS1Acu & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AE_TOTAL_SOCKET_B_HIGH] =   (energyS2Acu >>16);
  objectDictionary[XCDV_FAMILY_AE_TOTAL_SOCKET_B_LOW] =    energyS2Acu & 0xFFFF;

  objectDictionary[XCDV_FAMILY_AP_TOTAL_SOCKET_A_HIGH] =   (powerInstantaneousS1 >>16);
  objectDictionary[XCDV_FAMILY_AP_TOTAL_SOCKET_A_LOW]  =   powerInstantaneousS1 & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AP_TOTAL_SOCKET_B_HIGH] =   (powerInstantaneousS2 >>16);
  objectDictionary[XCDV_FAMILY_AP_TOTAL_SOCKET_B_LOW] =    powerInstantaneousS2 & 0xFFFF;

  objectDictionary[XCDV_FAMILY_AI_TOTAL_SOCKET_A_HIGH] =   (currentGlobalS1 >>16);
  objectDictionary[XCDV_FAMILY_AI_TOTAL_SOCKET_A_LOW]  =   currentGlobalS1 & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AI_TOTAL_SOCKET_B_HIGH] =   (currentGlobalS2 >>16);
  objectDictionary[XCDV_FAMILY_AI_TOTAL_SOCKET_B_LOW] =    currentGlobalS2 & 0xFFFF;

  objectDictionary[XCDV_FAMILY_AI_PH1_SOCKET_A_HIGH] =   (currentPh1S1 >>16);
  objectDictionary[XCDV_FAMILY_AI_PH1_SOCKET_A_LOW]  =   currentPh1S1 & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AI_PH1_SOCKET_B_HIGH] =   (currentPh1S2 >>16);
  objectDictionary[XCDV_FAMILY_AI_PH1_SOCKET_B_LOW] =    currentPh1S2 & 0xFFFF;

  objectDictionary[XCDV_FAMILY_AI_PH2_SOCKET_A_HIGH] =   (currentPh2S1 >>16);
  objectDictionary[XCDV_FAMILY_AI_PH2_SOCKET_A_LOW]  =   currentPh2S1 & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AI_PH2_SOCKET_B_HIGH] =   (currentPh2S2 >>16);
  objectDictionary[XCDV_FAMILY_AI_PH2_SOCKET_B_LOW] =    currentPh2S2 & 0xFFFF;

  objectDictionary[XCDV_FAMILY_AI_PH3_SOCKET_A_HIGH] =   (currentPh3S1 >>16);
  objectDictionary[XCDV_FAMILY_AI_PH3_SOCKET_A_LOW]  =   currentPh3S1 & 0xFFFF;
  objectDictionary[XCDV_FAMILY_AI_PH3_SOCKET_B_HIGH] =   (currentPh3S2 >>16);
  objectDictionary[XCDV_FAMILY_AI_PH3_SOCKET_B_LOW] =    currentPh3S2 & 0xFFFF;

#if DEBUG_ENERGIA
  if (timerTraza2 > TIMER_8S) {

	  if (networkParams.node1.currentConfiguration == 0)
		  env_debug("NOT_CONNECTED ..........");
	  else if (networkParams.node1.currentConfiguration == 1)
		  env_debug("SOCKET..................");
	  else if (networkParams.node1.currentConfiguration == 2)
		  env_debug("SUPPORT.................");
	  if (networkParams.node2.currentConfiguration == 0)
		  env_debug("  NOT_CONNECTED");
	  else if (networkParams.node2.currentConfiguration == 1)
		  env_debug("  SOCKET");
	  else if (networkParams.node2.currentConfiguration == 2)
		  env_debug("  SUPPORT");


	  env_debug("\n");
	  sprintf(bufferstr,"Energy meter nodeA: %i ---------- Energy meter nodeB: %i   \n", midAcEnergyActiveS1, midAcEnergyActiveS2);
	  env_debug(bufferstr);
	  env_debug("\n");

	  sprintf(bufferstr,"Energy nodeA: %i ------------- Energy nodeB: %i   \n", energyS1,energyS2);
	  env_debug(bufferstr);
	  env_debug("\n");

	  sprintf(bufferstr,"Energy nodeA last: %i ------------- Energy nodeB last: %i   \n", energyS1last,energyS2last);
	  env_debug(bufferstr);
	  env_debug("\n");

	  sprintf(bufferstr,"3 Energy Acu nodeA: %i --------- Energy Acu nodeB: %i   \n", energyS1Acu,energyS2Acu);
	  env_debug(bufferstr);
	  env_debug("\n");




	  timerTraza2 = 0;
  }
#endif
}


void setConfiguration (const __u16 targetNode, const __u8 masterNode, const Output_Config configuration, const float powerLimit)
{
  if (targetNode == networkParams.node1.nodeNumber)
  {
    nodeAconfigurationCommand.targetNode = targetNode;
    nodeAconfigurationCommand.masterNode = masterNode;
    nodeAconfigurationCommand.configuration = configuration;
    nodeAconfigurationCommand.powerLimit = powerLimit;
  }
  if (targetNode == networkParams.node2.nodeNumber)
  {
    nodeBconfigurationCommand.targetNode = targetNode;
    nodeBconfigurationCommand.masterNode = masterNode;
    nodeBconfigurationCommand.configuration = configuration;
    nodeBconfigurationCommand.powerLimit = powerLimit;
  }
}


void manageTransmissionsMaster(void)
{
  static unsigned int masterTransmissionIndex = 0;
  unsigned int id1 = 0, id2 = 0, data1 = 0, data2 = 0;
  int queGet = 0;
  EventWord eventReceived;

/*
  if (own_node == supervisor_node )
      getStateMessage( &networkParams.node1.stateNumber, &networkParams.node1.currentConfiguration, &networkParams.node1.socketContactorStatus ,&networkParams.node1.followedSocket, \
                         &networkParams.node1.maxVoltageRequest, &networkParams.node1.maxCurrentRequest);
*/
  if (own_node == supervisor_node ){

	  for (queGet = 0;queGet < NUM_PUBLISH; queGet++){
		    if (mensajeASuper.mensaje_rec[queGet]  == 1 ){
			    mensajeASuper.mensaje_rec[queGet]  = 0;

			    switch(queGet){

			    case PUBLISH_STATE:
			        getStateMessage( &networkParams.node1.stateNumber, &networkParams.node1.currentConfiguration, &networkParams.node1.socketContactorStatus ,&networkParams.node1.followedSocket, \
			                           &networkParams.node1.maxVoltageRequest, &networkParams.node1.maxCurrentRequest);
			    	break;

			    case PUBLISH_DELTA_DATA_MESSAGE:
			        if (getDataDeltasMessage(&id1, &data1, &id2, &data2) == 1){
			            objectDictionary[id1] = data1;
			            objectDictionary[id2] = data2;
			        }
			    	break;

			    case PUBLISH_LOW_LATENCY:
			        getLowLatencyMeasuresMessage(&networkParams.node1.outputVoltage, &networkParams.node1.availableVoltage, \
			                                        &networkParams.node1.outputCurrent, &networkParams.node1.availableCurrent);
			    	break;

			    case PUBLISH_TEMPERATURES:
			          getTemperaturesMessage(&networkParams.node1.temperature1, &networkParams.node1.temperature2, &networkParams.node1.temperature3, \
			                   &networkParams.node1.temperature4, &networkParams.node1.humidity1, &networkParams.node1.rpm1, &networkParams.node1.rpm2);
			    	break;

			    case PUBLISH_EVENT:
			        getEventMessage(&eventReceived);
			        // updateLastErrorIdentifierM(&message);
			        registerEventWord(&eventReceived, eventBufferM);
			    	break;

			    case PUBLISH_ACTIV:
			    	getActiveInductiveMessage(&(networkParams.node1.midAcEnergyActive), &(networkParams.node1.midAcEnergyInductive));
			    	break;

			    case PUBLISH_CAPACITIVE:
			        getCapacitiveInstantaneousPowerMessage(&(networkParams.node1.midAcEnergyCapacitive), &(networkParams.node1.midAcPowerInstantaneous));

			    	break;

			    case PUBLISH_CURRENT_PH1:
			    	getCurrentGlobalCurrentPh1Message(&(networkParams.node1.midAcCurrentGlobal), &(networkParams.node1.midAcCurrentPh1));
			    	break;

			    case PUBLISH_CURRENT_PH2:
			    	getCurrentPh2CurrentPh3Message(&(networkParams.node1.midAcCurrentPh2), &(networkParams.node1.midAcCurrentPh3));
			    	break;

			    case PUBLISH_VOLTAGE_PH1 :
			    	getVoltageGlobalVoltagePh1Message(&(networkParams.node1.midAcVoltageGlobal), &(networkParams.node1.midAcVoltagePh1));
			    	break;

			    case PUBLISH_VOLTAGE_PH2 :
			    	getVoltagePh2VoltagePh3Message(&(networkParams.node1.midAcVoltagePh2), &(networkParams.node1.midAcVoltagePh3));
			    	break;



			    }


		    }



	  }




  }

  if (timerMasterTransmission2 > TIMER_347MS)
  {
/*
	  if (networkParams.node1.errorComm != 0 || networkParams.node2.errorComm != 0){

 		   env_debug("Error comunicacion nodes...");
           env_debug("\n");
	  }
*/
      if (publishErrorComms(networkParams.node1.errorComm, networkParams.node2.errorComm,mb.errorModbusSlave,input_alarmsM,networkParams.socketA.errorComm,networkParams.socketB.errorComm) )
      {

        timerMasterTransmission2 = 0;
      }
  }
  if (timerMasterTransmission > TIMER_200MS)
  {
    switch (masterTransmissionIndex)
    {
    case 0:
//      if (publishConfiguration(networkParams.node1.errorComm, networkParams.node2.errorComm,nodeAconfigurationCommand.targetNode, nodeAconfigurationCommand.masterNode, nodeAconfigurationCommand.configuration, nodeAconfigurationCommand.powerLimit) )
#if DEBUG8
    	if (nodeaa !=nodeAconfigurationCommand.configuration ){
    		nodeaa = nodeAconfigurationCommand.configuration;

  		  env_debug(".........Conf setpoint enviada nodo A:");
          if (nodeAconfigurationCommand.configuration == 0)
   		   env_debug("  NOT_CONNECTED");
           else if (nodeAconfigurationCommand.configuration == 1)
    		   env_debug("  SOCKET");
           else if (nodeAconfigurationCommand.configuration == 2)
               		   env_debug("  SUPPORT");
           else
     		   env_debug("   STANDBY");

           env_debug("\n");
    }
#endif
    if (publishConfiguration(nodeAconfigurationCommand.targetNode, nodeAconfigurationCommand.masterNode, nodeAconfigurationCommand.configuration, nodeAconfigurationCommand.powerLimit,idMangueraActiva) )
      {
        masterTransmissionIndex++;
        timerMasterTransmission = 0;
      }
      break;
    case 1:
//    	if (publishConfiguration(networkParams.node1.errorComm, networkParams.node2.errorComm,nodeBconfigurationCommand.targetNode, nodeBconfigurationCommand.masterNode, nodeBconfigurationCommand.configuration, nodeBconfigurationCommand.powerLimit) )

#if DEBUG9
    	if (nodebb !=nodeBconfigurationCommand.configuration ){
    		nodebb = nodeBconfigurationCommand.configuration;
  		  env_debug(".........Conf setpoint enviada nodo B :");
          if (nodeBconfigurationCommand.configuration == 0)
   		   env_debug("  NOT_CONNECTED");
           else if (nodeBconfigurationCommand.configuration == 1)
    		   env_debug("  SOCKET");
           else if (nodeBconfigurationCommand.configuration == 2)
               		   env_debug("  SUPPORT");
           else
     		   env_debug("   STANDBY");

           env_debug("\n");
    	}
#endif
	if (publishConfiguration(nodeBconfigurationCommand.targetNode, nodeBconfigurationCommand.masterNode, nodeBconfigurationCommand.configuration, nodeBconfigurationCommand.powerLimit,idMangueraActiva) )
      {
        masterTransmissionIndex = 0;
        timerMasterTransmission = 0;
      }
      break;


    default:
      masterTransmissionIndex = 0;
      break;
    }
  }
}



// reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}

 // Converts a given integer x to string str[].  d is the number
 // of digits required in output. If d is more than the number
 // of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}


// Converts a given integer x to string str[].  d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.
int ulintToStr(unsigned long int x, char str[])
{
   int i = 0;
   while (x)
   {
       str[i++] = (x%10) + '0';
       x = x/10;
   }



   reverse(str, i);
   str[i] = '\0';
   return i;
}
// Converts a floating point number to string.
void ftoa(float n, char *res, int afterpoint)
{

    int neg = 0;
	if ( n < 0.0){
		neg = 1;
		n = n * -1.0;

	}
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }



    if (neg == 1){

    	i = strlen(res);
    	res[i+2] = 0x0;

    	while (i > 0){
    		 i--;
    		 res[i+1] = res[i];

    	}

    	res[0] = '-';

    }

}





void calefactorx(){
/*
    if (timerVentilador > (TIMER_20S + TIMER_10S))
  	  timerVentilador = TIMER_20S + TIMER_10S;
    if ((Temperature < VALUE_HEATER_MIN) || timerVentilador < TIMER_20S){
  	     HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
    }
    else{
  	  if (Temperature > VALUE_HEATER_MAX){
			  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
  	  }
    }
*/

}


void getStateMessage( __u8 *stateNumber, Output_Config *currentConfig, __u8 *contactorState, __u8 *listenedNode, float *maxVoltageRequest, float *maxCurrentRequest){


    *stateNumber = mensajeASuper.stateNumber;
    *currentConfig = mensajeASuper.currentConfiguration;
    *contactorState = mensajeASuper.socketContactorStatus;
    *listenedNode = mensajeASuper.followedSocket;
    *maxVoltageRequest = mensajeASuper.maxVoltageRequest;
    *maxCurrentRequest = mensajeASuper.maxCurrentRequest;



}

int getDataDeltasMessage( unsigned int *id1, unsigned int *data1, unsigned int *id2, unsigned int *data2)
{

  *id1 = mensajeASuper.id1;
  *data1 = mensajeASuper.data1;
  *id2 =  mensajeASuper.id2;
  *data2 = mensajeASuper.data2;
  return(1);
}


void getLowLatencyMeasuresMessage(float *outputVoltage, float *availableVoltage, float *outputCurrent, float *availableCurrent)
{


    *outputVoltage      = mensajeASuper.outputVoltage;
    *availableVoltage   = mensajeASuper.availableVoltage;
    *outputCurrent      = mensajeASuper.outputCurrent;
    *availableCurrent   = mensajeASuper.availableCurrent;

    return;
}

void getTemperaturesMessage(float *temperature1, float *temperature2, float *temperature3, \
                     float *temperature4, float *humidity1, unsigned char *tacho1, unsigned char *tacho2){


    *temperature1 = mensajeASuper.temperature1;
    *temperature2 = mensajeASuper.temperature2;
    *temperature3 = mensajeASuper.temperature3;
    *temperature4 = mensajeASuper.temperature4;
    *humidity1 = mensajeASuper.humidity1;
    *tacho1 = mensajeASuper.tacho1;
    *tacho2 = mensajeASuper.tacho2;

    return;
}




void getCapacitiveInstantaneousPowerMessage(__u32 *capacitive, __u32 *instantaneousPower){

    *capacitive = mensajeASuper.capacitive;
    *instantaneousPower = mensajeASuper.instantaneousPower;
}

void getCurrentGlobalCurrentPh1Message(__u32 *currentGlobal, __u32 *currentPh1){
    *currentGlobal = mensajeASuper.currentGlobal;
    *currentPh1 = mensajeASuper.currentPh1;
}

void getCurrentPh2CurrentPh3Message(__u32 *currentPh2, __u32 *currentPh3){
    *currentPh2 = mensajeASuper.currentPh2;
    *currentPh3 = mensajeASuper.currentPh3;
}

void getVoltageGlobalVoltagePh1Message(__u32 *voltageGlobal, __u32 *voltagePh1){
    *voltageGlobal = mensajeASuper.voltageGlobal;
    *voltagePh1 = mensajeASuper.voltagePh1;
}

void getVoltagePh2VoltagePh3Message(__u32 *voltagePh2, __u32 *voltagePh3){
    *voltagePh2 = mensajeASuper.voltagePh2;
    *voltagePh3 = mensajeASuper.voltagePh3;
}

void getActiveInductiveMessage(__u32 *active, __u32 *inductive){
    *active = mensajeASuper.active;
    *inductive = mensajeASuper.inductive;
}
