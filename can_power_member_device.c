/*
 * can_power_member_device.c
 *
 *  Created on: 22 nov. 2018
 *      Author: josepmaria.fernandez
 */
#include "can_power_member.h"

/* Insert your headers here */
#include "state_machine_master.h"
#include "specific_delta.h"
#include "state_machine_nodes.h"
#include "main.h"
#include "ModbusDataMap.h"
/* No more headers */
extern int deltaLastReceivedMessage;
extern int deltaErrorComm;

extern DeltaModule deltasSIMUL[];

/* Insert your variable declarations here, if any */
unsigned int timeOutCanStdMsg = 0;
uint32_t *txPtrMb1;
uint32_t *txPtrMb2;
Output_Config configurationSetPoint_old;

//__u16 deltaData2Supervisor[DELTA_ADDRESSES_PER_CONVERTER * MAXIMUM_NUMBER_OF_CONVERTERS];
//const __u16 deltaData2SupervisorSize = DELTA_ADDRESSES_PER_CONVERTER * MAXIMUM_NUMBER_OF_CONVERTERS;
__u16 lastSentIndex = 0;

/* No more variables*/

unsigned int canTransmitStdMsg(struct can_frame *canFrame){
  /* Insert your code for your platform */
  /* Local variable declarations*/
  uint8_t txData[8];
  CAN_TxHeaderTypeDef TxHeader;
  /* End of declarations */
  if ((own_node == supervisor_node + 1) && (envia_siempre == 0)){
	  return 1;
  }
  envia_siempre = 0;
  /* Insert your code for your platform */
  TxHeader.ExtId = canFrame->can_id;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_EXT;
  TxHeader.DLC = canFrame->can_dlc;
  TxHeader.TransmitGlobalTime = DISABLE;
  txData[0] = canFrame->data[0];
  txData[1] = canFrame->data[1];
  txData[2] = canFrame->data[2];
  txData[3] = canFrame->data[3];
  txData[4] = canFrame->data[4];
  txData[5] = canFrame->data[5];
  txData[6] = canFrame->data[6];
  txData[7] = canFrame->data[7];

  HAL_StatusTypeDef  messageStatus;
  messageStatus = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, (uint32_t*)&txPtrMb1) ;

  if (messageStatus == HAL_OK) return 1;
  else return 0;
  /* No more code*/
}

/* intiCanInternal initializes the can bus with master */
void initCanInternal(void)
{
  CAN_FilterTypeDef sFilterConfig;

   printf("Activating CAN internal\n");

   HAL_CAN_MspInit(&hcan1);
   HAL_CAN_Start(&hcan1);
   HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING );

   sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
   sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
   sFilterConfig.FilterIdHigh = 0x0000;
   sFilterConfig.FilterIdLow = 0x0000;
   sFilterConfig.FilterMaskIdHigh = 0x0000;
   sFilterConfig.FilterMaskIdLow = 0x0000;
   sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
   sFilterConfig.FilterActivation = ENABLE;
   sFilterConfig.FilterBank = 1;
   if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
   {
     Error_Handler();
   }
}

void isr_can_internal(void)
{
  // Insert ISR Code here
  struct can_frame message;
  unsigned long idGroup = 0;
  unsigned long emitterNode = 0;
  unsigned int id1 = 0, id2 = 0, data1 = 0, data2 = 0;
  EventWord eventReceived;
  EventWord DummyEvent2;
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t rxData[8] = {0,0,0,0,0,0,0,0};

  NodeParams *thisNodeParams;
  NodeParams *msgNodeParams;
  thisNodeParams = findNodeById(own_node);

  /** Get the message from the chip memory*/
  HAL_CAN_GetRxMessage(&hcan1,0,&RxHeader,rxData);

  message.can_dlc = RxHeader.DLC;
  message.can_id = RxHeader.ExtId;
  message.data[0] = rxData[0];
  message.data[1] = rxData[1];
  message.data[2] = rxData[2];
  message.data[3] = rxData[3];
  message.data[4] = rxData[4];
  message.data[5] = rxData[5];
  message.data[6] = rxData[6];
  message.data[7] = rxData[7];

  idGroup = message.can_id & 0xFFFFFFC0;
  emitterNode = message.can_id & 0x0000003F;

/*  if (message.can_id == 0x0C1){
    printf("Do something");
  }*/

  //__u16 voidData1 = 0;
  __u8 reserved1u8 = 0;
  __u8 reserved2u8 = 0;
  __u8 reserved3u8 = 0;
  float reservedFloat = 0.0;
  Output_Config dummyConf = NOT_CONNECTED;

  switch (idGroup)
  {
    case (0x00000040):
      //Coms for the master:
      if (emitterNode == networkParams.socketA.nodeNumber){
          getLowLatencyMeasures(&message, &networkParams.socketA.outputVoltage, &reservedFloat, \
                                &networkParams.socketA.outputCurrent, &reservedFloat);
          networkParams.socketA.lastReceivedMsg = 0;
      }
      else if (emitterNode == networkParams.socketB.nodeNumber){
          getLowLatencyMeasures(&message, &networkParams.socketB.outputVoltage, &reservedFloat, \
                                &networkParams.socketB.outputCurrent, &reservedFloat);
          networkParams.socketB.lastReceivedMsg = 0;
      }
      else if (emitterNode == networkParams.node1.nodeNumber){
          getLowLatencyMeasures(&message, &networkParams.node1.outputVoltage, &networkParams.node1.availableVoltage, \
                                &networkParams.node1.outputCurrent, &networkParams.node1.availableCurrent);
          networkParams.node1.lastReceivedMsg = 0;
      }
      else if (emitterNode == networkParams.node2.nodeNumber){
          getLowLatencyMeasures(&message, &networkParams.node2.outputVoltage, &networkParams.node2.availableVoltage, \
                                &networkParams.node2.outputCurrent, &networkParams.node2.availableCurrent);
          networkParams.node2.lastReceivedMsg = 0;
      }            //;
      break;
    case (0x000000C0):
      // Message for the node:
      if (emitterNode == supervisor_node || emitterNode == thisNodeParams->followedSocket){
          getState(&message,  &reserved1u8 , &dummyConf, &reserved2u8, &reserved3u8 ,&(thisNodeParams->maxVoltageRequest), &(thisNodeParams->maxCurrentRequest));
          thisNodeParams -> lastReceivedMsg = 0;
      }
      else{
    	  thisNodeParams -> lastReceivedMsg = 0;
      }
      // In case of master being a master:
      if (emitterNode == networkParams.socketA.nodeNumber){
          getState(&message, &networkParams.socketA.stateNumber, &dummyConf ,&reserved2u8, &reserved3u8,\
                               &networkParams.socketA.maxVoltageRequest, &networkParams.socketA.maxCurrentRequest);
          networkParams.socketA.lastReceivedMsg = 0;
          if (networkParams.socketA.stateNumber == 1){
        	  applyMinCurrent = 1;
          }
          else{
        	  applyMinCurrent = 0;
          }

      }
      else if (emitterNode == networkParams.socketB.nodeNumber){
          getState(&message, &networkParams.socketB.stateNumber, &dummyConf ,&reserved2u8, &reserved3u8, \
                               &networkParams.socketB.maxVoltageRequest, &networkParams.socketB.maxCurrentRequest);
          networkParams.socketB.lastReceivedMsg = 0;
      }
      else if (emitterNode == networkParams.node1.nodeNumber){
          getState(&message, &networkParams.node1.stateNumber, &networkParams.node1.currentConfiguration, &networkParams.node1.socketContactorStatus ,&networkParams.node1.followedSocket, \
                             &networkParams.node1.maxVoltageRequest, &networkParams.node1.maxCurrentRequest);
          networkParams.node1.lastReceivedMsg = 0;
      }
      else if (emitterNode == networkParams.node2.nodeNumber){
          getState(&message, &networkParams.node2.stateNumber, &networkParams.node2.currentConfiguration, &networkParams.node2.socketContactorStatus, &networkParams.node2.followedSocket, \
                             &networkParams.node2.maxVoltageRequest, &networkParams.node2.maxCurrentRequest);
          networkParams.node2.lastReceivedMsg = 0;
      }            //;

      break;
    case (0x00000140):
	if (emitterNode != 5)  {
	      getEvent(&message, &eventReceived);
	      updateLastErrorIdentifier(&message);
	      registerEventWord(&eventReceived, eventBuffer);
	}
	else if (emitterNode == 5){
      getEvent(&message, &eventReceived);
      updateLastErrorIdentifier(&message);
      registerEventWord(&eventReceived, eventBuffer);

      getEvent(&message, &eventReceived);
       updateLastErrorIdentifierM(&message);
       registerEventWord(&eventReceived, eventBufferM);
	}
      break;
    case (0x000001C0):
      // In case of master being a master:
      if (emitterNode == networkParams.socketA.nodeNumber){
          getTemperatures(&message, &networkParams.socketA.temperature1, &networkParams.socketA.temperature2, &networkParams.socketA.temperature3, \
                   &networkParams.socketA.temperature4, &networkParams.socketA.humidity1, &networkParams.socketA.rpm1, &networkParams.socketA.rpm2);
      }
      else if (emitterNode == networkParams.socketB.nodeNumber){
          getTemperatures(&message, &networkParams.socketB.temperature1, &networkParams.socketB.temperature2, &networkParams.socketB.temperature3, \
                   &networkParams.socketB.temperature4, &networkParams.socketB.humidity1, &networkParams.socketB.rpm1, &networkParams.socketB.rpm2);
      }
      else if (emitterNode == networkParams.node1.nodeNumber){
          getTemperatures(&message, &networkParams.node1.temperature1, &networkParams.node1.temperature2, &networkParams.node1.temperature3, \
                   &networkParams.node1.temperature4, &networkParams.node1.humidity1, &networkParams.node1.rpm1, &networkParams.node1.rpm2);
      }
      else if (emitterNode == networkParams.node2.nodeNumber){
          getTemperatures(&message, &networkParams.node2.temperature1, &networkParams.node2.temperature2, &networkParams.node2.temperature3, \
                   &networkParams.node2.temperature4, &networkParams.node2.humidity1, &networkParams.node2.rpm1, &networkParams.node2.rpm2);
      }
      break;
    case (0x00000240):
      //getActiveEnergy(&message, &dummyLong0, &dummyLong1);
      break;
    case (0x00000080):
		// Message for the node:
      if (emitterNode == supervisor_node || emitterNode == thisNodeParams -> followedSocket){
          getChargeCommand(&message, &(thisNodeParams->command), &(thisNodeParams -> voltageSetPoint), &(thisNodeParams -> currentSetPoint),
              &(thisNodeParams -> socketContactorRequest));
          thisNodeParams -> lastReceivedMsg = 0;
      }
      else{
    	  thisNodeParams -> lastReceivedMsg = 0;
      }

      // In case of master being a master:
      if (emitterNode == networkParams.socketA.nodeNumber){
          getChargeCommand(&message, &networkParams.socketA.command, &networkParams.socketA.voltageSetPoint, &networkParams.socketA.currentSetPoint, &networkParams.socketA.contactorRequest);
          networkParams.socketA.lastReceivedMsg = 0;
          if (networkParams.socketA.stateNumber == 1){
        	  applyMinCurrent = 1;
        	  minCurrent = networkParams.socketA.currentSetPoint;
          }
          else{
        	  applyMinCurrent = 0;
        	  minCurrent = 0.0;
          }


      }
      else if (emitterNode == networkParams.socketB.nodeNumber){
          getChargeCommand(&message, &networkParams.socketB.command, &networkParams.socketB.voltageSetPoint, &networkParams.socketB.currentSetPoint, &networkParams.socketB.contactorRequest);
          networkParams.socketB.lastReceivedMsg = 0;
      }
      else if (emitterNode == networkParams.node1.nodeNumber){
          getChargeCommand(&message, &networkParams.node1.command, &networkParams.node1.voltageSetPoint, &networkParams.node1.currentSetPoint, &networkParams.node1.socketContactorRequest);
          networkParams.node1.lastReceivedMsg = 0;
      }
      else if (emitterNode == networkParams.node2.nodeNumber){
          getChargeCommand(&message, &networkParams.node2.command, &networkParams.node2.voltageSetPoint, &networkParams.node2.currentSetPoint, &networkParams.node2.socketContactorRequest);
          networkParams.node2.lastReceivedMsg = 0;
      }            //;
      break;
    case (0x00000100):
      getAckEvent(&message, &DummyEvent2);
      updateLastErrorIdentifier(&message);
      break;
    case (0x00000180):
      getConfiguration(&message, &(thisNodeParams ->followedSocketRequest), &(thisNodeParams -> configurationSetPoint), &(thisNodeParams -> powerLimit), &idMangueraActiva);
      if (configurationSetPoint_old != thisNodeParams -> configurationSetPoint){
    	  configurationSetPoint_old = thisNodeParams -> configurationSetPoint;
#if DEBUG
  		  env_debug(".........Conf setpoint deseado :");
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


      break;
    case (0x00000900):
      //Only for master
      getDataDeltas(&message, &id1, &data1, &id2, &data2);
      objectDictionary[id1] = data1;
      objectDictionary[id2] = data2;
      break;

    case (0x000002C0):
      msgNodeParams = findNodeById(emitterNode);
      if (msgNodeParams != &voidNode)
      {
        if (mutexPower == ALLOW_ACCESS)
        {
          mutexPower = PROTECTED;
          getActiveInductive(&message, &(msgNodeParams ->midAcEnergyActive), &(msgNodeParams ->midAcEnergyInductive));
          mutexPower = ALLOW_ACCESS;
        }
      }
      break;
    case (0x00000340):
      msgNodeParams = findNodeById(emitterNode);
      if (msgNodeParams != &voidNode)
        getCapacitiveInstantaneousPower(&message, &(msgNodeParams ->midAcEnergyCapacitive), &(msgNodeParams ->midAcPowerInstantaneous));
      break;
    case (0x000003C0):
      msgNodeParams = findNodeById(emitterNode);
      if (msgNodeParams != &voidNode)
        getCurrentGlobalCurrentPh1 (&message, &(msgNodeParams ->midAcCurrentGlobal), &(msgNodeParams ->midAcCurrentPh1));
      break;
    case (0x00000440):
      msgNodeParams = findNodeById(emitterNode);
      if (msgNodeParams != &voidNode)
        getCurrentPh2CurrentPh3 (&message, &(msgNodeParams ->midAcCurrentPh2), &(msgNodeParams ->midAcCurrentPh3));
      break;
    case (0x000004C0):
      msgNodeParams = findNodeById(emitterNode);
      if (msgNodeParams != &voidNode)
        getVoltageGlobalVoltagePh1 (&message, &(msgNodeParams ->midAcVoltageGlobal), &(msgNodeParams ->midAcVoltagePh1));
      break;
    case (0x00000540):
      msgNodeParams = findNodeById(emitterNode);
      if (msgNodeParams != &voidNode)
        getVoltagePh2VoltagePh3 (&message, &(msgNodeParams ->midAcVoltagePh2), &(msgNodeParams ->midAcVoltagePh3));
      break;
    default:
      break;
  }
}

/* The following function initializes the can 1 peripheral */


void initCanDelta(void)
{
  CAN_FilterTypeDef sFilterConfig;

  HAL_CAN_MspInit(&hcan2);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

  hcan2.Instance = CAN1; //Is a HAL bug!!! // TODO: Josep: Instance = CAN2 it works!

  //0x1A00 1000
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = REQUEST_MASTER_READ_MASK_ID >> 13; // EXTID[28:13]
  sFilterConfig.FilterIdLow = (0xFFFF & (REQUEST_MASTER_READ_MASK_ID << 3)) | (1 << 2); // EXTID[12:0]
  sFilterConfig.FilterMaskIdHigh = 0xFFFF;
  sFilterConfig.FilterMaskIdLow = 0xFFFF;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.FilterBank = 14;
  sFilterConfig.SlaveStartFilterBank = 1;
  if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
  {
     Error_Handler();
  }

  // 0x0200XXXX 0x0201XXXX 0x0202XXXX 0x0203XXXX
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = SNAP_STEP_1_WRITE_MASK_ID >> 13; // EXTID[28:13]
  sFilterConfig.FilterIdLow = (0xFFFF & (SNAP_STEP_1_WRITE_MASK_ID << 3)) | (1 << 2); // EXTID[12:0]
  sFilterConfig.FilterMaskIdHigh = 0xFFE0;
  sFilterConfig.FilterMaskIdLow = 0x0000 | (1 << 2);
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.FilterBank = 15;
  sFilterConfig.SlaveStartFilterBank = 2;
  if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
  {
     Error_Handler();
  }

  //0x094012XX 0x094032XX
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = (PHYADDRESS_READ_MASK_ID >> 13); // EXTID[28:13]
  sFilterConfig.FilterIdLow = (0xFFFF & (PHYADDRESS_READ_MASK_ID << 3)) | (1 << 2); // EXTID[12:0]
  sFilterConfig.FilterMaskIdHigh = 0xFFF9;
  sFilterConfig.FilterMaskIdLow = 0xF800 | (1 << 2);
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.FilterBank = 16;
  sFilterConfig.SlaveStartFilterBank = 3;
  if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
  {
     Error_Handler();
  }

  //0x0941XXXX
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = GENERAL_MASK_ID_ACK >> 13; // EXTID[28:13]
  sFilterConfig.FilterIdLow = (0xFFFF & (GENERAL_MASK_ID_ACK << 3)) | (1 << 2); // EXTID[12:0]
  sFilterConfig.FilterMaskIdHigh = 0xFFF8;
  sFilterConfig.FilterMaskIdLow = 0x0000 | (1 << 2);
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.FilterBank = 17;
  sFilterConfig.SlaveStartFilterBank = 4;
  if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
  {
     Error_Handler();
  }

  //0x1B4031XX
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = RESPONSE_ID_WRITE_MASK_ID >> 13; // EXTID[28:13]
  sFilterConfig.FilterIdLow = (0xFFFF & (RESPONSE_ID_WRITE_MASK_ID << 3)) | (1 << 2); // EXTID[12:0]
  sFilterConfig.FilterMaskIdHigh = 0xFFFF;
  sFilterConfig.FilterMaskIdLow = 0xF800 | (1 << 2);
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.FilterBank = 18;
  sFilterConfig.SlaveStartFilterBank = 5;
  if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
  {
     Error_Handler();
  }

  //0x1D4041XX
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = PING_ACKNOWLEDGE_READ_MASK_ID >> 13; // EXTID[28:13]
	sFilterConfig.FilterIdLow = (0xFFFF & (PING_ACKNOWLEDGE_READ_MASK_ID << 3)) | (1 << 2); // EXTID[12:0]
	sFilterConfig.FilterMaskIdHigh = 0xFFFF;
	sFilterConfig.FilterMaskIdLow = 0xF800 | (1 << 2);
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.FilterBank = 19;
	sFilterConfig.SlaveStartFilterBank = 6;
	if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
	{
		 Error_Handler();
	}

  hcan2.Instance = CAN2;

}


/*
void initCanDelta(void)
{
  CAN_FilterTypeDef sFilterConfig;

  printf("Activating CAN for Delta\n");

  HAL_CAN_MspInit(&hcan2);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING );

  hcan2.Instance = CAN1; //Is a HAL bug!!!
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.FilterBank = 2;
  if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
  {
      Error_Handler();
  }


  hcan2.Instance = CAN2;

}
*/
/* This function loads the message to be sent in the corresponding buffer */
unsigned int sendCanDelta(struct can_frame *c)
{
  /* Local variable declarations*/
  uint8_t txData[8];
  CAN_TxHeaderTypeDef TxHeader;
  /* End of declarations */

  /* Insert your code for your platform */
  TxHeader.ExtId = c->can_id;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_EXT;
  TxHeader.DLC = c->can_dlc;
  TxHeader.TransmitGlobalTime = DISABLE;
  txData[0] = c->data[0];
  txData[1] = c->data[1];
  txData[2] = c->data[2];
  txData[3] = c->data[3];
  txData[4] = c->data[4];
  txData[5] = c->data[5];
  txData[6] = c->data[6];
  txData[7] = c->data[7];
  /* No more code*/
  HAL_StatusTypeDef  messageStatus;
  messageStatus = HAL_CAN_AddTxMessage(&hcan2, &TxHeader, txData, (uint32_t*)&txPtrMb2);

  if (messageStatus == HAL_OK) return 1;
  else return 0;
}

void isr_can_delta(void)
{
  DeltaModule *delta;
  struct can_frame message;
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t rxData[8] = {0,0,0,0,0,0,0,0};
  int i;

  /** Get the message from the chip memory*/
  HAL_CAN_GetRxMessage(&hcan2,0,&RxHeader,rxData);



  message.can_dlc = RxHeader.DLC;
  message.can_id = RxHeader.ExtId;
  message.data[0] = rxData[0];
  message.data[1] = rxData[1];
  message.data[2] = rxData[2];
  message.data[3] = rxData[3];
  message.data[4] = rxData[4];
  message.data[5] = rxData[5];
  message.data[6] = rxData[6];
  message.data[7] = rxData[7];


  // Call Delta structures to check if messages are for them
  delta = getModuleFromCanMsg(message.can_id); //Get the module if exists
  deltaLastReceivedMessage = 0;
  // if (message.can_id == 0x0941D301 || RxHeader.ExtId == 0x0941D301 ){
  //	  env_debug("si llega...\n ");
  // }
  if (delta != &nullConverter)
  {
    delta->can.pollReceived(delta,&message); //Case of known converter, save data!
  }
  else
  {
    for (i = 0; i < maxNumberOfConvertersWorking; i++)
    {
      deltaConverter[i].can.pollReceived(&(deltaConverter[i]),&message); //case of new discovered converter without ID.
    }
  }
}

#if SIMUL_DELTAS
void can_delta_SIMUL(int deltaSIMUL)
{
  DeltaModule *delta;
  struct can_frame message;
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t rxData[8] = {0,0,0,0,0,0,0,0};
  int i;


  /** Get the message from the chip memory*/
  //HAL_CAN_GetRxMessage(&hcan2,0,&RxHeader,rxData);
  for (i = 0; i < maxNumberOfConvertersWorking; i++)
  {
    deltaConverter[i].machine.pong = 1;
  }
  if (timerSimul1 > TIMER_1S){
	  if (deltasSIMUL[deltaSIMUL].msgIdSimul == 0){
		  deltasSIMUL[deltaSIMUL].msgIdSimul = VOLTAGE_CURRENT_STATUS_READ_MASK_IDSIMUL;
		  timerSimul1 = 0;
	  }

  }

  if (deltasSIMUL[deltaSIMUL].msgIdSimul == 0)
	  return;

  switch (deltasSIMUL[deltaSIMUL].msgIdSimul)
   {
		case (REQUEST_MASTER_READ_MASK_IDSIMUL):
    	    message.can_id          = REQUEST_MASTER_READ_MASK_ID;
    	    message.can_dlc          = 1;
				   break;

       case (VOLTAGE_CURRENT_STATUS_READ_MASK_IDSIMUL):
						message.can_id          = VOLTAGE_CURRENT_STATUS_READ_MASK_ID + deltaSIMUL+1;
						message.can_dlc          = 1;
						  message.data[0] =((int)(deltasSIMUL[deltaSIMUL].outputVoltage * 10))  & 0xFF;
						  message.data[1] =((int)(deltasSIMUL[deltaSIMUL].outputVoltage * 10)) >> 8;
						  message.data[2] = ((int)(deltasSIMUL[deltaSIMUL].outputCurrent * 10)) & 0xFF;
						  message.data[3] =((int)(deltasSIMUL[deltaSIMUL].outputCurrent * 10)) >> 8;
						  message.data[4] = ((int)(deltasSIMUL[deltaSIMUL].powerSetPoint * 0.1))  & 0xFF;
						  message.data[5] =((int)(deltasSIMUL[deltaSIMUL].powerSetPoint * 0.1)) >> 8;
						  message.data[6] = 0x00;
						  message.data[7] = 0x00;
					deltasSIMUL[deltaSIMUL].msgIdSimul = 0;
    		   break;
       case (TEMPERATURE_PRIMARY_READ_MASK_IDSIMUL):
    		   break;
       case (TEMPERATURE_SECONDARY_READ_MASK_IDSIMUL):
    		   break;
       case (TEMPERATURE_AIR_READ_MASK_IDSIMUL):
    		   break;
       case (MODULE_SERIAL_NUMBER_1_READ_MASK_IDSIMUL):
				message.can_id          = MODULE_SERIAL_NUMBER_1_READ_MASK_ID + deltaSIMUL+1;
				message.can_dlc          = 1;
				  message.data[0] = 0x8A;
				  message.data[1] = 0x48 + deltaSIMUL+1;
				  message.data[2] = 0x24;
				  message.data[3] = 0x84 + deltaSIMUL+1;
				  message.data[4] = 0x01;
				  message.data[5] = 0xF0 + deltaSIMUL+1;
				  message.data[6] = 0x22;
				  message.data[7] = 0xE0 + deltaSIMUL+1;
			deltasSIMUL[deltaSIMUL].msgIdSimul = 0;
    		   break;
       case (MODULE_SERIAL_NUMBER_2_READ_MASK_IDSIMUL):
				message.can_id          = MODULE_SERIAL_NUMBER_2_READ_MASK_ID + deltaSIMUL+1;
				message.can_dlc          = 1;
				message.data[0] = 0x9A;
				message.data[1] = 0x58 + deltaSIMUL+1;
				message.data[2] = 0x34;
				message.data[3] = 0x94 + deltaSIMUL+1;
				message.data[4] = 0x11;
				message.data[5] = 0x70 + deltaSIMUL+1;
				message.data[6] = 0x22;
				message.data[7] = 0x80 + deltaSIMUL+1;
				deltasSIMUL[deltaSIMUL].msgIdSimul = 0;
    		   break;
       case (MODULE_SERIAL_NUMBER_3_READ_MASK_IDSIMUL):
						message.can_id          = MODULE_SERIAL_NUMBER_3_READ_MASK_ID + deltaSIMUL+1;
						message.can_dlc          = 1;
						message.data[0] = 0x3A;
						message.data[1] = 0x48 + deltaSIMUL+1;
						message.data[2] = 0x54;
						message.data[3] = 0x64 + deltaSIMUL+1;
						message.data[4] = 0x71;
						message.data[5] = 0x80 + deltaSIMUL+1;
						message.data[6] = 0x92;
						message.data[7] = 0xa0 + deltaSIMUL+1;
						deltasSIMUL[deltaSIMUL].msgIdSimul = 0;
    		   break;
       case (MODULE_SERIAL_NUMBER_4_READ_MASK_IDSIMUL):
 						message.can_id          = MODULE_SERIAL_NUMBER_4_READ_MASK_ID + deltaSIMUL+1;
 						message.can_dlc          = 1;
 						message.data[0] = 0x3A;
 						message.data[1] = 0x48 + deltaSIMUL+1;
 						message.data[2] = 0x54;
 						message.data[3] = 0x64 + deltaSIMUL+1;
 						message.data[4] = 0x71;
 						message.data[5] = 0x80 + deltaSIMUL+1;
 						message.data[6] = 0x92;
 						message.data[7] = 0xa0 + deltaSIMUL+1;
 						deltasSIMUL[deltaSIMUL].msgIdSimul = 0;
     		   break;
       case (MODULE_SERIAL_NUMBER_5_READ_MASK_IDSIMUL):
				message.can_id          = MODULE_SERIAL_NUMBER_5_READ_MASK_ID + deltaSIMUL+1;
				message.can_dlc          = 1;
				message.data[0] = 0x8A;
				message.data[1] = 0x48 + deltaSIMUL+1;
				message.data[2] = 0x24;
				message.data[3] = 0x84 + deltaSIMUL+1;
				message.data[4] = 0x01;
				message.data[5] = 0xF0 + deltaSIMUL+1;
				message.data[6] = 0x22;
				message.data[7] = 0xE0 + deltaSIMUL+1;
				deltasSIMUL[deltaSIMUL].msgIdSimul = 0;
    		   break;
       case (MODULE_SERIAL_NUMBER_6_READ_MASK_IDSIMUL):
				message.can_id          = MODULE_SERIAL_NUMBER_6_READ_MASK_ID + deltaSIMUL+1;
				message.can_dlc          = 1;
				message.data[0] = 0x9A;
				message.data[1] = 0x58 + deltaSIMUL+1;
				message.data[2] = 0x34;
				message.data[3] = 0x94 + deltaSIMUL+1;
				message.data[4] = 0x11;
				message.data[5] = 0x70 + deltaSIMUL+1;
				message.data[6] = 0x22;
				message.data[7] = 0x80 + deltaSIMUL+1;
				deltasSIMUL[deltaSIMUL].msgIdSimul = 0;
    		   break;
       case (MODULE_SERIAL_NUMBER_7_READ_MASK_IDSIMUL):
						message.can_id          = MODULE_SERIAL_NUMBER_7_READ_MASK_ID + deltaSIMUL+1;
						message.can_dlc          = 1;
						message.data[0] = 0x3A;
						message.data[1] = 0x48 + deltaSIMUL+1;
						message.data[2] = 0x54;
						message.data[3] = 0x64 + deltaSIMUL+1;
						message.data[4] = 0x71;
						message.data[5] = 0x80 + deltaSIMUL+1;
						message.data[6] = 0x92;
						message.data[7] = 0xa0 + deltaSIMUL+1;
						deltasSIMUL[deltaSIMUL].msgIdSimul = 0;
    		   break;
       case (MODULE_SERIAL_NUMBER_8_READ_MASK_IDSIMUL):
 						message.can_id          = MODULE_SERIAL_NUMBER_8_READ_MASK_ID + deltaSIMUL+1;
 						message.can_dlc          = 1;
 						message.data[0] = 0x3A;
 						message.data[1] = 0x48 + deltaSIMUL+1;
 						message.data[2] = 0x54;
 						message.data[3] = 0x64 + deltaSIMUL+1;
 						message.data[4] = 0x71;
 						message.data[5] = 0x80 + deltaSIMUL+1;
 						message.data[6] = 0x92;
 						message.data[7] = 0xa0 + deltaSIMUL+1;
 						deltasSIMUL[deltaSIMUL].msgIdSimul = 0;
     		   break;

       case (PING_ACKNOWLEDGE_READ_MASK_IDSIMUL):
    		   break;
       case (ENTER_MONITOR_STATE_READ_MASK_IDSIMUL):
    		   break;
       case (FAN_SPEED_READ_MASK_IDSIMUL):
    		   break;
       case (OUTPUT_VOLTAGE_READ_MASK_IDSIMUL):
								message.can_id          = OUTPUT_VOLTAGE_READ_MASK_ID + deltaSIMUL+1;
								message.can_dlc          = 1;
								  message.data[0] =((int)(deltasSIMUL[deltaSIMUL].outputVoltage * 10))  & 0xFF;
								  message.data[1] =((int)(deltasSIMUL[deltaSIMUL].outputVoltage * 10)) >> 8;
								  message.data[2] = ((int)(deltasSIMUL[deltaSIMUL].outputCurrent * 10)) & 0xFF;
								  message.data[3] =((int)(deltasSIMUL[deltaSIMUL].outputCurrent * 10)) >> 8;
								  message.data[4] = ((int)(deltasSIMUL[deltaSIMUL].powerSetPoint * 0.1))  & 0xFF;
								  message.data[5] =((int)(deltasSIMUL[deltaSIMUL].powerSetPoint * 0.1)) >> 8;
								  message.data[6] = 0x00;
								  message.data[7] = 0x00;
							deltasSIMUL[deltaSIMUL].msgIdSimul = 0;
    		   break;
       case (RESPONSE_ID_WRITE_MASK_IDSIMUL):
   	    	    message.can_id          = RESPONSE_ID_WRITE_MASK_ID + deltaSIMUL+10;
   	     	    message.can_dlc          = 1;
   	     	deltasSIMUL[deltaSIMUL].msgIdSimul = 0;

    		   break;

       case (PHYADDRESS_READ_MASK_IDSIMUL):
			message.can_id          = PHYADDRESS_READ_MASK_ID + deltaSIMUL+1;
			message.can_dlc          = 1;
			message.data[0] = deltaSIMUL+1;
			message.data[1] = 0x0;
			deltasSIMUL[deltaSIMUL].msgIdSimul = 0;
    		   break;

       default:
    	   break;
   };
 // deltasSIMUL[deltaSIMUL].msgIdSimul = 0;
/*
  message.can_dlc = RxHeader.DLC;
  message.can_id = RxHeader.ExtId;
  message.data[0] = rxData[0];
  message.data[1] = rxData[1];
  message.data[2] = rxData[2];
  message.data[3] = rxData[3];
  message.data[4] = rxData[4];
  message.data[5] = rxData[5];
  message.data[6] = rxData[6];
  message.data[7] = rxData[7];
*/
  // Call Delta structures to check if messages are for them
  delta = getModuleFromCanMsg(message.can_id); //Get the module if exists
  deltaLastReceivedMessage = 0;
  if (delta != &nullConverter)
  {
    delta->can.pollReceived(delta,&message); //Case of known converter, save data!
  }
  else
  {
    for (i = 0; i < maxNumberOfConvertersWorking; i++)
    {
      deltaConverter[i].can.pollReceived(&(deltaConverter[i]),&message); //case of new discovered converter without ID.
    }
  }
}

#endif



void queryDeltaData(struct can_frame *canFrame)
{
  static int dataPointer = NODE1PADDING;
  unsigned int nodePadding = 0;

  if (own_node == networkParams.node1.nodeNumber)  nodePadding = NODE1PADDING;
  if (own_node == networkParams.node2.nodeNumber)  nodePadding = NODE2PADDING;

  if (own_node == supervisor_node + 1){
	  mensajeASuper.mensaje_rec[PUBLISH_DELTA_DATA_MESSAGE] = 1;
	  mensajeASuper.id1 = dataPointer;
	  mensajeASuper.data1 = objectDictionary[dataPointer];


	  dataPointer++;
	  if (dataPointer >= (DELTA_ADDRESSES_PER_CONVERTER * MAXIMUM_NUMBER_OF_CONVERTERS + nodePadding) ) dataPointer = nodePadding;

	  mensajeASuper.id2 = dataPointer;
	  mensajeASuper.data2 = objectDictionary[dataPointer];

	  dataPointer++;
	  if (dataPointer >= (DELTA_ADDRESSES_PER_CONVERTER * MAXIMUM_NUMBER_OF_CONVERTERS + nodePadding) ) dataPointer = nodePadding;

  }
  else {

	  canFrame->can_id = 0x900 + own_node;
	  canFrame->data[0] = dataPointer >> 8;
	  canFrame->data[1] = dataPointer;
	  canFrame->data[2] = objectDictionary[dataPointer] >> 8;
	  canFrame->data[3] = objectDictionary[dataPointer];

	  dataPointer++;
	  if (dataPointer >= (DELTA_ADDRESSES_PER_CONVERTER * MAXIMUM_NUMBER_OF_CONVERTERS + nodePadding) ) dataPointer = nodePadding;

	  canFrame->data[4] = dataPointer >> 8;
	  canFrame->data[5] = dataPointer;
	  canFrame->data[6] = objectDictionary[dataPointer] >> 8;
	  canFrame->data[7] = objectDictionary[dataPointer];

	  dataPointer++;
	  if (dataPointer >= (DELTA_ADDRESSES_PER_CONVERTER * MAXIMUM_NUMBER_OF_CONVERTERS + nodePadding) ) dataPointer = nodePadding;

	  if (nodePadding) canFrame->can_dlc = 8;
	  else canFrame->can_dlc =  0;
  }

}

void loadStoredParams(const int position)
{
  DeltaModule *thisConverter;
  thisConverter = getModuleFromPosition(position);

  if (position == 0) return; // Position 0 is invalid. It is used for empty trays

  // As far as every converter has exactly the same number of entries in the Modbus dictionary, the padding strategy will be used depending on the position
  unsigned int paddingDeltaAdd = DELTA_ADDRESSES_PER_CONVERTER * (position - 1);

  thisConverter->chargedEnergy = (((unsigned long long)objectDictionary[DELTA25_ENERGIA_HIGH_1+paddingDeltaAdd])<<16) + (((unsigned long long)objectDictionary[DELTA25_ENERGIA_LOW_1+paddingDeltaAdd]));
  thisConverter->chargedEnergy *= 1000;
  thisConverter->chargedEnergy += (((unsigned long long)objectDictionary[DELTA25_RESERVADO0_1+paddingDeltaAdd])); //Used for W
/*
  thisConverter->operationTime = ((unsigned long)objectDictionary[DELTA_WORKING_HOURS_1+paddingDeltaAdd]);
  thisConverter->operationTime *= 3600;
  thisConverter->operationTime += (((unsigned long long)objectDictionary[DELTA_RESERVADO0_2+paddingDeltaAdd])); //Used for s
  */
}

void prepareParamsToSave(const int position)
{
  DeltaModule *thisConverter;
  thisConverter = getModuleFromPosition(position);

  if (position == 0) return; // Position 0 is invalid. It is used for empty trays

  // As far as every converter has exactly the same number of entries in the Modbus dictionary, the padding strategy will be used depending on the position
  unsigned int paddingDeltaAdd = DELTA_ADDRESSES_PER_CONVERTER * (position - 1);

  objectDictionary[DELTA25_ENERGIA_HIGH_1 + paddingDeltaAdd] = (thisConverter->chargedEnergy / 1000) >> 16;
  objectDictionary[DELTA25_ENERGIA_LOW_1 + paddingDeltaAdd] = (thisConverter->chargedEnergy / 1000) & 0xFFFF;
  objectDictionary[DELTA25_RESERVADO0_1 + paddingDeltaAdd] = (thisConverter->chargedEnergy % 1000); //Keep Watts
/*
  objectDictionary[DELTA_WORKING_HOURS_1 + paddingDeltaAdd] = (thisConverter->operationTime / 3600) ;
  objectDictionary[DELTA_RESERVADO0_2 + paddingDeltaAdd] = (thisConverter->chargedEnergy % 3600); //Save seconds
  */
}

