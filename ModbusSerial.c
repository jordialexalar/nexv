#include "ModbusSerial.h"
#include "ModbusLog.h"
#include "ModbusSettings.h"
#include "main.h"

unsigned char msgModbusTxSlave[MB_BUFFER_SIZE<<1];
unsigned char msgModbusRxSlave[MB_BUFFER_SIZE<<1];
unsigned char tinnyRxBufferSlave[2];
unsigned int msgModbusRxBuffIndexSlave = 0;
unsigned int msgModbusRxBuffReadedSlave = 0;

unsigned char tinnyRxBufferSlave3[20];
unsigned int msgModbusRxBuffIndexSlave3 = 0;
unsigned int msgModbusRxBuffReadedSlave3 = 0;
unsigned char msgModbusTxMaster[MB_BUFFER_SIZE<<1];
unsigned char msgModbusRxMaster[MB_BUFFER_SIZE<<1];
unsigned char tinnyRxBufferMaster[2];
unsigned int msgModbusRxBuffIndexMaster = 0;
unsigned int msgModbusRxBuffReadedMaster = 0;

unsigned char msgModbusTxDebug[MB_BUFFER_SIZE<<1];
unsigned char msgModbusRxDebug[MB_BUFFER_SIZE<<1];
unsigned char tinnyRxBufferDebug[2];
unsigned int msgModbusRxBuffIndexDebug = 0;
unsigned int msgModbusRxBuffReadedDebug = 0;
// Clear flags of overflow
void serial_clear3()
{
  //buffers must be cleared
  HAL_UART_Abort(&huart3); //Stops and clears any pending process
  HAL_UART_AbortReceive_IT(&huart3);
  HAL_UART_AbortTransmit_IT(&huart3);
  // HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);  // Enable listening the bus
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_UART_Receive_IT(&huart3, (uint8_t *)tinnyRxBufferSlave3, 10);//MB_BUFFER_SIZE<<1); // Worst case, reserve all memory
  msgModbusRxBuffIndexSlave3 = 0;
  msgModbusRxBuffReadedSlave3 = 0;
}
void serial_clear()
{
  //buffers must be cleared
  HAL_UART_Abort(&huart1); //Stops and clears any pending process
  HAL_UART_AbortReceive_IT(&huart1);
  HAL_UART_AbortTransmit_IT(&huart1);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);  // Enable listening the bus
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_UART_Receive_IT(&huart1, (uint8_t *)tinnyRxBufferSlave, 1);//MB_BUFFER_SIZE<<1); // Worst case, reserve all memory
  msgModbusRxBuffIndexSlave = 0;
  msgModbusRxBuffReadedSlave = 0;
}

// Get how much data is at the RX FIFO Buffer
Uint16 serial_rxBufferStatus(){
/*	return SciaRegs.SCIFFRX.bit.RXFFST;*/
  //return ((huart1.RxXferSize - huart1.RxXferCount - msgModbusRxBuffIndex) >> 1); //Return the number of bytes that left to process
  return (msgModbusRxBuffIndexSlave - msgModbusRxBuffReadedSlave);
}

// Enable or disable RX (receiver)
void serial_setSerialRxEnabled(bool status){
  //Needed?
/*	SERIAL_DEBUG();
	SciaRegs.SCICTL1.bit.RXENA = status;*/
  HAL_UART_Abort(&huart1); //Stops and clears any pending process
  HAL_UART_AbortReceive_IT(&huart1);
  HAL_UART_AbortTransmit_IT(&huart1);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);  // Enable listening the bus
  HAL_UART_Receive_IT(&huart1, (uint8_t *)tinnyRxBufferSlave, 1);//MB_BUFFER_SIZE<<1); // Worst case, reserve all memory

}

// Enable or disable TX (trasmiter)
void serial_setSerialTxEnabled(bool status){
  //Needed?
/*	SERIAL_DEBUG();
	SciaRegs.SCICTL1.bit.TXENA = status;*/
  HAL_UART_Abort(&huart1); //Stops and clears any pending process
  HAL_UART_AbortTransmit_IT(&huart1);
  HAL_UART_AbortReceive_IT(&huart1);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);  // Enable listening the bus
}

// Initialize Serial (actually SCIA)
void serial_init(Serial *self)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = SERIAL_BAUDRATE;
  huart1.Init.WordLength = UART_WORDLENGTH_8B; // If parity is different than NONE, increase the length
  huart1.Init.StopBits = UART_STOPBITS_1;
  //if (SERIAL_PARITY == SERIAL_PARITY_NONE) huart1.Init.Parity = UART_PARITY_NONE;
  //if (SERIAL_PARITY == SERIAL_PARITY_ODD) huart1.Init.Parity = UART_PARITY_ODD;
  //huart1.Init.Parity = UART_PARITY_ODD;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

// Transmit variable data based on passed size
Uint16 serial_transmitData(Uint16 * data, Uint16 size)
{
  static unsigned short ongoingProcess = 0;
  Uint16 i;
  Uint16 j = 0;

  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);  // Enable writing the bus
//  HAL_Delay(1); //TODO Optimize it!!!

  // First entry
  if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY && ongoingProcess == 0)
  {
    for (i = 0; i < size; i++)
    {
      //msgModbusTxSlave[j] = data[i] >> 8;
      //j++;
      msgModbusTxSlave[j] = data[i] & 0xFF;
      j++;
    }
    HAL_UART_Abort(&huart1); //Stops and clears any pending process
    HAL_UART_AbortReceive_IT(&huart1);
    HAL_UART_AbortTransmit_IT(&huart1);
    HAL_UART_Transmit_IT(&huart1, (uint8_t*)msgModbusTxSlave, size);
    ongoingProcess = 1;
  }
  else if (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY && ongoingProcess == 1) return (0);
  else if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY && ongoingProcess == 1)
  {
    ongoingProcess = 0;
    return (size);
  }
  else return (0);

  return (0);
}

// Read data from buffer (byte per byte)
Uint16 serial_getRxBufferedWord()
{
  Uint16 retVal = msgModbusRxSlave[msgModbusRxBuffReadedSlave];
  msgModbusRxBuffReadedSlave++;
  return(retVal);
}

bool serial_getRxError(){
  if (HAL_UART_GetError(&huart1)) return true;
  return false;
}

// Construct the Serial Module
Serial construct_Serial(){
	Serial serial;

	serial.clear = serial_clear;
	serial.rxBufferStatus = serial_rxBufferStatus;
	serial.setSerialRxEnabled = serial_setSerialRxEnabled;
	serial.setSerialTxEnabled = serial_setSerialTxEnabled;
	serial.init = serial_init;
	serial.transmitData = serial_transmitData;
	serial.getRxBufferedWord = serial_getRxBufferedWord;
	serial.getRxError = serial_getRxError;

	serial.fifoWaitBuffer = 0;

	return serial;
}

//**** Serial for Master ****//

// Clear flags of overflow
void serial_clearMaster(){
  //buffers must be cleared
  //TXEMA  HAL_UART_Abort(&huart2); //Stops and clears any pending process
  //TXEMA   HAL_UART_AbortReceive_IT(&huart2);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);  // Enable listening the bus
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
//  HAL_UART_Receive_IT(&huart2, (uint8_t *)tinnyRxBufferMaster, 1);//MB_BUFFER_SIZE<<1); // Worst case, reserve all memory
  msgModbusRxBuffIndexMaster = 0;
  msgModbusRxBuffReadedMaster = 0;
}

// Get how much data is at the RX FIFO Buffer
Uint16 serial_rxBufferStatusMaster()
{
  return (msgModbusRxBuffIndexMaster - msgModbusRxBuffReadedMaster);
}

// Enable or disable RX (receiver)
void serial_setSerialRxEnabledMaster(bool status)
{
//TXEMA  HAL_UART_AbortTransmit_IT(&huart2);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);  // Enable listening the bus
  HAL_UART_Receive_IT(&huart2, (uint8_t *)tinnyRxBufferMaster, 1);//MB_BUFFER_SIZE<<1); // Worst case, reserve all memory
  msgModbusRxBuffIndexMaster = 0;
  msgModbusRxBuffReadedMaster = 0;
}

// Enable or disable TX (trasmiter)
void serial_setSerialTxEnabledMaster(bool status)
{
  //TXEMA   HAL_UART_AbortReceive_IT(&huart2);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);  // Enable listening the bus
}

// Initialize Serial (SCIC)
void serial_initMaster(Serial *self){
  huart2.Instance = USART2;
  huart2.Init.BaudRate = SERIAL_BAUDRATE;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;//UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  //if (SERIAL_PARITY == SERIAL_PARITY_NONE) huart1.Init.Parity = UART_PARITY_NONE;
  //if (SERIAL_PARITY == SERIAL_PARITY_ODD) huart1.Init.Parity = UART_PARITY_ODD;
  //huart2.Init.Parity = UART_PARITY_ODD;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

// Transmit variable data based on passed size
Uint16 serial_transmitDataMaster(Uint16 * data, Uint16 size){
  static unsigned short ongoingProcess = 0;
  Uint16 i;
  Uint16 j = 0;

  // First entry
  if (HAL_UART_GetState(&huart2) == HAL_UART_STATE_READY && ongoingProcess == 0)
  {
    for (i = 0; i < size; i++)
    {
      //msgModbusTxSlave[j] = data[i] >> 8;
      //j++;
      msgModbusTxMaster[j] = data[i] & 0xFF;
      j++;
    }
    HAL_UART_Transmit_IT(&huart2, (uint8_t*)msgModbusTxMaster, size);
    ongoingProcess = 1;
  }
  else if (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY && ongoingProcess == 1) return (0);
  else if (HAL_UART_GetState(&huart2) == HAL_UART_STATE_READY && ongoingProcess == 1)
  {
    ongoingProcess = 0;
    return (size);
  }
  else return (0);

  return (0);
}

// Read data from buffer (byte per byte)
Uint16 serial_getRxBufferedWordMaster()
{
  Uint16 retVal = msgModbusRxMaster[msgModbusRxBuffReadedMaster];
  msgModbusRxBuffReadedMaster++;
  return(retVal);
}

bool serial_getRxErrorMaster()
{
  if (HAL_UART_GetError(&huart2)) return true;
  return false;
}

// Construct the Serial Module
Serial construct_SerialMaster(){
    Serial serial;

    serial.clear = serial_clearMaster;
    serial.rxBufferStatus = serial_rxBufferStatusMaster;
    serial.setSerialRxEnabled = serial_setSerialRxEnabledMaster;
    serial.setSerialTxEnabled = serial_setSerialTxEnabledMaster;
    serial.init = serial_initMaster;
    serial.transmitData = serial_transmitDataMaster;
    serial.getRxBufferedWord = serial_getRxBufferedWordMaster;
    serial.getRxError = serial_getRxErrorMaster;

    serial.fifoWaitBuffer = 0;

    return serial;
}
