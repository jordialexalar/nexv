#include "ModbusMaster.h"
#include "ModbusSettings.h"
#include "ModbusLog.h"

#if DEBUG_UTILS_PROFILING
#include "Profiling.h"
ProfilingTool profiling;
#endif

Uint64 ticks = 0;

void master_loopStates(ModbusMaster *self){
	MB_MASTER_DEBUG();
	switch (self->state) {
	case MB_CREATE:
		MB_MASTER_DEBUG("State: MB_MASTER_CREATE\n");
		self->create(self);
		break;
	case MB_START:
		MB_MASTER_DEBUG("State: MB_MASTER_START\n");
		self->start(self);
	case MB_END:
		MB_MASTER_DEBUG("State: MB_MASTER_START\n");
		self->start(self);
		break;
	case MB_WAIT:
		self->wait(self);
		break;
	case MB_REQUEST:
		MB_MASTER_DEBUG("State: MB_MASTER_REQUEST\n");
		self->request(self);
		break;
	case MB_RECEIVE:
		MB_MASTER_DEBUG("State: MB_MASTER_RECEIVE\n");
		self->receive(self);
		break;
	case MB_PROCESS:
		MB_MASTER_DEBUG("State: MB_MASTER_PROCESS\n");
		self->process(self);
		break;
	case MB_DESTROY:
		MB_MASTER_DEBUG("State: MB_MASTER_DESTROY\n");
		self->destroy(self);
		break;
	default:
	  self->state = MB_CREATE;
	  break;
	}
}

void master_create(ModbusMaster *self){
	MB_MASTER_DEBUG();

	// Configure Serial Port A
	self->serial.baudrate = SERIAL_BAUDRATE;
	self->serial.parityType = SERIAL_PARITY;
	self->serial.bitsNumber = SERIAL_BITS_NUMBER;
	self->serial.init(&self->serial);

	self->timer.init(&self->timer, MB_REQ_TIMEOUT);

#if DEBUG_UTILS_PROFILING
	profiling = construct_ProfilingTool();
#endif
	self->state = MB_START;
}

void master_start(ModbusMaster *self){
	MB_MASTER_DEBUG();
	
	self->dataRequest.clear(&self->dataRequest);
	self->dataResponse.clear(&self->dataResponse);

	self->serial.clear();
	self->timeout = false;

	self->timer.resetTimer();
	self->timer.setTimerReloadPeriod(&self->timer, MB_REQ_INTERVAL);
	self->timer.start();

	self->state = MB_WAIT;
}

void master_wait(ModbusMaster *self) {
	if (self->timer.expiredTimer(&self->timer)) {
		self->timer.stop();
		self->state = MB_REQUEST;
	}
}

Uint16 * transmitString;
void master_request(ModbusMaster *self){

	self -> serial.setSerialTxEnabled(true);
	// Wait until the code signals that the request is ready to transmit
	if (self->requester.isReady == false ) {return; }
	// Reset request ready signal
	self->requester.isReady = false;

	transmitString = self->dataResponse.getTransmitString(&self->dataRequest);
#if DEBUG_UTILS_PROFILING
	profiling.start(&profiling);
#endif
	if (self->serial.transmitData(transmitString, self->dataRequest.size))
	{
	  MB_MASTER_DEBUG();
	  self->state = MB_RECEIVE;
	  self->serial.setSerialRxEnabled(true);
	}
}

#include "main.h"
Uint32 initTimer = 0;
Uint32 lastTimer = 0;
Uint32 timerStatus = 0;
Uint32 timerStatusPrev = 0;

Uint32 bufferStatus = 0;
ReceiveSubSMMaster recvSubSmMaster = RECEIVE_SM_CLEAR_MASTER;


void master_receive(ModbusMaster *self){
	// self->lastReceivedMsgModbusMaster = 0;
  switch(recvSubSmMaster)
  {
    case RECEIVE_SM_CLEAR_MASTER:
      self->timer.init(&self->timer, MB_REQ_TIMEOUT);
      self->timer.resetTimer();
      self->timer.start();
      self->requestProcessed = false;
      recvSubSmMaster = RECEIVE_SM_NEW_WORD_MASTER;
      break;
    case RECEIVE_SM_NEW_WORD_MASTER:
      // Wait to have some data at the RX Buffer
      // If removed it can give errors on address and function code receiving
      if( ( self->serial.rxBufferStatus() < 2 ) &&
              (self->serial.getRxError() == false ) ){
          if (self->timer.expiredTimer(&self->timer)){
              recvSubSmMaster = RECEIVE_SM_CLEAR_MASTER;
              self->state = MB_END; //Timeout!
              self->serial.clear();//noise!
              self->serial.setSerialRxEnabled(true);
              self->timer.stop();
          }
      }
      else{
          // Check which function code it is to adjust the size of the RX FIFO buffer
          // In the case of function code 0x0010, it has a variable size
          self->dataResponse.slaveAddress = self->serial.getRxBufferedWord();
          self->dataResponse.functionCode = self->serial.getRxBufferedWord();
          recvSubSmMaster = RECEIVE_SM_ADDRESS_MASTER;
      }
      break;
    case RECEIVE_SM_ADDRESS_MASTER:
      // If the function code is for writing on multiple registers, then the FIFO Wait Buffer will not be fixed
      // Else it uses a fixed value (6 bytes)
      // Prepare the buffer size
      if (self->dataRequest.functionCode == MB_FUNC_READ_HOLDINGREGISTERS) {
        self->serial.fifoWaitBuffer = MB_SIZE_COMMON_DATA_WITHOUTCRC;
        self->serial.fifoWaitBuffer += self->dataRequest.content[3] * 2;  //requested bytes
        self->serial.fifoWaitBuffer += 1; //Byte count
      }
      else {
        self->serial.fifoWaitBuffer = MB_SIZE_RESP_WRITE;
      }
      recvSubSmMaster = RECEIVE_SM_DATA_MASTER;
      break;
    case RECEIVE_SM_DATA_MASTER:
    	self->lastReceivedMsgModbusMaster = 0;
      if ( ( self->serial.fifoWaitBuffer > 0 )
        && ( self->serial.getRxError() == false )
        && ( self->timer.expiredTimer(&self->timer) == false )
      )
      {
        while(self->serial.rxBufferStatus() > 0) {
          self->dataResponse.content[self->dataResponse.contentIdx++] = self->serial.getRxBufferedWord();
          self->serial.fifoWaitBuffer--;
        }
      }
      if (self->serial.fifoWaitBuffer == 0){
        recvSubSmMaster = RECEIVE_FINISHING_MASTER;
      }
      if (self->timer.expiredTimer(&self->timer) == true )
      {
        recvSubSmMaster = RECEIVE_SM_CLEAR_MASTER;
        self->state = MB_END; //Timeout!
      }
      break;
    case RECEIVE_FINISHING_MASTER:
      // Receive the CRC
      if ( ( self->serial.rxBufferStatus() < 2 )
          && ( self->serial.getRxError() == false )
          && ( self->timer.expiredTimer(&self->timer) == false )
        )
      {
        break;
      }
      self->dataResponse.crc = (self->serial.getRxBufferedWord() << 8) | self->serial.getRxBufferedWord();

      self->timer.stop();

      // Jump to START if there is any problem with the basic info
      if (self->dataResponse.slaveAddress != self->dataRequest.slaveAddress ||
          self->dataResponse.functionCode != self->dataRequest.functionCode ) {
        recvSubSmMaster = RECEIVE_SM_CLEAR_MASTER;
        self->state = MB_END;
        break ;
      }

      // If there is any error on Reception, it will go to the START state
      if (self->serial.getRxError() == true || self->timer.expiredTimer(&self->timer))
      {
        recvSubSmMaster = RECEIVE_SM_CLEAR_MASTER;
        self->state = MB_END;
      }
      else
      {
        recvSubSmMaster = RECEIVE_SM_CLEAR_MASTER;
        self->state = MB_PROCESS;
      }
      break;
  }
  return;
}

void master_process (ModbusMaster *self){
	self->requester.save(self);

	self->requestProcessed = true;

	self->state = MB_END;
}

void master_destroy(ModbusMaster *self){
	MB_MASTER_DEBUG();
}

ModbusMaster construct_ModbusMaster(){
	ModbusMaster modbusMaster;

	MB_MASTER_DEBUG();
	modbusMaster.lastReceivedMsgModbusMaster = 0;
	modbusMaster.state = MB_CREATE;
	modbusMaster.dataRequest = construct_ModbusData();
	modbusMaster.dataResponse = construct_ModbusData();
	modbusMaster.requester = construct_ModbusRequestHandler();
	modbusMaster.serial = construct_SerialMaster();
	modbusMaster.timer = construct_TimerMaster();

	modbusMaster.timeoutCounter = 0;
	modbusMaster.successfulRequests = 0;
	modbusMaster.requestReady = false;
	modbusMaster.requestProcessed = false;

#if MB_COILS_ENABLED
	modbusMaster.coils = construct_ModbusCoilsMap();
#endif
#if MB_INPUTS_ENABLED
	modbusMaster.inputs = construct_ModbusInputsMap();
#endif
#if MB_HOLDING_REGISTERS_ENABLED
	modbusMaster.holdingRegisters = construct_ModbusHoldingRegistersMap();
#endif
#if MB_INPUT_REGISTERS_ENABLED
	modbusMaster.inputRegisters = construct_ModbusInputRegistersMap();
#endif

	modbusMaster.loopStates = master_loopStates;
	modbusMaster.create = master_create;
	modbusMaster.start = master_start;
	modbusMaster.wait = master_wait;
	modbusMaster.request = master_request;
	modbusMaster.receive = master_receive;
	modbusMaster.process = master_process;
	modbusMaster.destroy = master_destroy;

	return modbusMaster;
}
