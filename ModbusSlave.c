#include "ModbusSlave.h"
#include "ModbusSettings.h"
#include "ModbusLog.h"

#include "can_power_member.h"


extern Uint32 timerModbusSlaveError;
extern Uint32 timerModbusMasterError;
#if DEBUG_UTILS_PROFILING
#include "Profiling.h"
ProfilingTool profiling;
#endif

void slave_loopStates(ModbusSlave *self){
	MB_SLAVE_DEBUG();
	switch (self->state) {
	case MB_CREATE:
		MB_SLAVE_DEBUG("State: MB_CREATE\n");
		self->create(self);
		break;
	case MB_START:
		MB_SLAVE_DEBUG("State: MB_START\n");
		self->start(self);
		break;
	case MB_TIMER_T35_WAIT:
		MB_SLAVE_DEBUG("State: MB_TIMER_T35_WAIT\n");
		self->timerT35Wait(self);
		break;
	case MB_RECEIVE:
		MB_SLAVE_DEBUG("State: MB_RECEIVE\n");
		self->receive(self);
		break;
	case MB_PROCESS:
		MB_SLAVE_DEBUG("State: MB_PROCESS\n");
		self->process(self);
		break;
	case MB_TRANSMIT:
		MB_SLAVE_DEBUG("State: MB_TRANSMIT\n");
		self->transmit(self);
		break;
	case MB_DESTROY:
		MB_SLAVE_DEBUG("State: MB_DESTROY\n");
		self->destroy(self);
		break;
	default:
	  self->state = MB_CREATE;
	  break;
	}
}

// STATE: MB_CREATE
// Initialize all needed peripherals (timer and serial mostly)
// Only executed once!
void slave_create(ModbusSlave *self){
	MB_SLAVE_DEBUG();

	// Configure Serial Port A
	self->serial.baudrate = SERIAL_BAUDRATE;
	self->serial.parityType = SERIAL_PARITY;
	self->serial.bitsNumber = SERIAL_BITS_NUMBER;
	self->serial.init(&self->serial);

	// Configure timer with the 3.5c time for timeout
	self->timer.init(&self->timer, mbT35);
	self->lastReceivedMsgModbusSlave = 0;

#if DEBUG_UTILS_PROFILING
	profiling = construct_ProfilingTool();
#endif

	self->state = MB_START;
}

// STATE: MB_START
// Reset all data from serial, data frames and timers
// Restart point to the lifecycle
void slave_start(ModbusSlave *self){
	MB_SLAVE_DEBUG();

	// Return the value to 3.5 char
	self->timer.resetTimer();
    self->timer.init(&self->timer, mbT35);
	
	self->dataRequest.clear(&self->dataRequest);
	self->dataResponse.clear(&self->dataResponse);

	self->serial.clear();
    self->timer.resetTimer();
	self->timer.start();
#if DEBUG_UTILS_PROFILING
	profiling.start(&profiling);
#endif

	self->state = MB_TIMER_T35_WAIT;
}

// STATE: MB_TIMER_T35_WAIT
// Waits until the timer expires (when timeout = 3.5c)
void slave_timerT35Wait(ModbusSlave *self){
	MB_SLAVE_DEBUG();

	if ( self->timer.expiredTimer(&self->timer) )
	{
      self->timer.stop();
      if (self->serial.rxBufferStatus() == 0)
      {
         self->state = MB_RECEIVE;
      }
      else
      {
        self->state = MB_START;
      }
	}
}

// STATE: MB_RECEIVE
// Will receive serial data and store at dataRequest var

ReceiveSubSM recvSubSm = RECEIVE_SM_CLEAR;

void slave_receive(ModbusSlave *self)
{
	// self->lastReceivedMsgModbusSlave = 0;
	switch(recvSubSm)
	{
	case RECEIVE_SM_CLEAR:
    self->timer.init(&self->timer, mbT15 * 2 );
    self->timer.resetTimer();
    self->timer.start();
    if( (self->serial.rxBufferStatus() > 0) || (self->serial.getRxError() == true) ) recvSubSm = RECEIVE_SM_NEW_WORD;
        //break;
	case RECEIVE_SM_NEW_WORD:
    // Wait to receive Slave ID and Function Code
      if( ( self->serial.rxBufferStatus() < 2 ) &&
              (self->serial.getRxError() == false ) ){
          if (self->timer.expiredTimer(&self->timer)){
              recvSubSm = RECEIVE_SM_CLEAR;
              self->serial.clear();//noise!
              self->serial.setSerialRxEnabled(true);
              self->timer.stop();
          }
      }
      else{
          // Check which function code it is to adjust the size of the RX FIFO buffer
          // In the case of function code 0x0010, it has a variable size
          self->dataRequest.slaveAddress = self->serial.getRxBufferedWord();
          self->dataRequest.functionCode = self->serial.getRxBufferedWord();
          self->timer.init(&self->timer, mbT15 * 6);
          self->timer.resetTimer();
          self->timer.start();
          recvSubSm = RECEIVE_SM_ADDRESS;
      }
      break;
	case RECEIVE_SM_ADDRESS:
	    // If the function code is for writing on multiple registers, then the FIFO Wait Buffer will not be fixed
      // Else it uses a fixed value (6 bytes)
      if (self->dataRequest.functionCode == MB_FUNC_WRITE_NREGISTERS ||
              self->dataRequest.functionCode == MB_FUNC_FORCE_NCOILS) {
          // Wait to receive the first address of registers and the quantity
          if ( ( self->serial.rxBufferStatus() < 5 ) &&
                  (self->serial.getRxError() == false ) ){
              if (self->timer.expiredTimer(&self->timer)){
                  recvSubSm = RECEIVE_SM_CLEAR; // Go back, lost in messages
                  self->state = MB_START;
              }
          }
          else{
              self->dataRequest.content[MB_WRITE_N_ADDRESS_HIGH] = self->serial.getRxBufferedWord();
              self->dataRequest.content[MB_WRITE_N_ADDRESS_LOW] = self->serial.getRxBufferedWord();
              self->dataRequest.content[MB_WRITE_N_QUANTITY_HIGH] = self->serial.getRxBufferedWord();
              self->dataRequest.content[MB_WRITE_N_QUANTITY_LOW] = self->serial.getRxBufferedWord();
              self->dataRequest.content[MB_WRITE_N_BYTES] = self->serial.getRxBufferedWord();

              self->dataRequest.contentIdx = 5;
              self->serial.fifoWaitBuffer = self->dataRequest.content[MB_WRITE_N_BYTES] + 2;

              // Waiting RX data until fifoWaitBuffer == 0
              // While waiting the data, it already empty the buffer, allowing to receive a lot of data
              self->timer.init(&self->timer, mbT15 * (self->serial.fifoWaitBuffer + 2));
              self->timer.resetTimer();
              self->timer.start();
              recvSubSm = RECEIVE_SM_DATA;
          }
      }
/*      else if (self->dataRequest.functionCode == MB_FUNC_READ_INPUTREGISTERS ||
          self->dataRequest.functionCode == MB_FUNC_READ_HOLDINGREGISTERS)
      {

      }*/
      else {
          self->serial.fifoWaitBuffer = 6;
          // Waiting RX data until fifoWaitBuffer == 0
          // While waiting the data, it already empty the buffer, allowing to receive a lot of data
          self->timer.init(&self->timer, mbT15 * (self->serial.fifoWaitBuffer + 2));
          self->timer.resetTimer();
          self->timer.start();
          recvSubSm = RECEIVE_SM_DATA;
      }
    break;
	case RECEIVE_SM_DATA:
		self->lastReceivedMsgModbusSlave = 0;
	    if ( (self->serial.fifoWaitBuffer > 0) &&
            (self->serial.getRxError() == false ) &&
            (self->timer.expiredTimer(&self->timer) == false) ){
            while(self->serial.rxBufferStatus() > 0) {
                self->dataRequest.content[self->dataRequest.contentIdx++] = self->serial.getRxBufferedWord();
                self->serial.fifoWaitBuffer--;
            }
        }
	    // Timer 1.5c timeout
        if (self->timer.expiredTimer(&self->timer)) {
            self->state = MB_START;
            recvSubSm = RECEIVE_SM_CLEAR;
            return ;
        }
        //That's all, no more data to receive
        if (self->serial.fifoWaitBuffer == 0){
            self->timer.stop();
            recvSubSm = RECEIVE_FINISHING;
        }
        break;
    case RECEIVE_FINISHING:
        // If there is any error on reception, it will go to the START state
        // Else it will get some data from the request
        if (self->serial.getRxError() == true){
            self->state = MB_START;
        } else {
            // Clears interruptions or overflow flags from serial
            self->serial.clear();

            // Configures the size of data request frame
            // It will be used in some loops on ModbusDataHandler
            // By default it will use the READ size, because it is always the same (independent of function code)
            if(self->dataRequest.functionCode == MB_FUNC_WRITE_NREGISTERS || self->dataRequest.functionCode == MB_FUNC_FORCE_NCOILS) {
                self->dataRequest.size = MB_SIZE_REQ_WRITE_N_MINIMUM + self->dataRequest.content[MB_WRITE_N_BYTES];
            } else if (self->dataRequest.functionCode == MB_FUNC_WRITE_HOLDINGREGISTER || self->dataRequest.functionCode == MB_FUNC_FORCE_COIL){
                self->dataRequest.size = MB_SIZE_REQ_WRITE;
            } else {
                self->dataRequest.size = MB_SIZE_REQ_READ;
            }

            self->state = MB_PROCESS;
            recvSubSm = RECEIVE_SM_CLEAR;
            //self->serial.setSerialTxEnabled(true);
        }

        #if DEBUG_UTILS_PROFILING
        profiling.registerStep(&profiling, profiling_MB_RECEIVE);
        #endif
	}
}

// STATE: MB_PROCESS
// Do the "magic". Check if the request have the right CRC and is for this device
// After first checks, it will begin the requested function code and prepare the dataResponse
void slave_process(ModbusSlave *self){
	Uint16 sizeWithoutCrc = self->dataRequest.size - 2;
	Uint16 generatedCrc;

	self->jumpProcessState = false;

	#if (MB_CHECKS == true)
	// Get the received CRC
	self->dataRequest.crc = (self->dataRequest.content[self->dataRequest.contentIdx - 2] << 8) |
			self->dataRequest.content[self->dataRequest.contentIdx - 1];
	// Generate CRC code based on received data
	generatedCrc = generateCrc(self->dataRequest.getTransmitStringWithoutCRC(&self->dataRequest),
			sizeWithoutCrc, true);

    // Requested slave address must be equal of pre-defined ID
    if (self->dataRequest.slaveAddress != own_node /*&& self->dataRequest.slaveAddress != 0*/){
        MB_SLAVE_DEBUG("Request is not for this device!");
        self->state = MB_START;
        self->jumpProcessState = true;
    }

	// Check if the received CRC is equal to locally generated CRC
	if (generatedCrc != self->dataRequest.crc && self->jumpProcessState == false) {
		MB_SLAVE_DEBUG("Error on CRC!");
		self->dataHandler.exception(self, MB_ERROR_ILLEGALDATA);
		self->state = MB_TRANSMIT;
        self->serial.setSerialTxEnabled(true);
		self->jumpProcessState = true;
	}


	#endif

	if (self->jumpProcessState == false) {
		// Check the function code and do some action using dataHandler
		if (self->dataRequest.functionCode == MB_FUNC_READ_COIL && MB_COILS_ENABLED){
			MB_SLAVE_DEBUG("Reading coils");
			self->dataHandler.readDigitalData(self, MB_FUNC_READ_COIL);
		}
		else if (self->dataRequest.functionCode == MB_FUNC_READ_HOLDINGREGISTERS && MB_HOLDING_REGISTERS_ENABLED){
			MB_SLAVE_DEBUG("Reading holding registers");
			self->dataHandler.readAnalogData(self, MB_FUNC_READ_HOLDINGREGISTERS);
		}
		else if (self->dataRequest.functionCode == MB_FUNC_WRITE_HOLDINGREGISTER && MB_HOLDING_REGISTERS_ENABLED){
			MB_SLAVE_DEBUG("Presetting holding register");
			self->dataHandler.presetSingleRegister(self);
		}
		/*else if (self->dataRequest.functionCode == MB_FUNC_FORCE_COIL && MB_COILS_ENABLED){
			MB_SLAVE_DEBUG("Forcing coil");
			self->dataHandler.forceSingleCoil(self);
		}*/
		else if (self->dataRequest.functionCode == MB_FUNC_WRITE_NREGISTERS && MB_HOLDING_REGISTERS_ENABLED){
			MB_SLAVE_DEBUG("Presetting multiple holding registers");
			self->dataHandler.presetMultipleRegisters(self);
		}
		/*else if (self->dataRequest.functionCode == MB_FUNC_FORCE_NCOILS && MB_COILS_ENABLED){
			MB_SLAVE_DEBUG("Forcing multiple coils");
			self->dataHandler.forceMultipleCoils(self);
		}*/
#if MB_INPUTS_ENABLED
		else if (self->dataRequest.functionCode == MB_FUNC_READ_INPUT && MB_INPUTS_ENABLED){
			MB_SLAVE_DEBUG("Reading inputs");
			self->dataHandler.readDigitalData(self, MB_FUNC_READ_INPUT);
		}
#endif
#if MB_INPUT_REGISTERS_ENABLED
		else if (self->dataRequest.functionCode == MB_FUNC_READ_INPUTREGISTERS && MB_INPUT_REGISTERS_ENABLED){
			MB_SLAVE_DEBUG("Reading input registers");
			self->dataHandler.readAnalogData(self, MB_FUNC_READ_INPUTREGISTERS);
		}
#endif
		else {
			MB_SLAVE_DEBUG("Exception: ILLEGALFUNC");
			self->dataHandler.exception(self, MB_ERROR_ILLEGALFUNC);
		}
		self->state = MB_TRANSMIT;
	    self->serial.setSerialTxEnabled(true);
	}

	if (self->dataRequest.slaveAddress == 0 || self->dataRequest.slaveAddress == MB_BROADCAST_EXTRA_ID){
		MB_SLAVE_DEBUG("Broadcast message - Jumping to MB_START");
		self->state = MB_START;
	}

	#if DEBUG_UTILS_PROFILING
	profiling.registerStep(&profiling, profiling_MB_PROCESS);
	#endif
}

// STATE: MB_TRANSMIT
// Transmit all data from dataResponse throught TX and then go back to MB_START
void slave_transmit(ModbusSlave *self){
	MB_SLAVE_DEBUG();

	if (self->serial.transmitData(self->dataResponse.getTransmitString(&self->dataResponse),
			self->dataResponse.size)) self->state = MB_START;


	#if DEBUG_UTILS_PROFILING
	profiling.registerStep(&profiling, profiling_MB_TRANSMIT);
	profiling.stop(&profiling);
	#endif
}

void slave_destroy(ModbusSlave *self){
	MB_SLAVE_DEBUG();
}

ModbusSlave construct_ModbusSlave(){
	ModbusSlave modbusSlave;

	MB_SLAVE_DEBUG();

	modbusSlave.state = MB_CREATE;
	modbusSlave.dataRequest = construct_ModbusData();
	modbusSlave.dataResponse = construct_ModbusData();
	modbusSlave.serial = construct_Serial();
	modbusSlave.timer = construct_Timer();

	modbusSlave.dataHandler = construct_ModbusDataHandler();
	modbusSlave.lastReceivedMsgModbusSlave = 0;
#if MB_COILS_ENABLED
	modbusSlave.coils = construct_ModbusCoilsMap();
#endif
#if MB_INPUTS_ENABLED
	modbusSlave.inputs = construct_ModbusInputsMap();
#endif
#if MB_HOLDING_REGISTERS_ENABLED
	//modbusSlave.holdingRegisters = construct_ModbusHoldingRegistersMap();
#endif
#if MB_INPUT_REGISTERS_ENABLED
	//modbusSlave.inputRegisters = construct_ModbusInputRegistersMap();
#endif
	modbusSlave.state = MB_CREATE;
	modbusSlave.loopStates = slave_loopStates;
	modbusSlave.create = slave_create;
	modbusSlave.start = slave_start;
	modbusSlave.timerT35Wait = slave_timerT35Wait;
	modbusSlave.receive = slave_receive;
	modbusSlave.process = slave_process;
	modbusSlave.transmit = slave_transmit;
	modbusSlave.destroy = slave_destroy;

	modbusSlave.jumpProcessState = false;

	return modbusSlave;
}
