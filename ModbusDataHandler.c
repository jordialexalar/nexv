#include "ModbusDefinitions.h"
#include "ModbusDataHandler.h"
//#include "ModbusDataMap.h"
#include "ModbusSlave.h"
#include "ModbusLog.h"
#include "ModbusCrc.h"
#include "math.h"
#include "main.h"

#include "can_power_member.h"

void datahandler_readDigitalData(ModbusSlave *slave, ModbusFunctionCode funcCode) {
	// Each requested register has two bytes inside of it, so the code has to multiply by 2
	Uint16 firstAddr = (slave->dataRequest.content[MB_READ_ADDRESS_HIGH] << 8) |
			slave->dataRequest.content[MB_READ_ADDRESS_LOW];
	Uint16 totalData = (slave->dataRequest.content[MB_READ_TOTALDATA_HIGH] << 8) |
			slave->dataRequest.content[MB_READ_TOTALDATA_LOW];
	Uint16 numberOfBytes = ceil(totalData/8 + 0.5);

	Uint16 sizeWithoutCRC;
	Uint16 * transmitString;
	Uint16 idx, subIdx;
	Uint16 itemsToProcess;
	Uint16 padding = 0;
	Uint16 sizeOfMap = 0;
	unsigned short tmpSwap;

	// Reference to MODBUS Data Map
	char *dataPtr = (char *)&(slave->coils);

	// dataPtr points to the right struct (depending on passed funcCode)
	if (funcCode == MB_FUNC_READ_COIL){
		dataPtr = (char *)&(slave->coils);
		sizeOfMap = sizeof(slave->coils);
	}
#if MB_INPUTS_ENABLED
	else if(funcCode == MB_FUNC_READ_INPUT){
		dataPtr = (char *)&(slave->inputs);
		sizeOfMap = sizeof(slave->inputs);
	}
#endif

	if (slave->dataHandler.checkException(slave, firstAddr, totalData, sizeOfMap))
		return ;

	slave->dataResponse.slaveAddress = own_node;
	slave->dataResponse.functionCode = funcCode;

	// Number of Bytes
	slave->dataResponse.content[slave->dataResponse.contentIdx++] = numberOfBytes;

	// MODIFIABLE: specified address values
	for (idx = 0; idx < numberOfBytes; idx++) {
		tmpSwap = 0;
		itemsToProcess = 8;

		if (totalData < 8)
			itemsToProcess = totalData;

		for(subIdx = 0; subIdx < itemsToProcess; subIdx++) {
			padding = firstAddr + subIdx + idx*8;
			tmpSwap = (( *(dataPtr + padding) & 0x01 ) << subIdx) | tmpSwap;
		}

		slave->dataResponse.content[slave->dataResponse.contentIdx++] = tmpSwap;
		totalData = totalData - 8;
	}

	// Data response size based on total data requested
	// This size will be used at serial.transmitData internal loop
	slave->dataResponse.size = MB_SIZE_RESP_READ_MINIMUM + numberOfBytes;

	// Generate CRC
	sizeWithoutCRC = slave->dataResponse.size - 2;
	transmitString = slave->dataResponse.getTransmitStringWithoutCRC(&slave->dataResponse);
	slave->dataResponse.crc = generateCrc( transmitString, sizeWithoutCRC, true);

	MB_DATA_HANDLER_DEBUG();
}

void datahandler_readAnalogData(ModbusSlave *slave, ModbusFunctionCode funcCode) {
	// Each requested register has two bytes inside of it, so the code has to multiply by 2
	Uint16 firstAddr = (slave->dataRequest.content[MB_READ_ADDRESS_HIGH] << 8) |
			slave->dataRequest.content[MB_READ_ADDRESS_LOW];
	Uint16 totalData = (slave->dataRequest.content[MB_READ_TOTALDATA_HIGH] << 8) |
			slave->dataRequest.content[MB_READ_TOTALDATA_LOW];
	Uint16 firstAddrwPadding = 0;

	Uint16 sizeWithoutCRC;
	Uint16 * transmitString;
	Uint16 idx;
	Uint16 sizeOfMap = 0;
	Uint16 paddingMultip = 0;//sizeof(Uint16)>>1;

	// Reference to MODBUS Data Map
	Uint16 *registersPtr;
	if (funcCode == MB_FUNC_READ_HOLDINGREGISTERS){
		//registersPtr = (char *)&(slave->holdingRegisters);
		//sizeOfMap = sizeof(slave->holdingRegisters);
	    registersPtr = (Uint16 *)&(objectDictionary);
	    firstAddrwPadding = firstAddr;
	    if (firstAddr == 0x4919) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4919)); firstAddrwPadding = 0;}
	    if (firstAddr == 0x4e1e) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4e1e)); firstAddrwPadding = 0;}
	    if (firstAddr == 0x4e19) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4e19)); firstAddrwPadding = 0;}
	    if (firstAddr == 0x4930) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4930)); firstAddrwPadding = 0;}
	    if (firstAddr == 0x4931) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4931)); firstAddrwPadding = 0;}
	    if (firstAddr == 0x4932) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4932)); firstAddrwPadding = 0;}
	    sizeOfMap = MAXIMUM_OD_OBJECTS;
	}
#if MB_INPUT_REGISTERS_ENABLED
	else if(funcCode == MB_FUNC_READ_INPUTREGISTERS){
		//registersPtr = (char *)&(slave->inputRegisters);
		//sizeOfMap = sizeof(slave->inputRegisters);
	    registersPtr = (Uint16 *)&(objectDictionary);
	    firstAddrwPadding = firstAddr;
        if (firstAddr == 0x4919) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4919)); firstAddrwPadding = 0;}
        if (firstAddr == 0x4e1e) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4e1e)); firstAddrwPadding = 0;}
        if (firstAddr == 0x4e19) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4e19)); firstAddrwPadding = 0;}
        if (firstAddr == 0x4930) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4930)); firstAddrwPadding = 0;}
        if (firstAddr == 0x4931) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4931)); firstAddrwPadding = 0;}
        if (firstAddr == 0x4932) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4932)); firstAddrwPadding = 0;}
        sizeOfMap = MAXIMUM_OD_OBJECTS;
	}
#endif
  //unsigned int Prova = registersPtr;

	if (slave->dataHandler.checkException(slave, firstAddr, totalData, sizeOfMap))
		return ;

	slave->dataResponse.slaveAddress = own_node;
	slave->dataResponse.functionCode = funcCode;

	// Number of Bytes
	slave->dataResponse.content[slave->dataResponse.contentIdx++] = totalData * 2;

	// MODIFIABLE: Reads specified address values
	// The actual model works with 32 bits size registers. Adapt it to your needs.
	Uint16 padding;
	for (idx = 0; idx < totalData; idx++) {
		padding = idx + firstAddrwPadding;

#if MB_32_BITS_REGISTERS == true
		slave->dataResponse.content[slave->dataResponse.contentIdx++] = (*(registersPtr + padding + 1) & 0xFF00) >> 8;
		slave->dataResponse.content[slave->dataResponse.contentIdx++] = *(registersPtr + padding + 1) & 0x00FF;
		idx++;
		slave->dataResponse.content[slave->dataResponse.contentIdx++] = (*(registersPtr + padding) & 0xFF00) >> 8;
		slave->dataResponse.content[slave->dataResponse.contentIdx++] = *(registersPtr + padding) & 0x00FF;
#else
		slave->dataResponse.content[slave->dataResponse.contentIdx++] = (*(registersPtr + (padding<<paddingMultip)) & 0xFF00) >> 8;
		slave->dataResponse.content[slave->dataResponse.contentIdx++] = *(registersPtr + (padding<<paddingMultip)) & 0x00FF;
#endif
	}

	// Data response size based on total data requested
	// This size will be used at serial.transmitData internal loop
	slave->dataResponse.size = MB_SIZE_RESP_READ_MINIMUM + totalData * 2;

	// Generate CRC
	sizeWithoutCRC = slave->dataResponse.size - 2;
	transmitString = slave->dataResponse.getTransmitStringWithoutCRC(&slave->dataResponse);
	slave->dataResponse.crc = generateCrc( transmitString, sizeWithoutCRC, true);

	MB_DATA_HANDLER_DEBUG();
}

void datahandler_presetSingleRegister(ModbusSlave *slave){
	Uint16 firstAddr = (slave->dataRequest.content[MB_WRITE_ADDRESS_HIGH] << 8) |
				slave->dataRequest.content[MB_WRITE_ADDRESS_LOW];
	Uint16 dataValue = (slave->dataRequest.content[MB_WRITE_VALUE_HIGH] << 8) |
			slave->dataRequest.content[MB_WRITE_VALUE_LOW];
	Uint16 * transmitString;
	Uint16 sizeOfMap = MAXIMUM_OD_OBJECTS;
  Uint16 paddingMultip = 0;//sizeof(Uint16)>>1;
  Uint16 firstAddrwPadding = firstAddr;


	// Reference to MODBUS Data Map
    Uint16 * registersPtr;
	registersPtr = (Uint16 *)&(objectDictionary);
    if (firstAddr == 0x4919) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4919)); firstAddrwPadding = 0;}
    if (firstAddr == 0x4e1e) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4e1e)); firstAddrwPadding = 0;}
    if (firstAddr == 0x4e19) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4e19)); firstAddrwPadding = 0;}
    if (firstAddr == 0x4930) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4930)); firstAddrwPadding = 0;}
    if (firstAddr == 0x4931) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4931)); firstAddrwPadding = 0;}
    if (firstAddr == 0x4932) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4932)); firstAddrwPadding = 0;}
    /*Special commands*/
	//if (firstAddr == 0x4919 && dataValue ==  0x55AA) NVIC_SystemReset(); //Chao bambino -> It's bootloader time!

	if (slave->dataHandler.checkException(slave, firstAddr, 1, sizeOfMap))
		return ;

	// Set register value
	*(registersPtr + (firstAddrwPadding<<paddingMultip)) = dataValue;

	// Prepare the MODBUS Frame response
	slave->dataResponse.slaveAddress = own_node;
	slave->dataResponse.functionCode = MB_FUNC_WRITE_HOLDINGREGISTER;
	slave->dataResponse.content[slave->dataResponse.contentIdx++] = slave->dataRequest.content[MB_WRITE_ADDRESS_HIGH];
	slave->dataResponse.content[slave->dataResponse.contentIdx++] = slave->dataRequest.content[MB_WRITE_ADDRESS_LOW];
	slave->dataResponse.content[slave->dataResponse.contentIdx++] = (*(registersPtr + (firstAddrwPadding<<paddingMultip)) & 0xFF00) >> 8;
	slave->dataResponse.content[slave->dataResponse.contentIdx++] = *(registersPtr + (firstAddrwPadding<<paddingMultip)) & 0x00FF;

	// Data response size
	// This size will be used at serial.transmitData internal loop
	slave->dataResponse.size = MB_SIZE_RESP_WRITE;

	// Generate CRC
	transmitString = slave->dataResponse.getTransmitStringWithoutCRC(&slave->dataResponse);
	slave->dataResponse.crc = generateCrc( transmitString, MB_SIZE_RESP_WRITE_WITHOUTCRC, true);

	MB_DATA_HANDLER_DEBUG();
}

void datahandler_forceSingleCoil(ModbusSlave *slave){
	Uint16 firstAddr = (slave->dataRequest.content[MB_WRITE_ADDRESS_HIGH] << 8) |
				slave->dataRequest.content[MB_WRITE_ADDRESS_LOW];
	Uint16 dataValue = (slave->dataRequest.content[MB_WRITE_VALUE_HIGH] << 8) |
			slave->dataRequest.content[MB_WRITE_VALUE_LOW];
	Uint16 coilValueResponse;
	Uint16 * transmitString;
	Uint16 sizeOfMap = sizeof(slave->coils);

	// Reference to MODBUS Data Map
	char * registersPtr;
	registersPtr = (char *)&(slave->coils);

	if (slave->dataHandler.checkException(slave, firstAddr, 1, sizeOfMap))
		return ;

	// Extract the HIGH value when necessary
	if (dataValue == 0xFF00) dataValue = 1;

	// Set coil value
	*(registersPtr + firstAddr) = dataValue;

	// Prepare the MODBUS Frame response
	if( *(registersPtr + firstAddr) & 0x0001) coilValueResponse = 0xFF;
	else coilValueResponse = 0x00;

	slave->dataResponse.slaveAddress = own_node;
	slave->dataResponse.functionCode = MB_FUNC_FORCE_COIL;
	slave->dataResponse.content[slave->dataResponse.contentIdx++] = slave->dataRequest.content[MB_WRITE_ADDRESS_HIGH];
	slave->dataResponse.content[slave->dataResponse.contentIdx++] = slave->dataRequest.content[MB_WRITE_ADDRESS_LOW];
	slave->dataResponse.content[slave->dataResponse.contentIdx++] = coilValueResponse;
	slave->dataResponse.content[slave->dataResponse.contentIdx++] = 0x00;

	// Data response size
	// This size will be used at serial.transmitData internal loop
	slave->dataResponse.size = MB_SIZE_RESP_WRITE;

	// Generate CRC
	transmitString = slave->dataResponse.getTransmitStringWithoutCRC(&slave->dataResponse);
	slave->dataResponse.crc = generateCrc( transmitString, MB_SIZE_RESP_WRITE_WITHOUTCRC, true);

	MB_DATA_HANDLER_DEBUG();
}

void datahandler_presetMultipleRegisters(ModbusSlave *slave){
	Uint16 * memAddr;
	Uint16 idx;
	Uint16 firstAddr = (slave->dataRequest.content[MB_WRITE_N_ADDRESS_HIGH] << 8) |
				slave->dataRequest.content[MB_WRITE_N_ADDRESS_LOW];
	Uint16 * transmitString;
	Uint16 totalData = slave->dataRequest.content[MB_WRITE_N_BYTES] / 2;
	Uint16 sizeOfMap = MAXIMUM_OD_OBJECTS;
  Uint16 paddingMultip = 0;//sizeof(Uint16)>>1;
  Uint16 firstAddrwPadding = firstAddr;


	// Reference to MODBUS Data Map
  Uint16 * registersPtr;
	registersPtr = (Uint16 *)&(objectDictionary);
    if (firstAddr == 0x4919) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4919)); firstAddrwPadding = 0;}
    if (firstAddr == 0x4e1e) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4e1e)); firstAddrwPadding = 0;}
    if (firstAddr == 0x4e19) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4e19)); firstAddrwPadding = 0;}
    if (firstAddr == 0x4930) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4930)); firstAddrwPadding = 0;}
    if (firstAddr == 0x4931) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4931)); firstAddrwPadding = 0;}
    if (firstAddr == 0x4932) { registersPtr = (Uint16 *)(&(objectDictionarySpecialObject4932)); firstAddrwPadding = 0;}
	/*Special commands*/
	/*Uint16 dataValue = (slave->dataRequest.content[MB_WRITE_N_VALUES_START_HIGH] << 8) |
	                slave->dataRequest.content[MB_WRITE_N_VALUES_START_LOW];
    if (firstAddr == 0x4919 && dataValue ==  0x55AA) NVIC_SystemReset();*/ //Chao bambino -> It's bootloader time!

	// Checking for common errors
	if (slave->dataHandler.checkException(slave, firstAddr, totalData, sizeOfMap))
		return ;

	// Prepare response (it is the same thing of dataRequest, but you can do some checks at writen registers)
	slave->dataResponse.slaveAddress = own_node;
	slave->dataResponse.functionCode = MB_FUNC_WRITE_NREGISTERS;
	slave->dataResponse.content[slave->dataResponse.contentIdx++] = slave->dataRequest.content[MB_WRITE_N_ADDRESS_HIGH];
	slave->dataResponse.content[slave->dataResponse.contentIdx++] = slave->dataRequest.content[MB_WRITE_N_ADDRESS_LOW];
	slave->dataResponse.content[slave->dataResponse.contentIdx++] = slave->dataRequest.content[MB_WRITE_N_QUANTITY_HIGH];
	slave->dataResponse.content[slave->dataResponse.contentIdx++] = slave->dataRequest.content[MB_WRITE_N_QUANTITY_LOW];

	// MODIFIABLE: Writes values at specified address values
	for (idx = 0; idx < totalData; idx++) {
#if MB_32_BITS_REGISTERS
		// Used to invert the ptr to access the register. The values are saved in the format LOW|HIGH instead of HIGH|LOW
		// This tweak invert the idx to gave the right access based on the order of MODBUS
		Uint16 idxCorrection = idx^0x0001;
		Uint16 padding = idxCorrection + firstAddr;
		memAddr = (Uint16 *) (registersPtr + padding);

		*(memAddr) = (slave->dataRequest.content[MB_WRITE_N_VALUES_START_HIGH + idx*2] << 8) |
				slave->dataRequest.content[MB_WRITE_N_VALUES_START_LOW + idx*2];
#else
		Uint16 padding = idx + firstAddrwPadding;
		memAddr = (Uint16 *) (registersPtr + (padding<<paddingMultip));

		*(memAddr) = (slave->dataRequest.content[MB_WRITE_N_VALUES_START_HIGH + idx*2] << 8) |
				slave->dataRequest.content[MB_WRITE_N_VALUES_START_LOW + idx*2];
#endif
	}

	// Data response size
	// This size will be used at serial.transmitData internal loop
	slave->dataResponse.size = MB_SIZE_RESP_WRITE_N;

	// Generate CRC
	transmitString = slave->dataResponse.getTransmitStringWithoutCRC(&slave->dataResponse);
	slave->dataResponse.crc = generateCrc( transmitString, MB_SIZE_RESP_WRITE_N_WITHOUTCRC, true);

	MB_DATA_HANDLER_DEBUG();
}

void datahandler_forceMultipleCoils(ModbusSlave *slave){
	Uint16 idx;
	Uint16 * transmitString;
	Uint16 firstAddr = (slave->dataRequest.content[MB_WRITE_N_ADDRESS_HIGH] << 8) |
				slave->dataRequest.content[MB_WRITE_N_ADDRESS_LOW];
	Uint16 totalData = (slave->dataRequest.content[MB_WRITE_N_QUANTITY_HIGH] << 8) |
			slave->dataRequest.content[MB_WRITE_N_QUANTITY_LOW];
	Uint16 padding;
	Uint16 sizeOfMap = sizeof(slave->coils);


	// Reference to MODBUS Data Map
	char * coilsPtr;
	coilsPtr = (char *)&(slave->coils);

	// Checking for common errors
	if (slave->dataHandler.checkException(slave, firstAddr, totalData, sizeOfMap))
		return ;

	// Prepare response (it is the same thing of dataRequest, but you can do some checks at writen registers)
	slave->dataResponse.slaveAddress = own_node;
	slave->dataResponse.functionCode = MB_FUNC_FORCE_NCOILS;
	slave->dataResponse.content[slave->dataResponse.contentIdx++] = slave->dataRequest.content[MB_WRITE_N_ADDRESS_HIGH];
	slave->dataResponse.content[slave->dataResponse.contentIdx++] = slave->dataRequest.content[MB_WRITE_N_ADDRESS_LOW];
	slave->dataResponse.content[slave->dataResponse.contentIdx++] = slave->dataRequest.content[MB_WRITE_N_QUANTITY_HIGH];
	slave->dataResponse.content[slave->dataResponse.contentIdx++] = slave->dataRequest.content[MB_WRITE_N_QUANTITY_LOW];

	// MODIFIABLE: Writes values at specified address values
	for (idx = 0; idx < totalData; idx++) {
		padding = firstAddr + idx;
		*(coilsPtr + padding) = (slave->dataRequest.content[MB_WRITE_N_VALUES_START_HIGH + idx/8] >> idx%8) & 0x01;
	}

	// Data response size
	// This size will be used at serial.transmitData internal loop
	slave->dataResponse.size = MB_SIZE_RESP_WRITE_N;

	// Generate CRC
	transmitString = slave->dataResponse.getTransmitStringWithoutCRC(&slave->dataResponse);
	slave->dataResponse.crc = generateCrc( transmitString, MB_SIZE_RESP_WRITE_N_WITHOUTCRC, true);

	MB_DATA_HANDLER_DEBUG();
}

bool datahandler_checkException(ModbusSlave *slave, Uint16 firstAddr, Uint16 totalData, Uint16 sizeOfMap){
//	if( ( totalData == 0 ) || (totalData - 1 > MB_DATA_MAX_REQUEST_SIZE) )
  if (firstAddr < MAXIMUM_OD_OBJECTS)
  {
    if( ( totalData == 0 ) || (totalData > MAXIMUM_OD_OBJECTS) )
    {
        slave->dataHandler.exception(slave, MB_ERROR_ILLEGALDATA);
        return true;
    }
    if( ( firstAddr + totalData) > sizeOfMap)
    {
        slave->dataHandler.exception(slave, MB_ERROR_ILLEGALADDR);
        return true;
    }
  }
  else
  {
    if (firstAddr == 0x4919 && totalData == 1) return false;
    else if (firstAddr == 0x4e1e && totalData == 2) return false;
    else if (firstAddr == 0x4e19 && totalData == 1) return false;
    else if (firstAddr == 0x4930 && totalData == 1) return false;
    else if (firstAddr == 0x4931 && totalData == 1) return false;
    else
    {
      slave->dataHandler.exception(slave, MB_ERROR_ILLEGALDATA);
      return true;
    }
  }
  return false;
}

void datahandler_exception(ModbusSlave *slave, ModbusError exceptionCode){
	Uint16 * transmitString;

	slave->dataResponse.slaveAddress = own_node;
	slave->dataResponse.functionCode = slave->dataRequest.functionCode | 0x80; // Adds the signal of exception
	slave->dataResponse.content[slave->dataResponse.contentIdx++] = exceptionCode;

	slave->dataResponse.size = MB_SIZE_RESP_EXCEPTION;

	transmitString = slave->dataResponse.getTransmitStringWithoutCRC(&slave->dataResponse);
	slave->dataResponse.crc = generateCrc(transmitString, MB_SIZE_RESP_EXCEPTION_WITHOUTCRC, true);

	MB_DATA_HANDLER_DEBUG();
}

ModbusDataHandler construct_ModbusDataHandler(){
	ModbusDataHandler modbusDataHandler;

	modbusDataHandler.readDigitalData = datahandler_readDigitalData;
	modbusDataHandler.readAnalogData = datahandler_readAnalogData;

	modbusDataHandler.presetSingleRegister = datahandler_presetSingleRegister;
	modbusDataHandler.forceSingleCoil = datahandler_forceSingleCoil;
	modbusDataHandler.presetMultipleRegisters = datahandler_presetMultipleRegisters;
	modbusDataHandler.forceMultipleCoils = datahandler_forceMultipleCoils;

	modbusDataHandler.checkException = datahandler_checkException;
	modbusDataHandler.exception = datahandler_exception;

	MB_DATA_HANDLER_DEBUG();

	return modbusDataHandler;
}
