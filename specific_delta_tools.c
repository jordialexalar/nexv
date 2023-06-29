/*
 * specific_delta_tools.c
 *
 *  Created on: 16 ago. 2018
 *      Author: josepmaria.fernandez
 */
#include "main.h"
#include "specific_delta.h"     // Header file Include File
#include "can_power_member.h"
#include "EventList.h"

unsigned long                   errorDx[7];
unsigned int                    globalNodeDiscoverySemaphore            =   GREEN_SEM;
unsigned int                    nodeList[MAXIMUM_NUMBER_OF_CONVERTERS];
unsigned int                    discoveredModules;
unsigned long                   activeOutputRelaysVector = 0, activeOutputRelaysVectorLast = 0, discoveredBaysWithModule = 0;
SerialNodeRegister              serialNodeRegister[MAXIMUM_NUMBER_OF_CONVERTERS];
DeltaModule                     deltaConverter[MAXIMUM_NUMBER_OF_CONVERTERS]; //Declara la variable en la mem�ria llunyana
DeltaModule                     nullConverter;
Uint32                          systemEpoch = 1; /* iniciamos a uno para que ningin evento se cree en tiempo cero */
Uint32                          auxSystemEpoch = 0;

canid_t messageData[] = {TEMPERATURE_1_SEND_MASK_ID,TEMPERATURE_2_SEND_MASK_ID,TEMPERATURE_3_SEND_MASK_ID,TEMPERATURE_4_SEND_MASK_ID,
		OPERATETIME_1_SEND_MASK_ID,OPERATETIME_2_SEND_MASK_ID,OPERATETIME_3_SEND_MASK_ID,GET_INPUT_VOLTAGE_REQUEST_MASK_ID,
		GET_INPUT_CURRENT_REQUEST_MASK_ID,GET_INPUT_POWER_REQUEST_MASK_ID,FAN_SPEED_SEND_MASK_ID,PHYADDRESS_SEND_MASK_ID};

#define LIMIT_MESSAGE_DATA                       12

//--- DELTA MODULE CONSTRUCTOR
DeltaModule construct_DeltaData(){
    DeltaModule     deltaModule;

    //Inicializaci�n de las variables:
    deltaModule.pings                   = 0;
    deltaModule.commandWord             = 0;
    deltaModule.statusWord              = 0;
    deltaModule.currentSetPoint         = 0;
    deltaModule.voltageSetPoint         = 0;
    deltaModule.outputCurrent           = 0;
    deltaModule.outputVoltage           = 0;
    deltaModule.outputPower             = 0;
    deltaModule.availableCurrent        = 0;
    deltaModule.chargedEnergy           = 1;
    deltaModule.sessionEnergy           = 0;

    //Converter specific:
    deltaModule.powerSetPoint           = 0;
    deltaModule.temperaturePrimary      = 0;
    deltaModule.temperatureSecondary    = 0;
    deltaModule.temperatureAir          = 0;
    deltaModule.rpm1                    = 0;
    deltaModule.rpm2                    = 0;
    deltaModule.identifier              = 0;
    deltaModule.position                = 0;
    deltaModule.phyAddr                 = 0;
    deltaModule.desiredPhyAddr          = 0;
    deltaModule.protocoloAsignado       = 0;
    int indexSN;
    for (indexSN = 0; indexSN < 16; indexSN++) {
        deltaModule.serialNumber[indexSN] = 0;
    }
    for (indexSN = 0; indexSN < 12; indexSN++) {
        deltaModule.version[indexSN] = 0;
    }

    deltaModule.errorDataWord           = 0;

    //Construction of the auxiliary structures
    deltaModule.dataHandler             = construct_DeltaDataHandler();
    deltaModule.can                     = construct_DeltaCan();
    deltaModule.energy                  = construct_DeltaEnergy();
    deltaModule.machine                 = construct_DeltaStateMachine();
    deltaModule.errorCounter            = construct_DeltaErrorCounter();

    deltaModule.limits.tempHigh         = MAXIMUM_DELTA_TEMP;
    deltaModule.limits.tempLow          = MINIMUM_DELTA_TEMP;
    deltaModule.limits.vMax             = MAXIMUM_DELTA_VOLTAGE;
    deltaModule.limits.iMax             = MAXIMUM_DELTA_CURRENT;
    deltaModule.limits.pMax             = MAXIMUM_DELTA_POWER;

    // Clear all lists
    int indexNodo;
    for (indexNodo = 0; indexNodo < MAXIMUM_NUMBER_OF_CONVERTERS; indexNodo++) {
        nodeList[indexNodo] = 0;
    };



    // Clear all serial nodes
    for (indexNodo = 0; indexNodo < MAXIMUM_NUMBER_OF_CONVERTERS; indexNodo++) {
        serialNodeRegister[indexNodo].serialNumberPart[0]   = 0;
        serialNodeRegister[indexNodo].serialNumberPart[1]   = 0;
        serialNodeRegister[indexNodo].serialNumberPart[2]   = 0;
        serialNodeRegister[indexNodo].serialNumberPart[3]   = 0;
        serialNodeRegister[indexNodo].serialNumberPart[4]   = 0;
        serialNodeRegister[indexNodo].serialNumberPart[5]   = 0;
        serialNodeRegister[indexNodo].serialNumberPart[6]   = 0;
        serialNodeRegister[indexNodo].serialNumberPart[7]   = 0;
        serialNodeRegister[indexNodo].serialNumberPart[8]   = 0;
        serialNodeRegister[indexNodo].serialNumberPart[9]   = 0;
        serialNodeRegister[indexNodo].serialNumberPart[10]   = 0;
        serialNodeRegister[indexNodo].serialNumberPart[11]   = 0;
        serialNodeRegister[indexNodo].serialNumberPart[12]   = 0;
        serialNodeRegister[indexNodo].serialNumberPart[13]   = 0;
        serialNodeRegister[indexNodo].serialNumberPart[14]   = 0;
        serialNodeRegister[indexNodo].serialNumberPart[15]   = 0;

        serialNodeRegister[indexNodo].tray                  = 0;
    };
    //---   Inicializa el vector de salidas habilitadas
    activeOutputRelaysVector        = 0;
    activeOutputRelaysVectorLast    = 0;

    return (deltaModule);
}


//--- DATAHANDLER CONSTRUCTOR ---//
// Esta estructura y sus funciones se encargar�n de traducir los valores guardados en los
// registros a valores entendibles por el convertidor y a la inversa
DeltaDataHandler construct_DeltaDataHandler(void){
    DeltaDataHandler deltaDataHandler;

    deltaDataHandler.getVoltageCurrentStatus    = datahandler_getVoltageCurrentStatus;
    deltaDataHandler.getTemperatures            = datahandler_getTemperatures;
    deltaDataHandler.getSerialNumber            = datahandler_getSerialNuber;
    deltaDataHandler.writeControlCommand        = datahandler_writeControlCommand;
    deltaDataHandler.writeVoltageCurrentPower   = datahandler_writeVoltageCurrentPower;
    deltaDataHandler.newNodeDiscovered          = datahandler_newNodeDiscovered;
    deltaDataHandler.manageReceivedPing         = datahandler_manageReceivedPing;
    deltaDataHandler.getRPMs                    = datahandler_getRPMs;
    deltaDataHandler.getInputs                  = datahandler_getInputs;
    deltaDataHandler.getCurrent                 = datahandler_getCurrent;
    deltaDataHandler.getOperateTime             = datahandler_getOperateTime;
    deltaDataHandler.getVersion                 = datahandler_getVersion;
    deltaDataHandler.getPhyAddr                 = datahandler_getPhyAddr;
    return (deltaDataHandler);
}

void datahandler_getVoltageCurrentStatus(DeltaModule *self, struct can_frame *message){
	EventLocation event;
    int    outputVoltageCoded          = 0, outputCurrentCoded         = 0;
    short int v = 0;

    float           outputVoltageDecodedFloat   = 0.0, outputCurrentDecodedFloat  = 0.0;

//    unsigned long   rmmainStatus                = 0;

    // Check if the received message is for this function
    if (((message->can_id & 0xFFFFFF00) == VOLTAGE_CURRENT_STATUS_READ_MASK_ID)  || ((message->can_id & 0xFFFFFF00) == OUTPUT_VOLTAGE_READ_MASK_ID))
    {
        /* voltage */
    	v = CAN_2BYTES_TO_UINT16(message->data[1],message->data[0]);
    	outputVoltageCoded = (int) v;


    	outputVoltageDecodedFloat   = (float) outputVoltageCoded;
        self -> outputVoltage       = outputVoltageDecodedFloat * 0.1;

        /* current */
        v =  CAN_2BYTES_TO_UINT16(message->data[3],message->data[2]);

        outputCurrentCoded = (int) v;

        outputCurrentDecodedFloat   = (float)outputCurrentCoded;


        self -> outputCurrent   = outputCurrentDecodedFloat * 0.1;


	self->errorDataWord     = CAN_2BYTES_TO_UINT16(message->data[7],message->data[6]);

        // Decode the satus word
	/*
        if (self->errorDataWord  >=  0x00000800){} //0x08** codes external errors from the cabinet that affect the converter
        else
        {
            //self->errorDataWord     = ((unsigned long)message->Data.Words.Word3);
        	self->errorDataWord     = CAN_2BYTES_TO_UINT16(message->data[6],message->data[7]);
        }
*/
        //rmsubStatus             = self->errorDataWord & 0x000000FF;           //Aplica la m�scara y elimina el resto
        //rmmainStatus            = (self->errorDataWord & 0x00000F00) >> 8;    //Aplica m�scara y desplaza bits
	// self->errorDataWord = self->errorDataWord >> 16;
        self->statusWord        = 0;

/*
        if ((self->errorDataWord & 0x0010) == 0x0010){
          self->statusWord |= STANDBY_MODE_STATUS;
        }

        if ((self->errorDataWord & 0x0020) == 0x0020){
          self->statusWord |= SLEEP_MODE_STATUS;
        }

        if ((self->errorDataWord & 0x0004) == 0x0004){
          self->statusWord |= IN_DERATING_STATUS;
        }

        if ((self->errorDataWord & 0x0040) == 0x0040){
          self->statusWord |= SWITCHED_OFF_STATUS;
        }
        else{
          self->statusWord |= SWITCHED_ON_STATUS;

        }

*/

            //  Identification of enable signal for every power module
            if((self->errorDataWord & DELTA_STATUS_PS_KILLED) == DELTA_STATUS_PS_KILLED)
            {
                self->statusWord = SWITCHED_OFF_STATUS;
            }
            else
            {
                self->statusWord = SWITCHED_ON_STATUS;
            }

            if((self->errorDataWord & DELTA_STATUS_SERIES_MODE) == DELTA_STATUS_SERIES_MODE)
            {
            }

            if((self->errorDataWord & DELTA_STATUS_NORMAL_WORKING) == DELTA_STATUS_NORMAL_WORKING)
            {
                self->statusWord |= CHARGING_ON_STATUS;
            }

            if((self->errorDataWord & DELTA_STATUS_OUTPUT_DERATED) == DELTA_STATUS_OUTPUT_DERATED)
            {
                self->statusWord |= CHARGING_ON_STATUS;
            }

            if((self->errorDataWord & DELTA_STATUS_LOAD_SHORT_LATCH) == DELTA_STATUS_LOAD_SHORT_LATCH)
            {
                self->statusWord |= ERROR_STATUS;
                self->statusWord |= SHORT_LATCH_STATUS;
            }

            if((self->errorDataWord & DELTA_STATUS_CAN_TIMEOUT) == DELTA_STATUS_CAN_TIMEOUT)
            {
                self->statusWord  |= WARNING_STATUS;
            }

            if((self->errorDataWord & DELTA_STATUS_VAC_EXCEPTION) == DELTA_STATUS_VAC_EXCEPTION)
            {
                self->statusWord  |= WARNING_STATUS;
            }

            if((self->errorDataWord & DELTA_STATUS_TEMPERATURE_EXCEPTION) == DELTA_STATUS_TEMPERATURE_EXCEPTION)
            {
                self->statusWord  |= WARNING_STATUS;
            }

            if((self->errorDataWord & DELTA_STATUS_OTHER_EXCEPTION) == DELTA_STATUS_OTHER_EXCEPTION)
            {
                self->statusWord  |= WARNING_STATUS;
            }

            //  Error
            if((self->errorDataWord & DELTA_STATUS_SCI_FAIL) == DELTA_STATUS_SCI_FAIL)
            {
                self->statusWord  |= ERROR_STATUS;
            }

            if((self->errorDataWord & DELTA_STATUS_FAN_FAIL) == DELTA_STATUS_FAN_FAIL)
            {
                self->statusWord  |= ERROR_STATUS;
            }

            if((self->errorDataWord & DELTA_STATUS_DISCHARGE_FAIL) == DELTA_STATUS_DISCHARGE_FAIL)
            {
                self->statusWord  |= ERROR_STATUS;
            }

            if((self->errorDataWord & DELTA_STATUS_OTHER_FAILURE) == DELTA_STATUS_OTHER_FAILURE)
            {
                self->statusWord  |= ERROR_STATUS;
            }

            if((self->errorDataWord & DELTA_STATUS_UPDATING_FIRMWARE) == DELTA_STATUS_UPDATING_FIRMWARE)
            {
                self->statusWord  |= ERROR_STATUS;
            }

            //  Estados de la maquina de potencia. Si hay error, convertidor en error tambien
            if (self->machine.estadoActualConvertidor >= 700)
            {
                self->statusWord |= ERROR_STATUS;
            }

    }
#if	DEBUG_ERROR_DELTA
    if (errorDx[self->yi] != self->errorDataWord){

    			sprintf(bufferstr, "Delta %i: %i \n", (int) self->yi, (int) self->errorDataWord);
    			env_debug(bufferstr);
    			env_debug("\n");
    		    errorDx[self->yi] = self->errorDataWord;
    	  }
#endif





    //New: valora si hace falta guardar el registro:

    if ((self->statusWord & ERROR_STATUS) == ERROR_STATUS)
    {
        self->errorDataWordStd = errorCodeDelta22Circontrol(self->errorDataWord);
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_ERROR, self->errorDataWordStd);
        self->machine.pushErrorWord(self, &event);
    }

};  //END OF datahandler_getVoltageCurrentStatus


//------------------------------------------------------------------------------------------------------//
//  Funcion getInputs                                                                                             //
//------------------------------------------------------------------------------------------------------//

void datahandler_getInputs(DeltaModule *self, struct can_frame *message)
{
    if ((message->can_id & 0xFFFFFF00) == GET_INPUT_VOLTAGE_READ_MASK_ID)
    {
        self->Vr                    = CAN_2BYTES_TO_UINT16(message->data[1],message->data[0]);
        self->Vs                    = CAN_2BYTES_TO_UINT16(message->data[3],message->data[2]);
        self->Vt                    = CAN_2BYTES_TO_UINT16(message->data[5],message->data[4]);
        self->Freq                  = CAN_2BYTES_TO_UINT16(message->data[7],message->data[6]);
    }
    else if ((message->can_id & 0xFFFFFF00) == GET_INPUT_CURRENT_READ_MASK_ID)
    {
        self->Ir                    = CAN_2BYTES_TO_UINT16(message->data[1],message->data[0]);
        self->Is                    = CAN_2BYTES_TO_UINT16(message->data[3],message->data[2]);
        self->It                    = CAN_2BYTES_TO_UINT16(message->data[5],message->data[4]);
    }
    else if ((message->can_id & 0xFFFFFF00) == GET_INPUT_POWER_READ_MASK_ID)
    {
        self->Pr                    = CAN_2BYTES_TO_UINT16(message->data[1],message->data[0]);
        self->Ps                    = CAN_2BYTES_TO_UINT16(message->data[3],message->data[2]);
        self->Pt                    = CAN_2BYTES_TO_UINT16(message->data[5],message->data[4]);
    }
}




// This function is focused on translating the "current request" message
void datahandler_getCurrent(DeltaModule *self, struct can_frame *message){
	  int       outputCurrentCoded         = 0;
	  short int v = 0;

	     v =  CAN_2BYTES_TO_UINT16(message->data[3],message->data[2]);
	      outputCurrentCoded = (int) v;

	      self -> outputCurrent   = (float) (outputCurrentCoded) * 0.1;

};




void datahandler_getTemperatures(DeltaModule *self, struct can_frame *message){

    if ((message->can_id & 0xFFFFFF00)      == TEMPERATURE_1_READ_MASK_ID){
        //self->temperaturePrimary    = message->Data.Words.Word0 * 0.1;
    	self->temperatureAir    = CAN_2BYTES_TO_UINT16(message->data[1],message->data[0]) * 0.1;
    	self->tempPFC1    	= CAN_2BYTES_TO_UINT16(message->data[3],message->data[2]) * 0.1;
    	self->tempPFC2    	= CAN_2BYTES_TO_UINT16(message->data[5],message->data[4]) * 0.1;
    }
    if ((message->can_id & 0xFFFFFF00)      == TEMPERATURE_2_READ_MASK_ID){
        //self->temperaturePrimary    = message->Data.Words.Word0 * 0.1;
    	self->tempLlc1Pri    = CAN_2BYTES_TO_UINT16(message->data[1],message->data[0]) * 0.1;
    	self->tempLlc2Pri    = CAN_2BYTES_TO_UINT16(message->data[3],message->data[2]) * 0.1;
    	self->tempLlc1Sec    = CAN_2BYTES_TO_UINT16(message->data[5],message->data[4]) * 0.1;
    	self->tempLlc2Sec    = CAN_2BYTES_TO_UINT16(message->data[7],message->data[6]) * 0.1;
   }
    if ((message->can_id & 0xFFFFFF00)      == TEMPERATURE_3_READ_MASK_ID){
        //self->temperaturePrimary    = message->Data.Words.Word0 * 0.1;
    	self->tempBuckA    	= CAN_2BYTES_TO_UINT16(message->data[1],message->data[0]) * 0.1;
    	self->tempBuckB    	= CAN_2BYTES_TO_UINT16(message->data[3],message->data[2]) * 0.1;
    	self->tempRelayParaA    = CAN_2BYTES_TO_UINT16(message->data[5],message->data[4]) * 0.1;
    	self->tempRelayParaB    = CAN_2BYTES_TO_UINT16(message->data[7],message->data[6]) * 0.1;
   }
    if ((message->can_id & 0xFFFFFF00)      == TEMPERATURE_4_READ_MASK_ID){
        //self->temperaturePrimary    = message->Data.Words.Word0 * 0.1;
    	self->tempRelaySer      = CAN_2BYTES_TO_UINT16(message->data[1],message->data[0]) * 0.1;
    	self->tempIloadShunt    = CAN_2BYTES_TO_UINT16(message->data[3],message->data[2]) * 0.1;
    	self->tempOutlet        = CAN_2BYTES_TO_UINT16(message->data[5],message->data[4]) * 0.1;

   }


};

void datahandler_getVersion(DeltaModule *self, struct can_frame *message){

    if ((message->can_id & 0xFFFFFF00)      == VERSION_1_READ_MASK_ID){
    	self->version[0] = CAN_4BYTES_TO_UINT32(message->data[0],message->data[1],message->data[2],message->data[3]);
    	self->version[1] = CAN_4BYTES_TO_UINT32(message->data[4],message->data[5],message->data[6],message->data[7]);

    }
    if ((message->can_id & 0xFFFFFF00)      == VERSION_2_READ_MASK_ID){
    	self->version[2] = CAN_4BYTES_TO_UINT32(message->data[0],message->data[1],message->data[2],message->data[3]);
    	self->version[3] = CAN_4BYTES_TO_UINT32(message->data[4],message->data[5],message->data[6],message->data[7]);
    }
    if ((message->can_id & 0xFFFFFF00)      == VERSION_3_READ_MASK_ID){
    	self->version[4] = CAN_4BYTES_TO_UINT32(message->data[0],message->data[1],message->data[2],message->data[3]);
    	self->version[5] = CAN_4BYTES_TO_UINT32(message->data[4],message->data[5],message->data[6],message->data[7]);
    }
    if ((message->can_id & 0xFFFFFF00)      == VERSION_4_READ_MASK_ID){
    	self->version[6] = CAN_4BYTES_TO_UINT32(message->data[0],message->data[1],message->data[2],message->data[3]);
    	self->version[7] = CAN_4BYTES_TO_UINT32(message->data[4],message->data[5],message->data[6],message->data[7]);

    }
    if ((message->can_id & 0xFFFFFF00)      == VERSION_5_READ_MASK_ID){
    	self->version[8] = CAN_4BYTES_TO_UINT32(message->data[0],message->data[1],message->data[2],message->data[3]);
    	self->version[9] = CAN_4BYTES_TO_UINT32(message->data[4],message->data[5],message->data[6],message->data[7]);
    }
    if ((message->can_id & 0xFFFFFF00)      == VERSION_6_READ_MASK_ID){
    	self->version[10] = CAN_4BYTES_TO_UINT32(message->data[0],message->data[1],message->data[2],message->data[3]);
    	self->version[11] = CAN_4BYTES_TO_UINT32(message->data[4],message->data[5],message->data[6],message->data[7]);
    }

};

void datahandler_getOperateTime(DeltaModule *self, struct can_frame *message){

    if ((message->can_id & 0xFFFFFF00)      == OPERATETIME_1_READ_MASK_ID){
    	self->sleepTime   	= CAN_4BYTES_TO_UINT32(message->data[3],message->data[2],message->data[1],message->data[0]);
    	self->standbyTime  	= CAN_4BYTES_TO_UINT32(message->data[7],message->data[6],message->data[5],message->data[4]);

    }
    if ((message->can_id & 0xFFFFFF00)      == OPERATETIME_2_READ_MASK_ID){
    	self->pload25   	= CAN_4BYTES_TO_UINT32(message->data[3],message->data[2],message->data[1],message->data[0]);
    	self->pload50  	=    CAN_4BYTES_TO_UINT32(message->data[7],message->data[6],message->data[5],message->data[4]);
   }
    if ((message->can_id & 0xFFFFFF00)      == OPERATETIME_3_READ_MASK_ID){
    	self->pload75   	= CAN_4BYTES_TO_UINT32(message->data[3],message->data[2],message->data[1],message->data[0]);
    	self->pload100  	= CAN_4BYTES_TO_UINT32(message->data[7],message->data[6],message->data[5],message->data[4]);
   }

};

void datahandler_getSerialNuber(DeltaModule *self, struct can_frame *message) {

    if ((message->can_id & 0xFFFFFF00)    == MODULE_SERIAL_NUMBER_1_READ_MASK_ID)
    {
        //self->serialNumber[0] = message->Data.DWords.DWord0;
        //self->serialNumber[1] = message->Data.DWords.DWord1;
    	self->serialNumber[0] = CAN_4BYTES_TO_UINT32(message->data[0],message->data[1],message->data[2],message->data[3]);
    	self->serialNumber[1] = CAN_4BYTES_TO_UINT32(message->data[4],message->data[5],message->data[6],message->data[7]);
        self->machine.serialNumberReceivedPartFlag |=   0x01;   // Activates the first flag of the received serial
    }
    if ((message->can_id & 0xFFFFFF00)    == MODULE_SERIAL_NUMBER_2_READ_MASK_ID)
    {
        //self->serialNumber[2] = message->Data.DWords.DWord0;
        //self->serialNumber[3] = message->Data.DWords.DWord1;
    	self->serialNumber[2] = CAN_4BYTES_TO_UINT32(message->data[0],message->data[1],message->data[2],message->data[3]);
    	self->serialNumber[3] = CAN_4BYTES_TO_UINT32(message->data[4],message->data[5],message->data[6],message->data[7]);
        self->machine.serialNumberReceivedPartFlag |=   0x02;   // Activates the second flag of the received serial
    }
    if ((message->can_id & 0xFFFFFF00)    == MODULE_SERIAL_NUMBER_3_READ_MASK_ID)
    {
        //self->serialNumber[4] =  message->Data.DWords.DWord0;
        //self->serialNumber[5] =  message->Data.DWords.DWord1;
    	self->serialNumber[4] = CAN_4BYTES_TO_UINT32(message->data[0],message->data[1],message->data[2],message->data[3]);
    	self->serialNumber[5] = CAN_4BYTES_TO_UINT32(message->data[4],message->data[5],message->data[6],message->data[7]);
        self->machine.serialNumberReceivedPartFlag |=   0x04;   // Activates the third flag of the received serial
    }
    if ((message->can_id & 0xFFFFFF00)    == MODULE_SERIAL_NUMBER_4_READ_MASK_ID)
    {
        //self->serialNumber[0] = message->Data.DWords.DWord0;
        //self->serialNumber[1] = message->Data.DWords.DWord1;
    	self->serialNumber[6] = CAN_4BYTES_TO_UINT32(message->data[0],message->data[1],message->data[2],message->data[3]);
    	self->serialNumber[7] = CAN_4BYTES_TO_UINT32(message->data[4],message->data[5],message->data[6],message->data[7]);
        self->machine.serialNumberReceivedPartFlag |=   0x08;   // Activates the first flag of the received serial
    }
    if ((message->can_id & 0xFFFFFF00)    == MODULE_SERIAL_NUMBER_5_READ_MASK_ID)
    {
        //self->serialNumber[2] = message->Data.DWords.DWord0;
        //self->serialNumber[3] = message->Data.DWords.DWord1;
    	self->serialNumber[8] = CAN_4BYTES_TO_UINT32(message->data[0],message->data[1],message->data[2],message->data[3]);
    	self->serialNumber[9] = CAN_4BYTES_TO_UINT32(message->data[4],message->data[5],message->data[6],message->data[7]);
        self->machine.serialNumberReceivedPartFlag |=   0x10;   // Activates the second flag of the received serial
    }
    if ((message->can_id & 0xFFFFFF00)    == MODULE_SERIAL_NUMBER_6_READ_MASK_ID)
    {
        //self->serialNumber[4] =  message->Data.DWords.DWord0;
        //self->serialNumber[5] =  message->Data.DWords.DWord1;
    	self->serialNumber[10] = CAN_4BYTES_TO_UINT32(message->data[0],message->data[1],message->data[2],message->data[3]);
    	self->serialNumber[11] = CAN_4BYTES_TO_UINT32(message->data[4],message->data[5],message->data[6],message->data[7]);
        self->machine.serialNumberReceivedPartFlag |=   0x20;   // Activates the third flag of the received serial
    }
    if ((message->can_id & 0xFFFFFF00)    == MODULE_SERIAL_NUMBER_7_READ_MASK_ID)
    {
        //self->serialNumber[2] = message->Data.DWords.DWord0;
        //self->serialNumber[3] = message->Data.DWords.DWord1;
    	self->serialNumber[12] = CAN_4BYTES_TO_UINT32(message->data[0],message->data[1],message->data[2],message->data[3]);
    	self->serialNumber[13] = CAN_4BYTES_TO_UINT32(message->data[4],message->data[5],message->data[6],message->data[7]);
        self->machine.serialNumberReceivedPartFlag |=   0x40;   // Activates the second flag of the received serial
    }
    if ((message->can_id & 0xFFFFFF00)    == MODULE_SERIAL_NUMBER_8_READ_MASK_ID)
    {
        //self->serialNumber[4] =  message->Data.DWords.DWord0;
        //self->serialNumber[5] =  message->Data.DWords.DWord1;
    	self->serialNumber[14] = CAN_4BYTES_TO_UINT32(message->data[0],message->data[1],message->data[2],message->data[3]);
    	self->serialNumber[15] = CAN_4BYTES_TO_UINT32(message->data[4],message->data[5],message->data[6],message->data[7]);
        self->machine.serialNumberReceivedPartFlag |=   0x80;   // Activates the third flag of the received serial
    }
};

void datahandler_getPhyAddr(DeltaModule *self, struct can_frame *message){

    if ((message->can_id & 0xFFFFFF00)      == PHYADDRESS_READ_MASK_ID){

    	// self->phyAddr   	= message->data[0];
    	self->phyAddr   	= self->yi + 1;
    }

};


void datahandler_newNodeDiscovered (DeltaModule *self) {

    //TODO, LATER ON
};

void datahandler_manageReceivedPing (DeltaModule *self, struct can_frame *message){
    self->machine.pong = 1;
    self->pings--;
};


void datahandler_getRPMs(DeltaModule *self, struct can_frame *message){


        //self->rpm1 = (float)(message->Data.Words.Word0);
        self->rpm1 = (float)CAN_2BYTES_TO_UINT16(message->data[1],message->data[0]);

        //self->rpm2 = (float)(message->Data.Words.Word0);
    	self->rpm2 = (float)CAN_2BYTES_TO_UINT16(message->data[3],message->data[2]);

};

struct can_frame datahandler_writeControlCommand(DeltaModule *self, const unsigned int theCommand){
    struct can_frame message ;

    message.can_id          = MODULE_REMOTE_ON_OFF_WRITE_MASK_ID  + self->identifier;
    message.can_dlc          = 1;
    //message.Data.DWords.DWord0  = 0; // Clear
    //message.Data.DWords.DWord1  = 0; // Clear
    //message.Data.Bytes.Byte0    = (unsigned char)theCommand; // Set*/
    message.data[0] = (unsigned char)theCommand;


    return(message);
};


struct can_frame datahandler_writeVoltageCurrentPower(DeltaModule *self){
    struct can_frame message ;
    unsigned int voltageWord = 0,    currentWord = 0,    powerWord   = 0;
    if (self->currentSetPoint == 0.0 && applyMinCurrent == 1){
    	self->currentSetPoint = minCurrent;
    }



    voltageWord     = ((unsigned int)(self->voltageSetPoint * 10.0));    //ALERTA, est� suponiendo que los valores estan guardados en *0.1V/bit
    currentWord     = ((unsigned int)(self->currentSetPoint * 10.0));    //ALERTA, est� suponiendo que los valores estan guardados en *0.1A/bit
    powerWord       = ((unsigned int)((self->powerSetPoint  * 10.0)/1000.0));
    message.can_id     = VCMD_ICMD_PCMD_WRITE_MASK_ID  + self->identifier;
    message.can_dlc     = 8;
    /*message.Data.Words.Word0    = voltageWord;
    message.Data.Words.Word1    = currentWord;
    message.Data.Words.Word2    = powerWord;
    message.Data.Words.Word3    = 0;*/
    message.data[0] = ((int)(voltageWord)) & 0xFF;
    message.data[1] = ((int)(voltageWord)) >> 8;
    message.data[2] = ((int)(currentWord)) & 0xFF;
    message.data[3] = ((int)(currentWord)) >> 8;
    message.data[4] = ((int)(powerWord)) & 0xFF;
    message.data[5] = ((int)(powerWord)) >> 8;
    message.data[6] = 0;
    message.data[7] = 0;

    return(message);
};

//--- CAN CONSTRUCTOR ---//
// Esta estructura y sus funciones se encargar�n de enviar directamente los mensajes por CAN (si se puede)
DeltaCan construct_DeltaCan(){
    DeltaCan deltaCan;

    deltaCan.pollReceived                       = deltacan_pollReceived;
    deltaCan.sendControlCommand                 = deltacan_sendControlCommand;
    deltaCan.sendVoltageCurrentPower            = deltacan_sendVoltageCurrentPower;
    deltaCan.sendSystemRestart                  = deltacan_sendSystemRestart;


    deltaCan.sendSnapStep1                      = deltacan_sendSnapStep1;
    deltaCan.sendSnapStep2                      = deltacan_sendSnapStep2;
    deltaCan.sendSnapStep3                      = deltacan_sendSnapStep3;
    deltaCan.sendSnapStep4                      = deltacan_sendSnapStep4;
    deltaCan.sendRequestId                      = deltacan_sendRequestId;
    deltaCan.sendPing                           = deltacan_sendPing;
    deltaCan.sendRequestData               		= deltacan_sendRequestData;


    deltaCan.sendRequestSerial1                 = deltacan_sendRequestSerial1;
    deltaCan.sendRequestVersion                 = deltacan_sendRequestVersion;
    deltaCan.sendRequestSerial3                 = deltacan_sendRequestSerial3;
    deltaCan.sendAssignId                       = deltacan_sendAssignId;
    deltaCan.sendRequestPhyAddr                 = deltacan_sendRequestPhyAddr;
    deltaCan.sendWritePhyAddr                   = deltacan_sendWritePhyAddr;
    deltaCan.sendRequestForCurrent              = deltacan_sendRequestForCurrent;
    deltaCan.sendRequestForVoltage              = deltacan_sendRequestForVoltage;
    deltaCan.sendRequestForPower                = deltacan_sendRequestForPower;

    deltaCan.sendRequestVersion                 = deltacan_sendRequestVersion;

    return (deltaCan);
}

unsigned int deltacan_pollReceived(DeltaModule *self, struct can_frame *message){
    unsigned int checked   = 0;
    unsigned long msgId   = message->can_id;
    const unsigned long nodeId  = self->identifier;

    // First of all, subtract the node number from the node Id. If this message
    if      ((msgId & 0x000000FF) == nodeId){
	msgId = msgId - nodeId;
    }
    // If the ID points this node, process the message.
    checked = 1;
    switch (msgId)
    {
        case (VOLTAGE_CURRENT_STATUS_READ_MASK_ID): self->dataHandler.getVoltageCurrentStatus(self,message);    break;  //Obtiene las variables del mensaje y las guarda en su sitio
        case (TEMPERATURE_1_READ_MASK_ID):          self->dataHandler.getTemperatures(self,message);            break;
        case (TEMPERATURE_2_READ_MASK_ID):          self->dataHandler.getTemperatures(self,message);            break;
        case (TEMPERATURE_3_READ_MASK_ID):          self->dataHandler.getTemperatures(self,message);            break;
        case (TEMPERATURE_4_READ_MASK_ID):          self->dataHandler.getTemperatures(self,message);            break;
        case (OPERATETIME_1_READ_MASK_ID):          self->dataHandler.getOperateTime(self,message);            break;
        case (OPERATETIME_2_READ_MASK_ID):          self->dataHandler.getOperateTime(self,message);            break;
        case (OPERATETIME_3_READ_MASK_ID):          self->dataHandler.getOperateTime(self,message);            break;
        case (MODULE_SERIAL_NUMBER_1_READ_MASK_ID): self->dataHandler.getSerialNumber(self,message);            break;
        case (MODULE_SERIAL_NUMBER_2_READ_MASK_ID): self->dataHandler.getSerialNumber(self,message);            break;
        case (MODULE_SERIAL_NUMBER_3_READ_MASK_ID): self->dataHandler.getSerialNumber(self,message);            break;
        case (MODULE_SERIAL_NUMBER_4_READ_MASK_ID): self->dataHandler.getSerialNumber(self,message);            break;
        case (MODULE_SERIAL_NUMBER_5_READ_MASK_ID): self->dataHandler.getSerialNumber(self,message);            break;
        case (MODULE_SERIAL_NUMBER_6_READ_MASK_ID): self->dataHandler.getSerialNumber(self,message);            break;
        case (MODULE_SERIAL_NUMBER_7_READ_MASK_ID): self->dataHandler.getSerialNumber(self,message);            break;
        case (MODULE_SERIAL_NUMBER_8_READ_MASK_ID): self->dataHandler.getSerialNumber(self,message);            break;
        case (VERSION_1_READ_MASK_ID): 				self->dataHandler.getVersion(self,message);                 break;
        case (VERSION_2_READ_MASK_ID):              self->dataHandler.getVersion(self,message);                 break;
        case (VERSION_3_READ_MASK_ID): 				self->dataHandler.getVersion(self,message);                 break;
        case (VERSION_4_READ_MASK_ID): 				self->dataHandler.getVersion(self,message);                 break;
        case (VERSION_5_READ_MASK_ID): 				self->dataHandler.getVersion(self,message);                 break;
        case (VERSION_6_READ_MASK_ID): 				self->dataHandler.getVersion(self,message);                 break;
        case (PHYADDRESS_READ_MASK_ID): 			self->dataHandler.getPhyAddr(self,message);                 break;
        case (PING_ACKNOWLEDGE_READ_MASK_ID):       self->dataHandler.manageReceivedPing(self,message);         break;
//       case (EN_4_READ_MASK_ID): 				self->dataHandler.getVersion(self,message);                 break;
//      case (LEAVE_MONITOR_STATE_READ_MASK_ID):                                                                break; // �s el mateix que "ENTER_MONITOR_STATE_READ_MASK_ID"
        case (FAN_SPEED_READ_MASK_ID):              self->dataHandler.getRPMs(self,message);                    break;

        case (GET_INPUT_VOLTAGE_READ_MASK_ID):      self->dataHandler.getInputs(self,message);                 break;
        case (GET_INPUT_CURRENT_READ_MASK_ID):      self->dataHandler.getInputs(self,message);                 break;
        case (GET_INPUT_POWER_READ_MASK_ID):        self->dataHandler.getInputs(self,message);                 break;
        case (OUTPUT_VOLTAGE_READ_MASK_ID):         self->dataHandler.getVoltageCurrentStatus(self,message);   break;

        default:                                    checked = 0;                                                break;
    };

    // Node assignment, special case
    if      ((msgId == REQUEST_MASTER_READ_MASK_ID) && self->machine.structureSeekingConverter && (globalNodeDiscoverySemaphore == GREEN_SEM))// GREEN_SEM = 1;
    {
        self->machine.converterFound = 1;    //  Set the flag of node discovered
        globalNodeDiscoverySemaphore = RED_SEM;     //  Block the semaphore, no other structures can be found until this process finishes.
        checked = 1;        //  Message is valid
    };



    if     ( (msgId & 0xFFFFFF00) == RESPONSE_ID_WRITE_MASK_ID )
    {

	if (self->machine.converterFound )
    {
        // Accept the node auto-numbering
         self->identifier = msgId & 0x000000FF;
	 //   self->identifier =  self->yi+1;
        checked = 1;        //  Message is valid
    }
    };
    return (checked);
};


unsigned int deltacan_sendControlCommand(DeltaModule *self, const unsigned int theCommand){
    struct can_frame message;

    message.can_id          = MODULE_REMOTE_ON_OFF_WRITE_MASK_ID + self->identifier;
    message.can_dlc          = 4;
    //message.Data.Bytes.Byte0    = theCommand;
    message.data[0]    = theCommand;
    message.data[1]    = 0x0;
    message.data[2]    = 0x0;
    message.data[3]    = 0x0;
    if(sendCanDelta(&message)){
        self->machine.cmdShadowCopy = self->commandWord;
        return (1);
    }

    return (0);
};


unsigned int deltacan_sendVoltageCurrentPower(DeltaModule *self){
    struct can_frame mensaje;

    mensaje = self->dataHandler.writeVoltageCurrentPower(self);

    //Keep a copy of the last sent data. It's useful to send messages only on changes.
    self->machine.currentShadowCopy = self->currentSetPoint;
    self->machine.voltageShadowCopy = self->voltageSetPoint;
			#if SIMUL_DELTAS
    deltasSIMUL[self->yi].outputCurrent = self->currentSetPoint;
    deltasSIMUL[self->yi].outputVoltage = self->voltageSetPoint;
	#endif

    return (sendCanDelta(&mensaje)); // Return 1 if the message is added to the buffer.
};


unsigned int deltacan_sendSystemRestart(DeltaModule *self){
    struct can_frame message;

    message.can_id          = SYSTEM_RESTART_WRITE_MASK_ID ; //No node identifier, it is right
    message.can_dlc          = 0;

    return (sendCanDelta(&message)); // Return 1 if the message is added to the buffer.
};



unsigned int deltacan_sendSnapStep1(DeltaModule *self){
    struct can_frame message;

    message.can_id          = SNAP_STEP_1_WRITE_MASK_ID;
    message.can_dlc          = 0;

    return (sendCanDelta(&message)); // Return 1 if the message is added to the buffer.
};


unsigned int deltacan_sendSnapStep2(DeltaModule *self){
    struct can_frame message;

    message.can_id          = SNAP_STEP_2_WRITE_MASK_ID;
    message.can_dlc          = 0;

    return (sendCanDelta(&message)); // Return 1 if the message is added to the buffer.
};


unsigned int deltacan_sendSnapStep3(DeltaModule *self){
    struct can_frame message;

    message.can_id          = SNAP_STEP_3_WRITE_MASK_ID;
    message.can_dlc          = 0;

    return (sendCanDelta(&message)); // Return 1 if the message is added to the buffer.
};


unsigned int deltacan_sendSnapStep4(DeltaModule *self){
    struct can_frame message;

    message.can_id          = SNAP_STEP_4_WRITE_MASK_ID;
    message.can_dlc          = 0;

    return (sendCanDelta(&message)); // Return 1 if the message is added to the buffer.
};


unsigned int deltacan_sendRequestId(DeltaModule *self){
    struct can_frame message;

    message.can_id          = REQUEST_ID_WRITE_MASK_ID;
    message.can_dlc          = 0;
#if SIMUL_DELTAS
	deltasSIMUL[self->yi].msgIdSimul = RESPONSE_ID_WRITE_MASK_IDSIMUL;
#endif
    return (sendCanDelta(&message)); // Return 1 if the message is added to the buffer.
};


unsigned int deltacan_sendAssignId(DeltaModule *self){
    struct can_frame message;

    message.can_id          = ASSIGN_ID_WRITE_MASK_ID + self->identifier;
    message.can_dlc          = 0;

    return (sendCanDelta(&message)); // Return 1 if the message is added to the buffer.
};





unsigned int deltacan_sendPing(DeltaModule *self){
    struct can_frame message;

    message.can_id          = PING_WRITE_MASK_ID + self->identifier;
    message.can_dlc          = 2;
    /*message.Data.Bytes.Byte0    = 0x1F;
    message.Data.Bytes.Byte1    = 0x9C;*/
    message.data[0] = 0x1F;
    message.data[1] = 0x9C;

    return (sendCanDelta(&message)); // Return 1 if the message is added to the buffer.
};

unsigned int deltacan_sendRequestData(DeltaModule *self){
    struct can_frame message;

    message.can_id          = messageData[self->machine.num_data] + self->identifier;
    message.can_dlc          = 0;
	self->machine.num_data++;
    if (self->machine.num_data == LIMIT_MESSAGE_DATA){
        self->machine.num_data = 0;
    }

    return (sendCanDelta(&message)); // Return 1 if the message is added to the buffer.
};

unsigned int deltacan_sendRequestPhyAddr(DeltaModule *self){
    struct can_frame message;

    message.can_id          = PHYADDRESS_SEND_MASK_ID + self->identifier;
    message.can_dlc          = 0;
#if SIMUL_DELTAS
	deltasSIMUL[self->yi].msgIdSimul = PHYADDRESS_READ_MASK_IDSIMUL;
#endif
    return (sendCanDelta(&message)); // Return 1 if the message is added to the buffer.
};


unsigned int deltacan_sendWritePhyAddr(DeltaModule *self){
    struct can_frame message;

    message.can_id          = PHYADDRESS_WRITE_MASK_ID + self->identifier;
    message.can_dlc          = 1;

    message.data[0] = self->desiredPhyAddr;

    return (sendCanDelta(&message)); // Return 1 if the message is added to the buffer.

};


unsigned int deltacan_sendRequestOperateTime(DeltaModule *self){
    struct can_frame message;

    message.can_id          = OPERATETIME_1_SEND_MASK_ID + ((unsigned int) 0x100 * self->machine.num_serie) + self->identifier;
    message.can_dlc          = 0;

    return (sendCanDelta(&message)); // Return 1 if the message is added to the buffer.
};








unsigned int deltacan_sendRequestSerial1(DeltaModule *self){
    struct can_frame message;

    message.can_id          = MODULE_SERIAL_NUMBER_1_SEND_MASK_ID + ((unsigned int) 0x100 * self->machine.num_serie) + self->identifier;
    message.can_dlc          = 0;

#if SIMUL_DELTAS
    deltasSIMUL[self->yi].msgIdSimul = MODULE_SERIAL_NUMBER_1_READ_MASK_IDSIMUL + (self->machine.num_serie);
#endif


    return (sendCanDelta(&message)); // Return 1 if the message is added to the buffer.
};


unsigned int deltacan_sendRequestVersion(DeltaModule *self){
    struct can_frame message;

    message.can_id          = VERSION_1_REQUEST_MASK_ID + ((unsigned int) 0x100 * self->machine.num_version) + self->identifier;
    message.can_dlc          = 0;

    return (sendCanDelta(&message)); // Return 1 if the message is added to the buffer.
};


unsigned int deltacan_sendRequestSerial3(DeltaModule *self){
    struct can_frame message;

    message.can_id          = MODULE_SERIAL_NUMBER_3_SEND_MASK_ID + self->identifier;
    message.can_dlc          = 0;
			#if SIMUL_DELTAS
    deltasSIMUL[self->yi].msgIdSimul = MODULE_SERIAL_NUMBER_3_READ_MASK_IDSIMUL;
	#endif
    return (sendCanDelta(&message)); // Return 1 if the message is added to the buffer.
};


unsigned int deltacan_sendRequestForCurrent(DeltaModule *self){
    struct can_frame message;

    message.can_id          = OUTPUT_VOLTAGE_REQUEST_MASK_ID + self->identifier;
    message.can_dlc          = 0;

    return (sendCanDelta(&message)); // Return 1 if the message is added to the buffer.
};


unsigned int deltacan_sendRequestForVoltage(DeltaModule *self){
    struct can_frame message;

    message.can_id          = OUTPUT_VOLTAGE_REQUEST_MASK_ID + self->identifier;
    message.can_dlc          = 0;
#if SIMUL_DELTAS
	deltasSIMUL[self->yi].msgIdSimul = OUTPUT_VOLTAGE_READ_MASK_IDSIMUL;
#endif

    return (sendCanDelta(&message)); // Return 1 if the message is added to the buffer.
};

unsigned int deltacan_sendRequestForPower(DeltaModule *self){
    struct can_frame message;

    message.can_id          = OUTPUT_VOLTAGE_REQUEST_MASK_ID + self->identifier;
    message.can_dlc          = 0;

    return (sendCanDelta(&message)); // Return 1 if the message is added to the buffer.
};




//--- ENERGY CONSTRUCTOR ---//
DeltaEnergy construct_DeltaEnergy(void){
    DeltaEnergy deltaEnergy;

    deltaEnergy.powerFloat              = 0.0;
    deltaEnergy.energyPendingToIncrement = 0.0;

    deltaEnergy.calcOutputPower         = deltaenergy_calcOutputPower;
    deltaEnergy.calcAvailableCurrent    = deltaenergy_calcAvailableCurrent;
    deltaEnergy.integrateEnergy         = deltaenergy_integrateEnergy;
    deltaEnergy.getEnergyLastSessions   = deltaenergy_getEnergyLastSessions;
    deltaEnergy.storeEnergyThisSession  = deltaenergy_storeEnergyThisSession;

    return(deltaEnergy);
};


void deltaenergy_calcOutputPower(DeltaModule *self){
    self->energy.powerFloat             = self->outputCurrent * self->outputVoltage ;
    self->outputPower                   = self->energy.powerFloat;
};

// The following function must be called every second during operation time.
void deltaenergy_integrateEnergy(DeltaModule *self){
    float energyIncrement               = self->energy.powerFloat * 2.777777777777778e-4; //Equivalent to divide by 3600. Energy integration is made every second.

    self->energy.energyPendingToIncrement       = self->energy.energyPendingToIncrement + energyIncrement;
    long integerPartOfPendingToSave              = (long)self->energy.energyPendingToIncrement;

    if (integerPartOfPendingToSave > 0) {//Exists an integer part to accumulate
        self->chargedEnergy                     += integerPartOfPendingToSave;
        self->sessionEnergy                     += integerPartOfPendingToSave;
        self->energy.energyPendingToIncrement   = self->energy.energyPendingToIncrement - (float)integerPartOfPendingToSave;
    }
    self->operationTime++;
};


void deltaenergy_calcAvailableCurrent(DeltaModule *self){
    float   maxCurrentAtGivenVoltage    = 0.0;

    //Avoids division by 0
    if (self->outputVoltage > 50.0F) maxCurrentAtGivenVoltage = self->limits.pMax / self->outputVoltage;
    else maxCurrentAtGivenVoltage       = self->limits.iMax;

    self->availableCurrent = (maxCurrentAtGivenVoltage > self->limits.iMax) ?  self->limits.iMax :  maxCurrentAtGivenVoltage;

    if ((self->statusWord &  ERROR_STATUS ) == ERROR_STATUS)    self->availableCurrent = 0.0; //  In case of error, there is no available current
    // Known derating
    // When there is a warning, the converter may be at its maximum. When this condition occurs, it is not convenient to push
    // the requested current beyond the current value. If the converter can provide a slightly more current, it will do so until
    // the maximum is reached. That is the reason for the "+0.1F". It will inform the it can provide a bit more than the requested.
    if ((self->statusWord &  WARNING_STATUS ) == WARNING_STATUS) self -> availableCurrent = self -> outputCurrent + 0.1F;
    if (self->machine.estadoActualConvertidor < 100 || self->machine.estadoActualConvertidor >= 700) self->availableCurrent = 0.0; // todo: Gemma guide. Stop states.
};


void deltaenergy_getEnergyLastSessions(DeltaModule *self){

    // Will require EEPROM
  loadStoredParams(self->position);

};

int deltaenergy_storeEnergyThisSession(DeltaModule *self){

  prepareParamsToSave(self->position);
  return(1);
};




unsigned int checkThisSerial(unsigned long *snAdress){
    unsigned int indexSerial;
    unsigned int bandeja = 0;


    for (indexSerial = 0; indexSerial < MAXIMUM_NUMBER_OF_CONVERTERS; indexSerial++)
    {
        if (snAdress[0] == serialNodeRegister[indexSerial].serialNumberPart[0] &&\
                snAdress[1] == serialNodeRegister[indexSerial].serialNumberPart[1] &&\
                snAdress[2] == serialNodeRegister[indexSerial].serialNumberPart[2] &&\
                snAdress[3] == serialNodeRegister[indexSerial].serialNumberPart[3] &&\
                snAdress[4] == serialNodeRegister[indexSerial].serialNumberPart[4] &&\
                snAdress[5] == serialNodeRegister[indexSerial].serialNumberPart[5] &&\
                snAdress[6] == serialNodeRegister[indexSerial].serialNumberPart[6] &&\
                snAdress[7] == serialNodeRegister[indexSerial].serialNumberPart[7] &&\
                snAdress[8] == serialNodeRegister[indexSerial].serialNumberPart[8] &&\
                snAdress[9] == serialNodeRegister[indexSerial].serialNumberPart[9] &&\
                snAdress[10] == serialNodeRegister[indexSerial].serialNumberPart[10] &&\
				snAdress[11] == serialNodeRegister[indexSerial].serialNumberPart[11] &&\
				snAdress[12] == serialNodeRegister[indexSerial].serialNumberPart[12] &&\
				snAdress[13] == serialNodeRegister[indexSerial].serialNumberPart[13] &&\
				snAdress[14] == serialNodeRegister[indexSerial].serialNumberPart[14] &&\
				snAdress[15] == serialNodeRegister[indexSerial].serialNumberPart[15])

        {

           bandeja = serialNodeRegister[indexSerial].tray;
#if DEBUG
    		env_debug("Check serial bandeja:");

    		env_debug_int((int)bandeja);
    		env_debug("\n");
#endif
           return (bandeja);
        }
    };
    return (bandeja);  //  Return tray
};

unsigned int pushThisSerial(unsigned long *snAdress, unsigned char thisTray){
    unsigned int indexSerial;
#if DEBUG
    		env_debug("Voy a Push serial:");

    		env_debug_int((int)thisTray);
    		env_debug("\n");
#endif
    for (indexSerial = 0; indexSerial < MAXIMUM_NUMBER_OF_CONVERTERS; indexSerial++)
    {
		if (serialNodeRegister[indexSerial].tray == 0)
        {

#if DEBUG
    		env_debug("Push serial module:");

    		env_debug_int((int)thisTray);
    		env_debug("\n");
#endif
            serialNodeRegister[indexSerial].serialNumberPart[0] = snAdress[0] ;
            serialNodeRegister[indexSerial].serialNumberPart[1] = snAdress[1] ;
            serialNodeRegister[indexSerial].serialNumberPart[2] = snAdress[2] ;
            serialNodeRegister[indexSerial].serialNumberPart[3] = snAdress[3] ;
            serialNodeRegister[indexSerial].serialNumberPart[4] = snAdress[4] ;
            serialNodeRegister[indexSerial].serialNumberPart[5] = snAdress[5] ;
            serialNodeRegister[indexSerial].serialNumberPart[6] = snAdress[6] ;
            serialNodeRegister[indexSerial].serialNumberPart[7] = snAdress[7] ;
            serialNodeRegister[indexSerial].serialNumberPart[8] = snAdress[8] ;
            serialNodeRegister[indexSerial].serialNumberPart[9] = snAdress[9] ;
            serialNodeRegister[indexSerial].serialNumberPart[10] = snAdress[10] ;
            serialNodeRegister[indexSerial].serialNumberPart[11] = snAdress[11] ;
            serialNodeRegister[indexSerial].serialNumberPart[12] = snAdress[12] ;
            serialNodeRegister[indexSerial].serialNumberPart[13] = snAdress[13] ;
            serialNodeRegister[indexSerial].serialNumberPart[14] = snAdress[14] ;
            serialNodeRegister[indexSerial].serialNumberPart[15] = snAdress[15] ;

            serialNodeRegister[indexSerial].tray = thisTray;
            discoveredModules++;
            return (1);
        }


    }
    return(0);
};

void    destroyThisSerial(unsigned int clearPosition){
     unsigned int indexSerial;

    for (indexSerial = 0; indexSerial < MAXIMUM_NUMBER_OF_CONVERTERS; indexSerial++)
    {
        if (serialNodeRegister[indexSerial].tray == clearPosition)
        {
            serialNodeRegister[indexSerial].serialNumberPart[0] = 0 ;
            serialNodeRegister[indexSerial].serialNumberPart[1] = 0 ;
            serialNodeRegister[indexSerial].serialNumberPart[2] = 0 ;
            serialNodeRegister[indexSerial].serialNumberPart[3] = 0 ;
            serialNodeRegister[indexSerial].serialNumberPart[4] = 0 ;
            serialNodeRegister[indexSerial].serialNumberPart[5] = 0 ;
            serialNodeRegister[indexSerial].serialNumberPart[6] = 0 ;
            serialNodeRegister[indexSerial].serialNumberPart[7] = 0 ;
            serialNodeRegister[indexSerial].serialNumberPart[8] = 0 ;
            serialNodeRegister[indexSerial].serialNumberPart[9] = 0 ;
            serialNodeRegister[indexSerial].serialNumberPart[10] = 0 ;
            serialNodeRegister[indexSerial].serialNumberPart[11] = 0 ;
            serialNodeRegister[indexSerial].serialNumberPart[12] = 0 ;
            serialNodeRegister[indexSerial].serialNumberPart[13] = 0;
            serialNodeRegister[indexSerial].serialNumberPart[14] = 0 ;
            serialNodeRegister[indexSerial].serialNumberPart[15] = 0 ;

            serialNodeRegister[indexSerial].tray = 0;
            if (discoveredModules) discoveredModules--;
        }
    }
}

//--- INTEGRATION INSTRUCTIONS
//--- (Interruption timer every 1ms)
void clockTickAllSM (void)
{
    unsigned int nodeIndex;
    for (nodeIndex = 0; nodeIndex < maxNumberOfConvertersWorking; nodeIndex++)
    {
        deltaConverter[nodeIndex].machine.timeRunning++;
        deltaConverter[nodeIndex].machine.timerPower++;
        deltaConverter[nodeIndex].machine.timerOut2++;
        deltaConverter[nodeIndex].machine.timerPing++;
        deltaConverter[nodeIndex].machine.currentRequestTime++;
        if ( (deltaConverter[nodeIndex].commandWord & SWITCH_ON_COMMAND) == SWITCH_ON_COMMAND) deltaConverter[nodeIndex].machine.sessionTime++;
    }
};



void deltaErrors_getErrorsLastSessions  (DeltaModule *self){
/*  switch (self->position)
    {
       case 1:
            self->errorCounter.absolutErrorCounter = MODBUS_POWER_MODULE_1_ERROR_COUNT;
            break;
        case 2:
            self->errorCounter.absolutErrorCounter = MODBUS_POWER_MODULE_2_ERROR_COUNT;
            break;
        case 3:
            self->errorCounter.absolutErrorCounter = MODBUS_POWER_MODULE_3_ERROR_COUNT;
            break;
        case 4:
            self->errorCounter.absolutErrorCounter = MODBUS_POWER_MODULE_4_ERROR_COUNT;
            break;
        default:
            break;
    }

*/
};

void deltaErrors_pushErrorsThisSession  (DeltaModule *self){
/*
    int     errorsIn30s = 0, errorsIn1m = 0, errorsIn10m = 0, errorsIn15m = 0;
    int     i, j;
    long    converterWasActivated = 0;


    for (i = 0; i < MAXIMUM_NUMBER_OF_CONVERTERS; i++)
    {
        if (errorTimeRegister[i].tray == 0 || errorTimeRegister[i].tray == self->position)
        {
            errorTimeRegister[i].absolutSystemTimeRec[9] = errorTimeRegister[i].absolutSystemTimeRec[8];
            errorTimeRegister[i].absolutSystemTimeRec[8] = errorTimeRegister[i].absolutSystemTimeRec[7];
            errorTimeRegister[i].absolutSystemTimeRec[7] = errorTimeRegister[i].absolutSystemTimeRec[6];
            errorTimeRegister[i].absolutSystemTimeRec[6] = errorTimeRegister[i].absolutSystemTimeRec[5];
            errorTimeRegister[i].absolutSystemTimeRec[5] = errorTimeRegister[i].absolutSystemTimeRec[4];
            errorTimeRegister[i].absolutSystemTimeRec[4] = errorTimeRegister[i].absolutSystemTimeRec[3];
            errorTimeRegister[i].absolutSystemTimeRec[3] = errorTimeRegister[i].absolutSystemTimeRec[2];
            errorTimeRegister[i].absolutSystemTimeRec[2] = errorTimeRegister[i].absolutSystemTimeRec[1];
            errorTimeRegister[i].absolutSystemTimeRec[1] = errorTimeRegister[i].absolutSystemTimeRec[0];
            errorTimeRegister[i].absolutSystemTimeRec[0] = self->machine.absolutSystemTime;

            errorTimeRegister[i].tray = self->position;

            // Verifica errores por tiempo
            for (j = 0; j < 9; j++)
            {
                if (errorTimeRegister[i].absolutSystemTimeRec[j+1] == 0) break;
                if ((errorTimeRegister[i].absolutSystemTimeRec[0] - errorTimeRegister[i].absolutSystemTimeRec[j+1]) < 30000) errorsIn30s++;
                if ((errorTimeRegister[i].absolutSystemTimeRec[0] - errorTimeRegister[i].absolutSystemTimeRec[j+1]) < 60000) errorsIn1m++;
                if ((errorTimeRegister[i].absolutSystemTimeRec[0] - errorTimeRegister[i].absolutSystemTimeRec[j+1]) < 600000) errorsIn10m++;
                if ((errorTimeRegister[i].absolutSystemTimeRec[0] - errorTimeRegister[i].absolutSystemTimeRec[j+1]) < 900000) errorsIn15m++;
            }
            break;
        }
    }

    // Reglas de espera:
    if (errorsIn30s     >= 2) self->errorCounter.disableTime = TIMER_10S;
    if (errorsIn1m      >= 3) self->errorCounter.disableTime = TIMER_60S;
    if (errorsIn10m     >= 5) self->errorCounter.disableTime = 5*TIMER_60S;
    if (errorsIn15m     >= 7) self->errorCounter.disableTime = 30*TIMER_60S;


    // Actualizo las variables pr�pias del nodo que acabar�n en la fram:
    self->errorCounter.absolutErrorCounter++;
    switch (self->position)
    {
        case 1:     MODBUS_POWER_MODULE_1_ERROR_COUNT++;    break;
        case 2:     MODBUS_POWER_MODULE_2_ERROR_COUNT++;    break;
        case 3:     MODBUS_POWER_MODULE_3_ERROR_COUNT++;    break;
        case 4:     MODBUS_POWER_MODULE_4_ERROR_COUNT++;    break;
        default:                                            break;
    }

    Guardar_Parametros_FRAM_DELTA(self);*/
}

DeltaErrorCounter construct_DeltaErrorCounter(){
    DeltaErrorCounter deltaErrorCounter;

    deltaErrorCounter.absolutErrorCounter   = 0;
    deltaErrorCounter.disableTime           = 0;
    deltaErrorCounter.getErrors             = deltaErrors_getErrorsLastSessions;
    deltaErrorCounter.pushErrors            = deltaErrors_pushErrorsThisSession;

 /*   int i, j;

    for(i = 0; i < MAXIMUM_NUMBER_OF_CONVERTERS; i++)
    {
        for (j = 0; j < 10; j++)
        {
            errorTimeRegister[i].absolutSystemTimeRec[j] = 0;
        }
        errorTimeRegister[i].tray = 0;
    }*/

    return (deltaErrorCounter);
};


// Other useful tools
DeltaModule *getModuleFromCanMsg(unsigned long identifier){
    int i;
    unsigned long id = identifier & 0x000000FF;

    DeltaModule *adressToReturn;

   // At the beginning the address to return corresponds to a non-existing module
    adressToReturn = &nullConverter;
    if (id == 0) return(adressToReturn);    // The Id cannot be 0!
    for (i = 0; i < maxNumberOfConvertersWorking; i++)
    {
        if (deltaConverter[i].identifier == id) { adressToReturn = &deltaConverter[i]; break;}
    }

    // Return the address found
    return(adressToReturn);
}

DeltaModule *getModuleFromId(unsigned int id){
    int i;
    DeltaModule *adressToReturn;

    adressToReturn = &nullConverter;
    if (id == 0) return(adressToReturn);    // Identifier zero means nullConverter
    for (i = 0; i < maxNumberOfConvertersWorking; i++)
    {
        if (deltaConverter[i].identifier == id) { adressToReturn = &deltaConverter[i]; break;}
    }
    return(adressToReturn);
}


DeltaModule *getModuleFromPosition(unsigned int pos){
    int i;
    DeltaModule *adressToReturn;

    adressToReturn = &nullConverter;

    if (pos == 0) return(adressToReturn);
    for (i = 0; i < maxNumberOfConvertersWorking; i++)
    {
        if (deltaConverter[i].position == pos) { adressToReturn = &deltaConverter[i]; break;}
    }

    return(adressToReturn);
}



EventLocation createEvent(const unsigned char system, const unsigned char subSystem, const unsigned char subsubSystem, \
                          const SeverityEnum severity, const unsigned char errorCode){

	EventLocation event;
    event.hardware = system;
    event.subsystem = subSystem;
    event.subsubsystem = subsubSystem;
    event.severity = severity;
    event.eventCode = errorCode;

    if (severity > SEVERITY_WARNING){


#if DEBUG_CREATE_EVENT



    	if (severity > SEVERITY_WARNING){
    		env_debug("Evento creado:");
    		env_debug("     Sist:");

    		env_debug_int((int)system);
    		env_debug("     SubSist:");

    		env_debug_int((int)subSystem);
    		env_debug("     SubsubSist:");

    		env_debug_int((int)subsubSystem);
    		env_debug("     Sev:");

    		env_debug_int(severity);
    		env_debug("     Error:");

    		env_debug_int(errorCode);
    		env_debug("\n");
    	}


#endif


    //Points to global function
	if (globalState == STANDBY_A2 && subSystem == 0){
       /* void */
	}
	else{
	     if (own_node == supervisor_node)
	         registerEventLocation(&event, eventBufferM);
      	 else
	         registerEventLocation(&event, eventBuffer);
			 
	}

    }
    return event;
}

unsigned char errorCodeDelta2Circontrol(unsigned int deltaCode){
    switch(deltaCode){
        case ON_VOLTAGE_MODE_ERROR_WORD:            return(ON_VOLTAGE_MODE_ERROR_WORD_STD);
        case ON_LIGHT_LOAD_ERROR_WORD:              return(ON_LIGHT_LOAD_ERROR_WORD_STD);
        case ON_POWER_LIMIT_ERROR_WORD:             return(ON_POWER_LIMIT_ERROR_WORD_STD);
        case ON_CURRENT_LIMIT_ERROR_WORD:           return(ON_CURRENT_LIMIT_ERROR_WORD_STD);
        case ON_SEPARATE_CHARGE_ERROR_WORD:         return(ON_SEPARATE_CHARGE_ERROR_WORD_STD);
        case ON_LOAD_SHARE_PASSIVE_ERROR_WORD:      return(ON_LOAD_SHARE_PASSIVE_ERROR_WORD_STD);
        case ON_LOAD_SHARE_ERROR_WORD:              return(ON_LOAD_SHARE_ERROR_WORD_STD);
        case ON_HIGH_TEMPERATURE_ERROR_WORD:        return(ON_HIGH_TEMPERATURE_ERROR_WORD_STD);
        case ON_LOW_INPUT_VOLTAGE_ERROR_WORD:       return(ON_LOW_INPUT_VOLTAGE_ERROR_WORD_STD);
        case ON_ONE_LINE_OUTSIDE_RANGE_ERROR_WORD:  return(ON_ONE_LINE_OUTSIDE_RANGE_ERROR_WORD_STD);
        case ON_INTERNAL_FAILURE_ERROR_WORD:        return(ON_INTERNAL_FAILURE_ERROR_WORD_STD);
        case ON_FAN_FAILURE_ERROR_WORD:             return(ON_FAN_FAILURE_ERROR_WORD_STD);

        case REMOTE_OFF_ERROR_WORD:                 return(REMOTE_OFF_ERROR_WORD_STD);

        case OFF_INPUT_VOLTAGE_LOW_ERROR_WORD:      return(OFF_INPUT_VOLTAGE_LOW_ERROR_WORD_STD);
        case OFF_INPUT_VOLTAGE_HIGH_ERROR_WORD:     return(OFF_INPUT_VOLTAGE_HIGH_ERROR_WORD_STD);
        case OFF_ONE_LINE_OUTSIDE_RANGE_ERROR_WORD: return(OFF_ONE_LINE_OUTSIDE_RANGE_ERROR_WORD_STD);
        case OFF_TWO_LINE_OUTSIDE_RANGE_ERROR_WORD: return(OFF_TWO_LINE_OUTSIDE_RANGE_ERROR_WORD_STD);
        case OFF_STARTUP_DELAY_ERROR_WORD:          return(OFF_STARTUP_DELAY_ERROR_WORD_STD);
        case OFF_LOCAL_OFF_ERROR_WORD:              return(OFF_LOCAL_OFF_ERROR_WORD_STD);

        case TEMPORARY_OFF_OVERTEMPERATURE_ERROR_WORD:  return(TEMPORARY_OFF_OVERTEMPERATURE_ERROR_WORD_STD);
        case TEMPORARY_OFF_OVERVOLTAGE_ERROR_WORD:      return(TEMPORARY_OFF_OVERVOLTAGE_ERROR_WORD_STD);
        case TEMPORARY_OFF_FAN_FAILURE_ERROR_WORD:      return(TEMPORARY_OFF_FAN_FAILURE_ERROR_WORD_STD);
        case LATCHED_OFF_PRESENT_OVERTEMPERATURE_ERROR_WORD:  return(LATCHED_OFF_PRESENT_OVERTEMPERATURE_ERROR_WORD_STD);
        case LATCHED_OFF_PRESENT_OVERVOLTAGE_ERROR_WORD:      return(LATCHED_OFF_PRESENT_OVERVOLTAGE_ERROR_WORD_STD);
        case LATCHED_OFF_PRESENT_FAN_FAILURE_ERROR_WORD:      return(LATCHED_OFF_PRESENT_FAN_FAILURE_ERROR_WORD_STD);
        case LATCHED_OFF_PRESENT_INTERNAL_FAILURE_ERROR_WORD: return(LATCHED_OFF_PRESENT_INTERNAL_FAILURE_ERROR_WORD_STD);

        case LATCHED_OFF_NOT_PRESENT_OVERTEMPERATURE_ERROR_WORD:    return(LATCHED_OFF_NOT_PRESENT_OVERTEMPERATURE_ERROR_WORD_STD);
        case LATCHED_OFF_NOT_PRESENT_OVERVOLTAGE_ERROR_WORD:        return(LATCHED_OFF_NOT_PRESENT_OVERVOLTAGE_ERROR_WORD_STD);
        case LATCHED_OFF_NOT_PRESENT_FAN_FAILURE_ERROR_WORD:        return(LATCHED_OFF_NOT_PRESENT_FAN_FAILURE_ERROR_WORD_STD);
        case LATCHED_OFF_NOT_PRESENT_INTERNAL_FAILURE_ERROR_WORD:   return(LATCHED_OFF_NOT_PRESENT_INTERNAL_FAILURE_ERROR_WORD_STD);
        case LATCHED_OFF_NOT_PRESENT_FPC_FAILURE_ERROR_WORD:    return(LATCHED_OFF_NOT_PRESENT_FPC_FAILURE_ERROR_WORD_STD);
        case LATCHED_OFF_NOT_PRESENT_DCDC_FAILURE_ERROR_WORD:   return(LATCHED_OFF_NOT_PRESENT_DCDC_FAILURE_ERROR_WORD_STD);
        case LATCHED_OFF_NOT_PRESENT_UNKNOWN_ERROR_WORD:        return(LATCHED_OFF_NOT_PRESENT_UNKNOWN_ERROR_WORD_STD);

        case ERROR_INTERNAL_FAILURE_ERROR_WORD:                 return(ERROR_INTERNAL_FAILURE_ERROR_WORD_STD);
        case ERROR_PFC_FAILURE_ERROR_WORD:                      return(ERROR_PFC_FAILURE_ERROR_WORD_STD);
        case ERROR_DCDC_FAILURE_ERROR_WORD:                     return(ERROR_DCDC_FAILURE_ERROR_WORD_STD);
        case ERROR_OUTPUT_FUSE_ERROR_WORD:                      return(ERROR_OUTPUT_FUSE_ERROR_WORD_STD);
        case ERROR_INPUT_FUSE_ERROR_WORD:                       return(ERROR_INPUT_FUSE_ERROR_WORD_STD);
        case ERROR_OUTPUT_POWER_ERROR_WORD:                     return(ERROR_OUTPUT_POWER_ERROR_WORD_STD);
        case ERROR_CABINET_OUTPUT_DISCONNECTED:                 return(ERROR_CABINET_OUTPUT_DISCONNECTED_STD);
        default: return(0xFF);
    };
}


unsigned char errorCodeDelta22Circontrol(unsigned int deltaCode){
    switch(deltaCode){
case DELTA_STATUS_SERIES_MODE:                     return(DELTA_STATUS_SERIES_MODE_STD);
case DELTA_STATUS_NORMAL_WORKING:                  return(DELTA_STATUS_NORMAL_WORKING_STD);
case DELTA_STATUS_OUTPUT_DERATED:                  return(DELTA_STATUS_OUTPUT_DERATED_STD);
case DELTA_STATUS_LOAD_SHORT_LATCH:                return(DELTA_STATUS_LOAD_SHORT_LATCH_STD);
case DELTA_STATUS_STAND_BY_MODE:                   return(DELTA_STATUS_STAND_BY_MODE_STD);
case DELTA_STATUS_SLEEP_MODE:                      return(DELTA_STATUS_SLEEP_MODE_STD);
case DELTA_STATUS_PS_KILLED:                       return(DELTA_STATUS_PS_KILLED_STD);
case DELTA_STATUS_CAN_TIMEOUT:                     return(DELTA_STATUS_CAN_TIMEOUT_STD);
case DELTA_STATUS_VAC_EXCEPTION:                   return(DELTA_STATUS_VAC_EXCEPTION_STD);
case DELTA_STATUS_TEMPERATURE_EXCEPTION:           return(DELTA_STATUS_TEMPERATURE_EXCEPTION_STD);
case DELTA_STATUS_OTHER_EXCEPTION:                 return(DELTA_STATUS_OTHER_EXCEPTION_STD);
case DELTA_STATUS_SCI_FAIL:                        return(DELTA_STATUS_SCI_FAIL_STD);
case DELTA_STATUS_FAN_FAIL:                        return(DELTA_STATUS_FAN_FAIL_STD);
case DELTA_STATUS_DISCHARGE_FAIL:                  return(DELTA_STATUS_DISCHARGE_FAIL_STD);
case DELTA_STATUS_OTHER_FAILURE:                   return(DELTA_STATUS_OTHER_FAILURE_STD);
case DELTA_STATUS_UPDATING_FIRMWARE:               return(DELTA_STATUS_UPDATING_FIRMWARE_STD);
        default: return(0xFF);
    };
}
