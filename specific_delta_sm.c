/*
 * delta_specific.c
 *
 *  Created on: 13 ago. 2018
 *      Author: josepmaria.fernandez
 */


#include "specific_delta.h"     // Header file Include File
#include "main.h" // Only for tiny printf
#include "EventList.h" //Includes the error lists




unsigned int globalS = 0;
unsigned int globalS_ant = 0;

//inline functions
inline void deltaSM_errorCpy(EventWord *errorOrigin, EventWord *errorDestination);

DeltaStateMachine construct_DeltaStateMachine(){
    DeltaStateMachine deltaStateMachine;

    //--- Init timers
    deltaStateMachine.timeRunning                       = 0;
    deltaStateMachine.timerPower                        = 0;
    deltaStateMachine.timerOut2                         = 0;
    deltaStateMachine.timerPing                         = 0;
    deltaStateMachine.sessionTime                       = 0;
    deltaStateMachine.num_serie		                    = 0;
    deltaStateMachine.num_data		                    = 0;

    //--- Init estados
    deltaStateMachine.estadoActualConvertidor           = 0;
    //int indexEstado = 0;

    //--- Init flags
    deltaStateMachine.structureSeekingConverter     = 0;
    deltaStateMachine.converterFound                = 0;
    deltaStateMachine.serialNumberReceivedPartFlag      = 0;

    //--- Init punteros a funciones
    deltaStateMachine.stateMachine                      = deltaSM_stateMachine;
    deltaStateMachine.state000                          = deltaSM_state000;
    deltaStateMachine.state001                          = deltaSM_state001;
    deltaStateMachine.state002                          = deltaSM_state002;
    deltaStateMachine.state003                          = deltaSM_state003;
    deltaStateMachine.state004                          = deltaSM_state004;
    deltaStateMachine.state005                          = deltaSM_state005;
    deltaStateMachine.state006                          = deltaSM_state006;
    deltaStateMachine.state007                          = deltaSM_state007;
    deltaStateMachine.state008                          = deltaSM_state008;
    deltaStateMachine.state009                          = deltaSM_state009;
    deltaStateMachine.state010                          = deltaSM_state010;
    deltaStateMachine.state011                          = deltaSM_state011;
    deltaStateMachine.state012                          = deltaSM_state012;
    deltaStateMachine.state013                          = deltaSM_state013;
    deltaStateMachine.state014                          = deltaSM_state014;
    deltaStateMachine.state015                          = deltaSM_state015;
    deltaStateMachine.state016                          = deltaSM_state016;
    deltaStateMachine.state017                          = deltaSM_state017;
    deltaStateMachine.state018                          = deltaSM_state018;
    deltaStateMachine.state019                          = deltaSM_state019;
    deltaStateMachine.state020                          = deltaSM_state020;
    deltaStateMachine.state021                          = deltaSM_state021;
    deltaStateMachine.state022                          = deltaSM_state022;
    deltaStateMachine.state023                          = deltaSM_state023;
    deltaStateMachine.state024                          = deltaSM_state024;

    deltaStateMachine.state027                          = deltaSM_state027;
    deltaStateMachine.state028                          = deltaSM_state028;
    deltaStateMachine.state029                          = deltaSM_state029;
    deltaStateMachine.state030                          = deltaSM_state030;
    deltaStateMachine.state031                          = deltaSM_state031;
    deltaStateMachine.state032                          = deltaSM_state032;
    deltaStateMachine.state033                          = deltaSM_state033;
    deltaStateMachine.state034                          = deltaSM_state034;
    deltaStateMachine.state035                          = deltaSM_state035;

    deltaStateMachine.state100                          = deltaSM_state100;
    deltaStateMachine.state101                          = deltaSM_state101;
    deltaStateMachine.state102                          = deltaSM_state102;
    deltaStateMachine.state103                          = deltaSM_state103;
    deltaStateMachine.state104                          = deltaSM_state104;
    deltaStateMachine.state105                          = deltaSM_state105;
    deltaStateMachine.state106                          = deltaSM_state106;
    deltaStateMachine.state107                          = deltaSM_state107;
    deltaStateMachine.state108                          = deltaSM_state108;

    deltaStateMachine.state120                          = deltaSM_state120;
    deltaStateMachine.state121                          = deltaSM_state121;


    deltaStateMachine.state150                          = deltaSM_state150;
    deltaStateMachine.state151                          = deltaSM_state151;

    deltaStateMachine.state200                          = deltaSM_state200;
    deltaStateMachine.state201                          = deltaSM_state201;
    deltaStateMachine.state202                          = deltaSM_state202;
    deltaStateMachine.state203                          = deltaSM_state203;
    deltaStateMachine.state204                          = deltaSM_state204;
    deltaStateMachine.state205                          = deltaSM_state205;
    deltaStateMachine.state206                          = deltaSM_state206;
    deltaStateMachine.state207                          = deltaSM_state207;
    deltaStateMachine.state208                          = deltaSM_state208;
    deltaStateMachine.state220                          = deltaSM_state220;
    deltaStateMachine.state221                          = deltaSM_state221;



    deltaStateMachine.state250                          = deltaSM_state250;
    deltaStateMachine.state251                          = deltaSM_state251;
    deltaStateMachine.state252                          = deltaSM_state252;

    deltaStateMachine.state700                          = deltaSM_state700;
    deltaStateMachine.state701                          = deltaSM_state701;
    deltaStateMachine.state702                          = deltaSM_state702;

    deltaStateMachine.state800                          = deltaSM_state800;
    deltaStateMachine.state801                          = deltaSM_state801;
    deltaStateMachine.state802                          = deltaSM_state802;
    deltaStateMachine.state850                          = deltaSM_state850;
    deltaStateMachine.state851                          = deltaSM_state851;
    deltaStateMachine.state852                          = deltaSM_state852;
    deltaStateMachine.state853                          = deltaSM_state853;
    deltaStateMachine.state854                          = deltaSM_state854;
    deltaStateMachine.state855                          = deltaSM_state855;
    deltaStateMachine.state856                          = deltaSM_state856;
    deltaStateMachine.state857                          = deltaSM_state857;





    deltaStateMachine.state870                          = deltaSM_state870;
    deltaStateMachine.state871                          = deltaSM_state871;

    // Auxiliary functions initialization
    deltaStateMachine.initMachine                       = deltaSM_initStateMachine;
    deltaStateMachine.pushErrorWord                     = deltaSM_pushErrorWord;
    deltaStateMachine.checkIfNodeExists                 = deltaSM_checkIfNodeExists;
    deltaStateMachine.getNonExistingNode                = deltaSM_getNonExistingNode;
    deltaStateMachine.pushNewNode                       = deltaSM_pushNewNode;
    deltaStateMachine.destroyExistingNode               = deltaSM_destroyExistingNode;

    return (deltaStateMachine);
};

void deltaSM_initStateMachine(DeltaModule *self){
    self->identifier                                = 0;
    self->position                                  = 0;
    self->phyAddr                                   = 0;
    self->currentSetPoint                           = 0;
    self->voltageSetPoint                           = 0;
    self->powerSetPoint                             = 0;
    self->statusWord                                = 0;
    self->commandWord                               = 0;
    self->operationTime                             = 0;
    self->powerOnCycles                             = 0;
    self->errorDataWord                             = 0;
    self->errorDataWordStd                          = 0;
    self->chargedEnergy                             = 0;
    self->sessionEnergy                             = 0;
    self->availableCurrent                          = 0;
    self->protocoloAsignado                         = 0;
    self->machine.converterFound                    = 0;
    self->machine.structureSeekingConverter         = 0;
    self->machine.serialNumberReceivedPartFlag      = 0;

    int indexSN;
    for (indexSN = 0; indexSN < 16; indexSN++) {
        self->serialNumber[indexSN] = 0;
    }

    self->limits.tempHigh           = MAXIMUM_DELTA_TEMP;
    self->limits.tempLow            = MINIMUM_DELTA_TEMP;
    self->limits.vMax               = MAXIMUM_DELTA_VOLTAGE;
    self->limits.iMax               = MAXIMUM_DELTA_CURRENT;
    self->limits.pMax               = MAXIMUM_DELTA_POWER;

    self->errorCounter.disableTime  = TIMER_1S;
    self->errorCounter.lastDiscrepancyDetectedTime = 0;
} ;


/* Esta es la m�quina de estados propiamente dicha, est� dentro de la estructura
 * de DeltaConverter -> Machine */
void deltaSM_stateMachine(DeltaModule *self){
    //switch-case enorme
#if	DEBUG_ESTADOS_DELTAS
	if (self->yi == 0)
	{

		globalS = self->machine.estadoActualConvertidor;

		if (globalS != globalS_ant){
		  globalS_ant = globalS;

		  if (globalS >= 100){
			sprintf(bufferstr, "......Delta state: %i \n", (int) globalS);
			      env_debug(bufferstr);
		  }
		}
	 }
#endif
    switch (self->machine.estadoActualConvertidor)
    {
        case (0): self->machine.state000(self); break;
        case (1): self->machine.state001(self); break;
        case (2): self->machine.state002(self); break;
        case (3): self->machine.state003(self); break;
        case (4): self->machine.state004(self); break;
        case (5): self->machine.state005(self); break;
        case (6): self->machine.state006(self); break;
        case (7): self->machine.state007(self); break;
        case (8): self->machine.state008(self); break;
        case (9): self->machine.state009(self); break;
        case (10): self->machine.state010(self); break;
        case (11): self->machine.state011(self); break;
        case (12): self->machine.state012(self); break;
        case (13): self->machine.state013(self); break;
        case (14): self->machine.state014(self); break;
        case (15): self->machine.state015(self); break;
        case (16): self->machine.state016(self); break;
        case (17): self->machine.state017(self); break;
        case (18): self->machine.state018(self); break;

        case (19): self->machine.state019(self); break;
        case (20): self->machine.state020(self); break;
        case (21): self->machine.state021(self); break;
        case (22): self->machine.state022(self); break;
        case (23): self->machine.state023(self); break;
        case (24): self->machine.state024(self); break;


        case (27): self->machine.state027(self); break;
        case (28): self->machine.state028(self); break;
        case (29): self->machine.state029(self); break;
        case (30): self->machine.state030(self); break;
        case (31): self->machine.state031(self); break;
        case (32): self->machine.state032(self); break;
        case (33): self->machine.state033(self); break;
        case (34): self->machine.state034(self); break;
        case (35): self->machine.state035(self); break;
        case (100): self->machine.state100(self); break;
        case (101): self->machine.state101(self); break;
        case (102): self->machine.state102(self); break;
        case (103): self->machine.state103(self); break;
        case (104): self->machine.state104(self); break;
        case (105): self->machine.state105(self); break;
        case (106): self->machine.state106(self); break;
        case (107): self->machine.state107(self); break;
        case (108): self->machine.state108(self); break;

        case (120): self->machine.state120(self); break;
        case (121): self->machine.state121(self); break;


        case (150): self->machine.state150(self); break;
        case (151): self->machine.state151(self); break;
        case (200): self->machine.state200(self); break;
        case (201): self->machine.state201(self); break;
        case (202): self->machine.state202(self); break;
        case (203): self->machine.state203(self); break;
        case (204): self->machine.state204(self); break;
        case (205): self->machine.state205(self); break;
        case (206): self->machine.state206(self); break;
        case (207): self->machine.state207(self); break;
        case (208): self->machine.state208(self); break;
        case (220): self->machine.state220(self); break;
        case (221): self->machine.state221(self); break;

        case (250): self->machine.state250(self); break;
        case (251): self->machine.state251(self); break;
        case (252): self->machine.state252(self); break;

        case (700): self->machine.state700(self); break;
        case (701): self->machine.state701(self); break;
        case (702): self->machine.state702(self); break;
        case (800): self->machine.state800(self); break;
        case (801): self->machine.state801(self); break;
        case (802): self->machine.state802(self); break;

        case (850): self->machine.state850(self); break;
        case (851): self->machine.state851(self); break;
        case (852): self->machine.state852(self); break;
        case (853): self->machine.state853(self); break;
        case (854): self->machine.state854(self); break;
        case (855): self->machine.state855(self); break;
        case (856): self->machine.state856(self); break;
        case (857): self->machine.state857(self); break;

        case (870): self->machine.state870(self); break;
        case (871): self->machine.state871(self); break;

        default:  /*self->machine.estadoActualConvertidor = 800;*/ break;
    }
};

/**
       * A function.
       * Initializations of variables. Evolution without restrictions.
       * @param self The own structure itself.
       */
void deltaSM_state000(DeltaModule *self){
    self->machine.timeRunning               = 0;
    self->machine.timerPower                = 0;
    self->machine.timerOut2                 = 0;
    self->machine.timerPing                 = 0;
    self->machine.currentRequestTime        = 0;
    self->machine.initMachine(self);
    self->machine.estadoActualConvertidor++;
};


/**
       * A function.
       * Waits a predefined time until all peripherals are initialized. After that,
       * this function blocks until a converter is available.
       * @param self The own structure itself.
       */
void deltaSM_state001(DeltaModule *self){
    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;

    if ((self->machine.timeRunning > TIMER_10S)) \
        self->machine.structureSeekingConverter = 1;
    else self->machine.structureSeekingConverter = 0;


    if  ((self->machine.timeRunning > TIMER_10S) && \
            self->machine.converterFound)
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
};


/**
       * Starts the initialization process. In the first step, the "Snap 1" word
       * is required by the Delta converter. If it cannot be sent in 20ms, there is
       * an error in the CAN bus access
       * @param self The own structure itself.
       */
void deltaSM_state002(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                        = 0;
    self->machine.timerOut2                         = 0;
    self->machine.timerPing                         = 0;
    self->machine.structureSeekingConverter         = 1;
    self->machine.currentRequestTime                = 0;
    self->pings                                     = 0;

    if  (self->can.sendSnapStep1(self))
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_20MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_SNAP_STEP);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error */
    }
};

/**
       * In this function the state machine waits for at least 20ms (as said in
       * Delta specifications). If the waiting time is greater than 80ms there
       * is something wrong with the code.
       * @param self The own structure itself.
       */
void deltaSM_state003(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;

    if  (self->machine.timeRunning > TIMER_20MS && self->machine.timeRunning <= TIMER_80MS)
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_80MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_SNAP_STEP);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error */
    }
};

/**
       * Initialization process. The "Snap 2" word
       * is required by the Delta converter. If it cannot be sent in 20ms, there is
       * an error in the CAN bus access
       * @param self The own structure itself.
       */
void deltaSM_state004(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;

    if  (self->can.sendSnapStep2(self))
   {
       self->machine.timeRunning = 0;
       self->machine.estadoActualConvertidor++;
   }
   else if (self->machine.timeRunning > TIMER_20MS)
   {
       event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_SNAP_STEP);
       self->machine.pushErrorWord(self, &event);
       self->machine.timeRunning = 0;
       self->machine.estadoActualConvertidor  = 700;  /** < Machine to error */
   }
};

/**
       * In this function the state machine waits for at least 20ms (as said in
       * Delta specifications). If the waiting time is greater than 80ms there
       * is something wrong with the code.
       * @param self The own structure itself.
       */
void deltaSM_state005(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;

    if  (self->machine.timeRunning > TIMER_20MS && self->machine.timeRunning <= TIMER_80MS)
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_80MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_SNAP_STEP);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error */
    }
};

/**
       * Initialization process. The "Snap 3" word is required by the Delta converter. If it
       * cannot be sent in 20ms, there is an error in the CAN bus access.
       * @param self The own structure itself.
       */
void deltaSM_state006(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;

    if  (self->can.sendSnapStep3(self))
   {
       self->machine.timeRunning = 0;
       self->machine.estadoActualConvertidor++;
   }
   else if (self->machine.timeRunning > TIMER_20MS)
   {
       event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_SNAP_STEP);
       self->machine.pushErrorWord(self, &event);
       self->machine.timeRunning = 0;
       self->machine.estadoActualConvertidor  = 700;  /** < Machine to error */
   }
};

/**
       * In this function the state machine waits for at least 20ms (as said in
       * Delta specifications). If the waiting time is greater than 80ms there
       * is something wrong with the code.
       * @param self The own structure itself.
       */
void deltaSM_state007(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;

    if  (self->machine.timeRunning > TIMER_20MS && self->machine.timeRunning <= TIMER_80MS)
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_80MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_SNAP_STEP);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error */
    }
};

/**
       * Initialization process. The "Snap 4" word is required by the Delta converter. If it
       * cannot be sent in 20ms, there is an error in the CAN bus access.
       * @param self The own structure itself.
       */
void deltaSM_state008(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;

    if  (self->can.sendSnapStep4(self))
   {
       self->machine.timeRunning = 0;
       self->machine.estadoActualConvertidor++;
   }
   else if (self->machine.timeRunning > TIMER_20MS)
   {
       event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_SNAP_STEP);
       self->machine.pushErrorWord(self, &event);
       self->machine.timeRunning = 0;
       self->machine.estadoActualConvertidor  = 700;  /** < Machine to error */
   }
};


/**
       * In this function the state machine waits for at least 20ms (as said in
       * Delta specifications). If the waiting time is greater than 80ms there
       * is something wrong with the code.
       * @param self The own structure itself.
       */
void deltaSM_state009(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;

    if  (self->machine.timeRunning > TIMER_20MS && self->machine.timeRunning <= TIMER_80MS)
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_80MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_SNAP_STEP);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error */
    }
};

/**
       * Initialization process. The "RequestId" word is required by the Delta converter. If it
       * cannot be sent in 20ms, there is an error in the CAN bus access.
       * @param self The own structure itself.
       */
void deltaSM_state010(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;


    if  (self->can.sendRequestId(self))
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_20MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_REQUEST_ID);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};



/**
       * In this function the state machine waits for at least 20ms (as said in
       * Delta specifications). If the waiting time is greater than 80ms there
       * is something wrong with the code. There is something more in this function,
       * the converter must be identified before continuing. The timeout restriction
       * is still valid.
       * @param self The own structure itself.
       */
void deltaSM_state011(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;

//    if  (self->machine.timeRunning > TIMER_20MS && self->identifier)
    if  (self->machine.timeRunning > TIMER_20MS)
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_80MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_REQUEST_ID);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};

/**
       * Initialization process. In this function, the Identifier proposed by the
       * Delta module is overwritten according to logical numbering from 1 to
       * the maximum number of converters. After that, it is registered globally.
       * A failure in this function may be related to the number of converters, this
       * value could be greater than the maximum configured.
       * @param self The own structure itself.
       */
void deltaSM_state012(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;

    self->identifier = self->machine.getNonExistingNode(self); /** < Seek a free identifier*/

    if  (self->identifier)
    {
        if (self->machine.pushNewNode(self,self->identifier))
        {
            self->machine.timeRunning = 0;
            self->machine.estadoActualConvertidor++;
        }
        else
        {
            event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_ID_ASSIGNATION);
            self->machine.pushErrorWord(self, &event);
            self->machine.timeRunning = 0;
            self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
        }
    }
    else
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_ID_ASSIGNATION);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};

/**
       * This function sends the new identifier to Delta converter.
       * @param self The own structure itself.
       */
void deltaSM_state013(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;

    if  (self->can.sendAssignId(self))
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_20MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_ID_ASSIGNATION);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};


/**
       * In this function the state machine waits for at least 20ms (as said in
       * Delta specifications). If the waiting time is greater than 80ms there
       * is something wrong with the code.
       * @param self The own structure itself.
       */
void deltaSM_state014(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;

    if  (self->machine.timeRunning > TIMER_20MS && self->machine.timeRunning <= TIMER_80MS)
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor = 17;
    }
    else if (self->machine.timeRunning > TIMER_80MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_REQUEST_ID);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};
/**
       * Not used
       * @param self The own structure itself.
       */
void deltaSM_state015(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;
    self->phyAddr                               = 0;

    if  (self->can.sendWritePhyAddr(self))
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_20MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_ID_ASSIGNATION);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};


/**
       * In this function the state machine waits for at least 20ms (as said in
       * Delta specifications). If the waiting time is greater than 80ms there
       * is something wrong with the code.
       * @param self The own structure itself.
       */
void deltaSM_state016(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;

    if  (self->machine.timeRunning > TIMER_20MS && self->machine.timeRunning <= TIMER_80MS)
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor = 17;
    }
    else if (self->machine.timeRunning > TIMER_80MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_REQUEST_ID);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};



/**
       * This function sends the new identifier to Delta converter.
       * @param self The own structure itself.
       */
void deltaSM_state017(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;
    self->phyAddr                               = 0;

    if  (self->can.sendRequestPhyAddr(self))
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_20MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_ID_ASSIGNATION);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};


/**
       * In this function the state machine waits for at least 20ms (as said in
       * Delta specifications). If the waiting time is greater than 80ms there
       * is something wrong with the code.
       * @param self The own structure itself.
       */
void deltaSM_state018(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;

    if  (self->machine.timeRunning > TIMER_20MS && self->machine.timeRunning <= TIMER_80MS)
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor = 19;
    }
    else if (self->machine.timeRunning > TIMER_80MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_REQUEST_ID);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};





/**
       * This function sends the first ping required by the Delta converter.
       * @param self The own structure itself.
       */
void deltaSM_state019(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;

    if  (self->can.sendPing(self))
    {
        self->pings++;
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_500MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_SENDING_PING);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};

/**
       * This function restarts all variables and simply pass-by. It also clears
       * the flag of seeking converters, because a converter is already assigned.
       * Up to here the converter discovery.
       * @param self The own structure itself.
       */
void deltaSM_state020(DeltaModule *self){

    self->machine.timerPower                        = 0;
    self->machine.timerOut2                         = 0;
    //self->machine.timerPing                       = 0;    //** < Ping timer already running waiting for response
    self->machine.currentRequestTime                = 0;
    self->machine.structureSeekingConverter         = FALSE;
    self->machine.converterFound                    = 0;
    globalNodeDiscoverySemaphore                    = GREEN_SEM;    //** < Other processes can start a new discovery
    self->machine.serialNumberReceivedPartFlag      = 0;

    self->machine.estadoActualConvertidor++;
    self->machine.timeRunning = 0;
    self->machine.num_serie = 0;
};

/**
       * This function asks for the serial number of the converter
       * @param self The own structure itself.
       */
void deltaSM_state021(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;    //** < Ping timer already running
    self->machine.currentRequestTime            = 0;

    if  (self->can.sendRequestSerial1(self))
    {
		self->machine.num_serie++;
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_500MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_GETTING_SERIAL);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};

/**
       * In this function the state machine waits for at least 80ms (as said in
       * Delta specifications). If the waiting time is greater than 1s there
       * is something wrong with the code.
       * @param self The own structure itself.
       * */
void deltaSM_state022(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;    //** < Ping timer already running
    self->machine.currentRequestTime            = 0;

    if  (self->machine.timeRunning > TIMER_80MS && self->machine.timeRunning <= TIMER_1S)
    {
        self->machine.timeRunning = 0;
        if (self->machine.num_serie > 7){
            self->machine.num_serie = 0;
            self->machine.estadoActualConvertidor++;
            self->machine.num_version = 0;
#if DEBUG
            sprintf(bufferstr, "Delta %d present and ready for identification\n", self->identifier);
            env_debug(bufferstr);
            env_debug("\n");
#endif
        }
        else{
            self->machine.estadoActualConvertidor = 21;
        }
    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_GETTING_SERIAL);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};






/**
       * This function asks for the serial number of the converter
       * @param self The own structure itself.
       */
void deltaSM_state023(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;    //** < Ping timer already running
    self->machine.currentRequestTime            = 0;

    if  (self->can.sendRequestVersion(self))
    {
		self->machine.num_version++;
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_500MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_GETTING_SERIAL);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};

/**
       * In this function the state machine waits for at least 80ms (as said in
       * Delta specifications). If the waiting time is greater than 1s there
       * is something wrong with the code.
       * @param self The own structure itself.
       * */
void deltaSM_state024(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;    //** < Ping timer already running
    self->machine.currentRequestTime            = 0;

    if  (self->machine.timeRunning > TIMER_80MS && self->machine.timeRunning <= TIMER_1S)
    {
        self->machine.timeRunning = 0;
        if (self->machine.num_version > 5){
            self->machine.num_version = 0;
            self->machine.estadoActualConvertidor = 27;
        }
        else{
            self->machine.estadoActualConvertidor = 23;
        }
    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_GETTING_SERIAL);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};





#if EPROMS_IN_DELTAS

#define SERIAL_FLAGS_WHEN_ALL_SN_IS_RECEIVED 0xFF
/**     This function checks how many outputs are set. If there is only one, then proceeds with
     *  identification.
     *  If there is the need of PING, it performs the PING procedure.
     *  If the serial number is wrong, then restarts the procedure.
     *  If the serial number was already in memory, the position was recorded previously.
     *  Try to turn the converter on. If the converter gets the on state, that means that is placed
     *  on the activated bay.
     *  @param self The own structure itself.*/

void deltaSM_state027(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;
    self->machine.currentRequestTime            = 0;

    unsigned int enablesActivated = 0;


    if (self->machine.timerPing > TIMER_PING)
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor = 34;     /** < Go to ping!*/
    }
    /**  Check if the serial number is completely received*/
    else if (self->machine.serialNumberReceivedPartFlag != SERIAL_FLAGS_WHEN_ALL_SN_IS_RECEIVED)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_WARNING, ERROR_CODE_GETTING_SERIAL);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor = 21;     /** < Go back for the missing parts!*/
    }
    /** Check if the serial node was already registered */
    else if (checkThisSerial(&(self->serialNumber[0])))
    {
        self->position = checkThisSerial(&(self->serialNumber[0]));
        self->energy.getEnergyLastSessions(self);
        self->errorCounter.getErrors(self);
        self->powerOnCycles++;
        self->commandWord = DEEP_SLEEP_COMMAND | SWITCH_OFF_COMMAND;  // < Power OFF before transition
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor = 100;     /** < The node was previously identified, no more work to do, go to waiting state */
#if DEBUG
    		env_debug("ya existe...");
    		env_debug("\n");
#endif
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_INFO, INFO_CODE_CONVERTER_READY);
    }
    /** The node was unknown, if there is only one output activated, proceed with identification */
    else if (self->machine.timeRunning > TIMER_1S + TIMER_500MS)
    {
                     // self->position = shiftRelays + 1;
                    self->position = self->phyAddr;

#if DEBUG
            sprintf(bufferstr, "Delta %d no existia\n", self->identifier);
            env_debug(bufferstr);
            env_debug("\n");

#endif
                    pushThisSerial(&(self->serialNumber[0]),self->position); /** < Register this serial in this position */
                    self->energy.getEnergyLastSessions(self);
                    self->errorCounter.getErrors(self);

    }
    else self->machine.estadoActualConvertidor = 27;    /** < Stay */
};

#else


#define SERIAL_FLAGS_WHEN_ALL_SN_IS_RECEIVED 0xFF
/**     This function checks how many outputs are set. If there is only one, then proceeds with
     *  identification.
     *  If there is the need of PING, it performs the PING procedure.
     *  If the serial number is wrong, then restarts the procedure.
     *  If the serial number was already in memory, the position was recorded previously.
     *  Try to turn the converter on. If the converter gets the on state, that means that is placed
     *  on the activated bay.
     *  @param self The own structure itself.*/
void deltaSM_state027(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;
    self->machine.currentRequestTime            = 0;

    unsigned int enablesActivated = 0;
    unsigned int shiftRelays;
    /** If there is only one output activated, totalSumOnes equals to one */
    for (shiftRelays = 0; shiftRelays < maxNumberOfConvertersWorking; shiftRelays++)
    {
        enablesActivated += ((activeOutputRelaysVector >> shiftRelays) & 0x00000001);
    };
    if (enablesActivated != 1) {
      self->machine.timeRunning = 0;
    }

    if (self->machine.timerPing > TIMER_PING)
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor = 34;     /** < Go to ping!*/
    }
    /**  Check if the serial number is completely received*/
    else if (self->machine.serialNumberReceivedPartFlag != SERIAL_FLAGS_WHEN_ALL_SN_IS_RECEIVED)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_WARNING, ERROR_CODE_GETTING_SERIAL);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor = 21;     /** < Go back for the missing parts!*/
    }
    /** Check if the serial node was already registered */
    else if (checkThisSerial(&(self->serialNumber[0])))
    {
        self->position = checkThisSerial(&(self->serialNumber[0]));
        self->energy.getEnergyLastSessions(self);
        self->errorCounter.getErrors(self);
        self->powerOnCycles++;
        self->commandWord = DEEP_SLEEP_COMMAND | SWITCH_OFF_COMMAND;  // < Power OFF before transition
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor = 100;     /** < The node was previously identified, no more work to do, go to waiting state */
#if DEBUG
    		env_debug("ya existe...");
    		env_debug("\n");
#endif
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_INFO, INFO_CODE_CONVERTER_READY);
    }
    /** The node was unknown, if there is only one output activated, proceed with identification */
    else if (enablesActivated == 1 && self->machine.timeRunning > TIMER_1S + TIMER_500MS)
    {
        self->commandWord = SWITCH_ON_COMMAND;
       // self->statusWord = SWITCHED_ON_STATUS;

#if SIMUL_DELTAS
        self->statusWord = 0;
        for (shiftRelays = 0; shiftRelays < maxNumberOfConvertersWorking; shiftRelays++)
        {
            if ((activeOutputRelaysVector >> shiftRelays) & 0x00000001)
            {
            	if (self->identifier == shiftRelays + 1   )
            		self->statusWord = SWITCHED_ON_STATUS;
            }
        }

#endif


        if ((self->statusWord & SWITCHED_ON_STATUS) == SWITCHED_ON_STATUS)
        {

            for (shiftRelays = 0; shiftRelays < maxNumberOfConvertersWorking; shiftRelays++)
            {

        	if ((activeOutputRelaysVector >> shiftRelays) & 0x00000001)
                {

                    discoveredBaysWithModule |= activeOutputRelaysVector;
                    self->position = shiftRelays + 1;

                    pushThisSerial(&(self->serialNumber[0]),shiftRelays + 1); /** < Register this serial in this position */
                    self->energy.getEnergyLastSessions(self);
                    self->errorCounter.getErrors(self);
               }
            }
        }
        else
        {
            self->machine.timeRunning = 0;
            self->machine.estadoActualConvertidor++;
        }

    }
    else self->machine.estadoActualConvertidor = 27;    /** < Stay */
};



#endif

/**
       * This function sends a power set point until it succeeds.
       * @param self The own structure itself.
       */
void deltaSM_state028(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;
    self->machine.currentRequestTime            = 0;

    self->currentSetPoint       = 1.0F;
    self->voltageSetPoint       = 60.0F;
    self->powerSetPoint         = 1000.0F;
    /** Send command. If it has been sent successfully, reset the parameters */
    if  (self->can.sendVoltageCurrentPower(self))
    {
        self->currentSetPoint   = 0.0F;
        self->voltageSetPoint   = 0.0F;
        self->outputPower       = 0.0F;
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_500MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_GETTING_SERIAL);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};

/**
       * In this function the state machine waits for at least 80ms (as said in
       * Delta specifications). If the waiting time is greater than 1s there
       * is something wrong with the code.
       * @param self The own structure itself.
       * */
void deltaSM_state029(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;    //** < Ping timer already running
    self->machine.currentRequestTime            = 0;

    if  (self->machine.timeRunning > TIMER_80MS && self->machine.timeRunning <= TIMER_1S)
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_80MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_GETTING_SERIAL);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};

/**
       * This function sends the command to start switching.
       * @param self The own structure itself.
       */
void deltaSM_state030(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;
    self->machine.currentRequestTime            = 0;

    if  (self->can.sendControlCommand(self,REMOTE_ON_COMMAND)) // < Turn on!
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_500MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_GETTING_SERIAL);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};

/**
       * In this function the state machine waits for at least 80ms (as said in
       * Delta specifications). If the waiting time is greater than 1s there
       * is something wrong with the code.
       * @param self The own structure itself.
       */
void deltaSM_state031(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;    //** < Ping timer already running
    self->machine.currentRequestTime            = 0;

    if  (self->machine.timeRunning > TIMER_80MS && self->machine.timeRunning <= TIMER_1S)
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_80MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_GETTING_SERIAL);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};



/**
       * This function sends the command to start switching.
       * @param self The own structure itself.
       */
void deltaSM_state032(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;
    self->machine.currentRequestTime            = 0;

    if  (self->can.sendControlCommand(self,REMOTE_ON_COMMAND)) // < Turn on!
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_500MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_GETTING_SERIAL);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};

/**
       * In this function the state machine waits for at least 80ms (as said in
       * Delta specifications). If the waiting time is greater than 1s there
       * is something wrong with the code.
       * @param self The own structure itself.
       * */
void deltaSM_state033(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;    //** < Ping timer already running
    self->machine.currentRequestTime            = 0;

    if  (self->machine.timeRunning > TIMER_80MS && self->machine.timeRunning <= TIMER_1S)
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor = 27; /** < Go back to identification to check if the converter has started.*/
    }
    else if (self->machine.timeRunning > TIMER_500MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_GETTING_SERIAL);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};

/**
       * This function sends a ping to keep the converter alive.
       * @param self The own structure itself.
       */
void deltaSM_state034(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    //self->machine.timerOut2                     = 0; /** < Pong timeout    */
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;

    if  (self->can.sendPing(self))
    {
        self->machine.pong = 0;     /** < Wait for pong*/
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
        self->pings++;
    }
    else if (self->machine.timeRunning > TIMER_500MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_PING_SENDING_FAIL);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};

/**
       * In this function the state machine waits for at least 200ms (as said in
       * Delta specifications). If the waiting time is greater than 1s it will try to re-ping, but
       * in case of being higher than timeout, it will go to error.
       * @param self The own structure itself.
       */
void deltaSM_state035(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    //self->machine.timerOut2                   = 0;
    //self->machine.timerPing                   = 0;
    self->machine.currentRequestTime            = 0;

    if  (self->machine.pong && self->machine.timeRunning > TIMER_200MS)
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor = 27; /** < Go back to identification to check if the converter has started.*/
    }
    else if (self->machine.timeRunning > TIMER_1S)    /** < Try a ping again */
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor--;
    }
    else if (self->machine.timerOut2 > TIMER_PING_TIMEOUT)  /** < The node does not exist anymore */
    {
    	if (empieza_carga > 0)
    		event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_PONG_RECEIVE_FAIL);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 701;  /** < Machine to error state, lost converter*/
    }
};

/**
       * This function initializes all variables related to energy
       * @param self The own structure itself.
       */
void deltaSM_state100(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0; /* < Ping timer is already running
    self->machine.currentRequestTime            = 0;

    self->energy.powerFloat                     = 0.0;
    self->energy.energyPendingToIncrement       = 0.0;
    self->outputPower                           = 0;
    self->sessionEnergy                         = 0;
    self->energy.calcAvailableCurrent(self);


    if (self->machine.timerPing > TIMER_PING || self->machine.timeRunning > TIMER_50MS || \
            self->machine.cmdShadowCopy     != self->commandWord        )
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;        //  < This will send again the command word and ping
    }
    else if ((self->statusWord & ERROR_STATUS) == ERROR_STATUS)
    {
        self->errorDataWordStd = errorCodeDelta2Circontrol(self->errorDataWord);
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_ERROR, self->errorDataWordStd);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 702; /** < Error with the converter already detected*/
    }
};

/**
       * When there is the need of PING o to start the converter, the sequence is started.
       * In this case the Voltage/current/power command is sent.
       * @param self The own structure itself.
       */
void deltaSM_state101(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;
    self->machine.currentRequestTime            = 0;

    if  (self->can.sendVoltageCurrentPower(self))
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;        //  < This will send again the command word and ping
    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_ERROR, ERROR_CODE_CAN_UNREACHABLE_MSGS_BLOCKED);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 702;  /** < Machine to error state, lost converter*/
    }
};

/**
       * In this function the state machine waits for at least 80ms (as said in
       * Delta specifications). If the waiting time is greater than 1s there
       * is something wrong with the code.
       * @param self The own structure itself.
       * */
void deltaSM_state102(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;    //** < Ping timer already running
    self->machine.currentRequestTime            = 0;

    if  (self->machine.timeRunning > TIMER_20MS && self->machine.timeRunning <= TIMER_1S)
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_ERROR, ERROR_CODE_CAN_UNREACHABLE_MSGS_BLOCKED);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 702;  /** < Machine to error state*/
    }
};

/**
       * Sends the command word.
       * @param self The own structure itself.
       */
void deltaSM_state103(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;
    self->machine.currentRequestTime            = 0;

    int theCommand = REMOTE_ON_COMMAND;                                 /** < Turns on the Delta module */
    if (    ((self->commandWord & SWITCH_OFF_COMMAND)    == SWITCH_OFF_COMMAND) &&\
            ((self->commandWord & DEEP_SLEEP_COMMAND)    == DEEP_SLEEP_COMMAND))
    {
        theCommand = REMOTE_OFF_STANDBY_COMMAND;
    	//theCommand = SLOW_REMOTE_OFF_COMMAND;
    }

    if  (self->can.sendControlCommand(self,theCommand))
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_ERROR, ERROR_CODE_CAN_UNREACHABLE_MSGS_BLOCKED);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 702;  /** < Machine to error state*/
    }
};

/**
       * In this function the state machine waits for at least 80ms (as said in
       * Delta specifications). If the waiting time is greater than 1s there
       * is something wrong with the code.
       * @param self The own structure itself.
       * */
void deltaSM_state104(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;    //** < Ping timer already running
    self->machine.currentRequestTime            = 0;

    if  (self->machine.timeRunning > TIMER_20MS && self->machine.timeRunning <= TIMER_1S)
    {
        self->machine.timeRunning = 0;
        if ((self->desiredPhyAddr != 0) && (self->desiredPhyAddr != self->phyAddr)){
        	self->machine.estadoActualConvertidor = 107;
        }
        else{
        	self->machine.estadoActualConvertidor = 107;
        }

    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_ERROR, ERROR_CODE_CAN_UNREACHABLE_MSGS_BLOCKED);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 702;  /** < Machine to error state*/
    }
};

/**
       * This function sends the new identifier to Delta converter.
       * @param self The own structure itself.
       */
void deltaSM_state105(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;
    self->phyAddr                               = 0;

    if  (self->can.sendWritePhyAddr(self))
    {
    	self->desiredPhyAddr = 0;
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_20MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_ID_ASSIGNATION);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};



/**
       * In this function the state machine waits for at least 20ms (as said in
       * Delta specifications). If the waiting time is greater than 80ms there
       * is something wrong with the code.
       * @param self The own structure itself.
       */
void deltaSM_state106(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;

    if  (self->machine.timeRunning > TIMER_20MS && self->machine.timeRunning <= TIMER_80MS)
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor = 107;
    }
    else if (self->machine.timeRunning > TIMER_80MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_REQUEST_ID);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};

/**
       * Read current
       * @param self The own structure itself.
       */
void deltaSM_state107(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;    //** < Ping timer already running
    self->machine.currentRequestTime            = 0;

    if  (self->can.sendRequestForVoltage(self))
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;        //  El siguiente estado es el actual +1
    }
    else if (self->machine.timeRunning > TIMER_500MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_ERROR, ERROR_CODE_CAN_UNREACHABLE_MSGS_BLOCKED);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 702;  /** < Machine to error state */
    }
};


/**
       * In this function the state machine waits for at least 20ms (as said in
       * Delta specifications). If the waiting time is greater than 1s there
       * is something wrong with the code.
       * @param self The own structure itself.
       */
void deltaSM_state108(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;    //** < Ping timer already running
    self->machine.currentRequestTime            = 0;

    if  (self->machine.timeRunning > TIMER_20MS && self->machine.timeRunning <= TIMER_1S)
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor = 120;
    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_ERROR, ERROR_CODE_CAN_UNREACHABLE_MSGS_BLOCKED);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 702;  /** < Machine to error state*/
    }
};


/**
       * Ask for the temperature of the primary
       * @param self The own structure itself.
       */
void deltaSM_state120(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;
    self->machine.currentRequestTime            = 0;

    if  (self->can.sendRequestData(self))
    {

        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;        //  El siguiente estado es el actual +1
    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_ERROR, ERROR_CODE_CAN_UNREACHABLE_MSGS_BLOCKED);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 702;  /** < Machine to error state*/
    }
};

/**
       * In this function the state machine waits for at least 80ms (as said in
       * Delta specifications). If the waiting time is greater than 1s there
       * is something wrong with the code.
       * @param self The own structure itself.
       *
       */
void deltaSM_state121(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;    //** < Ping timer already running
    self->machine.currentRequestTime            = 0;

    if  (self->machine.timeRunning > TIMER_20MS && self->machine.timeRunning <= TIMER_1S)
    {
        self->machine.timeRunning = 0;
        if ((self->machine.cmdShadowCopy & SWITCH_ON_COMMAND) == SWITCH_ON_COMMAND) self->machine.estadoActualConvertidor = 200;
        else if (self->machine.timerPing < TIMER_PING) self->machine.estadoActualConvertidor = 100;
        else self->machine.estadoActualConvertidor = 150;
    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_ERROR, ERROR_CODE_CAN_UNREACHABLE_MSGS_BLOCKED);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 702;  /** < Machine to error state*/
    }
};







/**
       * This function sends a ping to keep the converter alive.
       * @param self The own structure itself.
       */
void deltaSM_state150(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    //self->machine.timerOut2                   = 0; /** < Pong timeout    */
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;

    if  (self->can.sendPing(self))
    {
        self->machine.pong = 0;     /** < Wait for pong*/
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
        self->pings++;
    }
    else if (self->machine.timeRunning > TIMER_500MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_PING_SENDING_FAIL);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};

/**
       * In this function the state machine waits for at least 200ms (as said in
       * Delta specifications). If the waiting time is greater than 1s it will try to re-ping, but
       * in case of being higher than timeout, it will go to error.
       * @param self The own structure itself.
       */
void deltaSM_state151(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    //self->machine.timerOut2                   = 0;
    //self->machine.timerPing                   = 0;
    self->machine.currentRequestTime            = 0;

    if  (self->machine.pong)
    {
        self->machine.timeRunning = 0;
        if ((self->commandWord & SWITCH_ON_COMMAND) == SWITCH_ON_COMMAND )
        {
            self->machine.estadoActualConvertidor = 200; /** < Go back to identification to check if the converter has started.*/
            self->machine.currentRequestTime = TIMER_100MS;
        }
        else
        {
            self->machine.estadoActualConvertidor = 100;
        }
    }
    else if (self->machine.timeRunning > (TIMER_1S))
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor--;
    }
    else if (self->machine.timerOut2 > TIMER_PING_TIMEOUT)
   {
    	if (empieza_carga > 0)
    		event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_PONG_RECEIVE_FAIL);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 701;  /** < Machine to error state, lost converter*/
    }
};

/**
       * Start charging
       * @param self The own structure itself.
       */
void deltaSM_state200(DeltaModule *self){
    EventLocation   event;

    self->machine.timeRunning = 0;              /* < This is not a timered state*/
    //self->machine.timerPower                  = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;
    //self->machine.currentRequestTime          = 0;

    self->energy.calcOutputPower(self);
    self->energy.calcAvailableCurrent(self);

    if (self->machine.timerPower > TIMER_1S)
    {
        self->machine.timerPower -= TIMER_1S;
        self->energy.integrateEnergy(self); // < Integrate energy every second
    }

    /** If a ping is needed or there is a change in commands, send new messages */
    if (self->machine.timerPing > TIMER_PING || \
       self->machine.cmdShadowCopy     != self->commandWord        ||\
       self->machine.currentShadowCopy != self->currentSetPoint    ||\
       self->machine.voltageShadowCopy != self->voltageSetPoint    ||\
       self->machine.currentRequestTime > TIMER_50MS) //Was set to 100MS
    {
        self->machine.currentRequestTime            = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if ((self->statusWord & ERROR_STATUS) == ERROR_STATUS)
    {
        self->errorDataWordStd = errorCodeDelta2Circontrol(self->errorDataWord);
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_ERROR, self->errorDataWordStd);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 702; /** < Error with the converter already working*/
    }
    //else: stay forever
};

/**
       * It has been a different command and needs to be refreshed in the converter
       */
void deltaSM_state201(DeltaModule *self){
    EventLocation   event;

    //self->machine.timerPower                  = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;
    //self->machine.currentRequestTime          = 0;

    if  (self->can.sendVoltageCurrentPower(self))
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_ERROR, ERROR_CODE_CAN_UNREACHABLE_MSGS_BLOCKED);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 702;  /** < Machine to error state, lost converter*/
    }
};

/**
       * In this function the state machine waits for at least 20ms (as said in
       * Delta specifications). If the waiting time is greater than 1s there
       * is something wrong with the code.
       * @param self The own structure itself.
       */
void deltaSM_state202(DeltaModule *self){
    EventLocation   event;

    //self->machine.timerPower                  = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;
    //self->machine.currentRequestTime          = 0;


    if  (self->machine.timeRunning > TIMER_20MS && self->machine.timeRunning <= TIMER_1S)
    {
        self->machine.estadoActualConvertidor++;
        self->machine.timeRunning = 0;
    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_ERROR, ERROR_CODE_CAN_UNREACHABLE_MSGS_BLOCKED);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 702;  /** < Machine to error state */
    }
};

/**
       * Send the enable/disable command
       * @param self The own structure itself.
       */
void deltaSM_state203(DeltaModule *self){
    EventLocation   event;

    //self->machine.timerPower                  = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;
    //self->machine.currentRequestTime          = 0;

    int theCommand = 0; /** < On by default */
    if (    ((self->commandWord & SWITCH_OFF_COMMAND)    == SWITCH_OFF_COMMAND) &&\
            ((self->commandWord & DEEP_SLEEP_COMMAND)    == DEEP_SLEEP_COMMAND))
    {
        theCommand = REMOTE_OFF_STANDBY_COMMAND;
    	//theCommand = SLOW_REMOTE_OFF_COMMAND;

    }

//    if  (self->can.sendControlCommand(self,theCommand))
    if (1)
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_ERROR, ERROR_CODE_CAN_UNREACHABLE_MSGS_BLOCKED);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 702;  /** < Machine to error state */
    }
};

/**
       * In this function the state machine waits for at least 20ms (as said in
       * Delta specifications). If the waiting time is greater than 1s there
       * is something wrong with the code.
       * @param self The own structure itself.
       */
void deltaSM_state204(DeltaModule *self){
    EventLocation   event;

    //self->machine.timerPower                  = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;
    //self->machine.currentRequestTime          = 0;


    if  (self->machine.timeRunning > TIMER_20MS && self->machine.timeRunning <= TIMER_1S)
    {
        self->machine.estadoActualConvertidor++;
        self->machine.timeRunning = 0;
    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_ERROR, ERROR_CODE_CAN_UNREACHABLE_MSGS_BLOCKED);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 702;  /** < Machine to error state */
    }
};



/**
       * Send the enable/disable command
       * @param self The own structure itself.
       */
void deltaSM_state205(DeltaModule *self){
    EventLocation   event;

    //self->machine.timerPower                  = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;
    //self->machine.currentRequestTime          = 0;

    int theCommand = 0; /** < On by default */
    if (    ((self->commandWord & SWITCH_OFF_COMMAND)    == SWITCH_OFF_COMMAND) &&\
            ((self->commandWord & DEEP_SLEEP_COMMAND)    == DEEP_SLEEP_COMMAND))
    {
        theCommand = REMOTE_OFF_STANDBY_COMMAND;
    	//theCommand = SLOW_REMOTE_OFF_COMMAND;

    }

    if  (self->can.sendControlCommand(self,theCommand))
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_ERROR, ERROR_CODE_CAN_UNREACHABLE_MSGS_BLOCKED);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 702;  /** < Machine to error state */
    }
};

/**
       * In this function the state machine waits for at least 20ms (as said in
       * Delta specifications). If the waiting time is greater than 1s there
       * is something wrong with the code.
       * @param self The own structure itself.
       */
void deltaSM_state206(DeltaModule *self){
    EventLocation   event;

    //self->machine.timerPower                  = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;
    //self->machine.currentRequestTime          = 0;


    if  (self->machine.timeRunning > TIMER_20MS && self->machine.timeRunning <= TIMER_1S)
    {
        self->machine.estadoActualConvertidor = 207;
        self->machine.timeRunning = 0;
    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_ERROR, ERROR_CODE_CAN_UNREACHABLE_MSGS_BLOCKED);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 702;  /** < Machine to error state */
    }
};

/**
       * Read current
       * @param self The own structure itself.
       */
void deltaSM_state207(DeltaModule *self){
    EventLocation   event;

    //self->machine.timerPower                  = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;
    //self->machine.currentRequestTime          = 0;

    if  (self->can.sendRequestForVoltage(self))
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;

    }
    else if (self->machine.timeRunning > TIMER_500MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_ERROR, ERROR_CODE_CAN_UNREACHABLE_MSGS_BLOCKED);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 702;  /** < Machine to error state */
    }
};

/**
       * In this function the state machine waits for at least 20ms (as said in
       * Delta specifications). If the waiting time is greater than 1s there
       * is something wrong with the code.
       * @param self The own structure itself.
       */
void deltaSM_state208(DeltaModule *self){
    EventLocation   event;

    //self->machine.timerPower                  = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;
    //self->machine.currentRequestTime          = 0;


    if  (self->machine.timeRunning > TIMER_20MS && self->machine.timeRunning <= TIMER_1S)
    {
        self->machine.estadoActualConvertidor = 220;
        self->machine.timeRunning = 0;
    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_ERROR, ERROR_CODE_CAN_UNREACHABLE_MSGS_BLOCKED);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 702;  /** < Machine to error state */
    }
};

/**
       * Start requesting temperatures
       * @param self The own structure itself.
       */
void deltaSM_state220(DeltaModule *self){
    EventLocation   event;

    //self->machine.timerPower                  = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;
    //self->machine.startUpTime                 = 0;

    if  (self->can.sendRequestData(self))
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
#if	DEBUG_ESTADOS_DELTAS1
        if (self->yi == 0){

            sprintf(bufferstr, "ValornumData: %x \n", self->machine.num_data);

            env_debug(bufferstr);

            env_debug("\n");


        }

#endif
    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_ERROR, ERROR_CODE_CAN_UNREACHABLE_MSGS_BLOCKED);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 702;  /** < Machine to error state */
    }
};

/**
       * In this function the state machine waits for at least 20ms (as said in
       * Delta specifications). If the waiting time is greater than 1s there
       * is something wrong with the code.
       * @param self The own structure itself.
       */
void deltaSM_state221(DeltaModule *self){
    EventLocation   event;

    //self->machine.timerPower                  = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;
    //self->machine.currentRequestTime          = 0;


    if  (self->machine.timeRunning > TIMER_20MS && self->machine.timeRunning <= TIMER_1S)
    {
        self->machine.timeRunning = 0;
         if ((self->machine.cmdShadowCopy & SWITCH_ON_COMMAND) == 0) self->machine.estadoActualConvertidor = 252;
         else if (self->machine.timerPing < TIMER_PING) self->machine.estadoActualConvertidor = 200;
         else self->machine.estadoActualConvertidor = 250;

        self->machine.timeRunning = 0;


    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_ERROR, ERROR_CODE_CAN_UNREACHABLE_MSGS_BLOCKED);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 702;  /** < Machine to error state */
    }
};








/**
       * Send ping
       * @param self The own structure itself.
       */
void deltaSM_state250(DeltaModule *self){
    EventLocation   event;

    //self->machine.timerPower                  = 0;
    //self->machine.timerOut2                   = 0; /** < Pong timeout    */
    self->machine.timerPing                     = 0;
    //self->machine.currentRequestTime          = 0;

    if  (self->can.sendPing(self))
    {
        self->machine.pong = 0;     /** < Wait for pong*/
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor++;
        self->pings++;
    }

    else if (self->machine.timeRunning > TIMER_500MS)
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_PING_SENDING_FAIL);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 700;  /** < Machine to error state*/
    }
};

/**
       * Just wait for pong
       * @param self The own structure itself.
       */
void deltaSM_state251(DeltaModule *self){
    EventLocation   event;

    //self->machine.timerPower                  = 0;
    //self->machine.timerOut2                   = 0;
    //self->machine.timerPing                   = 0;
    //self->machine.currentRequestTime          = 0;

    if  (self->machine.pong)
    {
        self->machine.timeRunning = 0;
        if ((self->machine.cmdShadowCopy & SWITCH_ON_COMMAND) == SWITCH_ON_COMMAND )
        {
            self->machine.estadoActualConvertidor = 200;    /** < Redirect depending on commands; still ON */
        }
        else
        {
            self->machine.estadoActualConvertidor++;        /** < Redirect depending on commands; turn OFF */
        }
    }
    else if (self->machine.timeRunning > (TIMER_1S))
    {
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor--;
    }
    else if (self->machine.timerOut2 > TIMER_PING_TIMEOUT)
    {
    	if (empieza_carga > 0)
    		event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_ERROR, ERROR_CODE_PONG_RECEIVE_FAIL);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 701;  /** < Machine to error state, lost converter*/
    }
};

/**
       * End of charge charge
       * @param self The own structure itself.
       */
void deltaSM_state252(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    //self->machine.timerPing                   = 0;
    //self->machine.startUpTime                 = 0;

    if(self->energy.storeEnergyThisSession(self))
    {
        event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_GLOBAL, SEVERITY_INFO, SUB_SUB_SYTEM_DELTA_GLOBAL);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor = 100;  /** < Go back to state 100 */
        //event = createEvent(own_node, SUB_SYSTEM_EEPROM, SUB_SUB_SYTEM_WRITE, INFO, ERROR_CODE_EEPROM_FAILS_TO_WRITE);
    }
    else
    {
        event = createEvent(own_node, SUB_SYSTEM_EEPROM, SUB_SUB_SYTEM_WRITE, SEVERITY_ERROR, ERROR_CODE_EEPROM_FAILS_TO_WRITE);
        self->machine.pushErrorWord(self, &event);
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor  = 702;  /** < Machine to error state, lost converter*/
    }
};



/**
     * This is a state transition error. Before shutdown, the converter must clear the flags of detection,
     * because this transition is activated when there was a node discovery
     */
void deltaSM_state700(DeltaModule *self){

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.currentRequestTime            = 0;
    self->statusWord                            |= ERROR_STATUS;
    self->position                              = 0; //Position is no longer occupied

    if (self->machine.converterFound == 1)
    {
        self->machine.structureSeekingConverter    = 0;
        self->machine.converterFound = 0;
        self->machine.destroyExistingNode(self,self->identifier );
        self->identifier                                = 0;
        globalNodeDiscoverySemaphore                    = GREEN_SEM;
    }

    self->machine.estadoActualConvertidor  = 800;
};

/**
     * This is a state transition error. Converter is lost, there is no ping response
     */
void deltaSM_state701(DeltaModule *self){


    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.currentRequestTime            = 0;
    self->statusWord                            |= ERROR_STATUS;

    self->energy.storeEnergyThisSession(self);
    self->energy.calcAvailableCurrent(self);
    self->errorCounter.pushErrors(self);

    self->machine.estadoActualConvertidor  = 802;
};


/**
     * This is a state transition error. Converter is lost, there is no ping response
     */
void deltaSM_state702(DeltaModule *self){

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.currentRequestTime            = 0;
    self->statusWord                            |= ERROR_STATUS;

    self->energy.storeEnergyThisSession(self);
    self->energy.calcAvailableCurrent(self);
    self->errorCounter.pushErrors(self);

    self->machine.estadoActualConvertidor  = 800;
};


/**
 *      This is the emergency state by default
 * */
void deltaSM_state800(DeltaModule *self){

    self->machine.timeRunning           = 0;
    self->machine.timerPower            = 0;
    //self->machine.timerOut2           = 0; //Used as a disable time
    self->machine.currentRequestTime    = 0;
    self->statusWord                    |= ERROR_STATUS;
    self->currentSetPoint               = 0;
    self->voltageSetPoint               = 0;
    self->commandWord                   = SWITCH_OFF_COMMAND | DEEP_SLEEP_COMMAND;
    self->energy.calcAvailableCurrent(self);

    //If the node was not identified, make a reset now
    if (self->identifier == 0)
    {
        self->machine.estadoActualConvertidor  = 0;
        return;
    }

    unsigned long rmmainStatus            = (self->errorDataWord & 0x00000F00) >> 8;
    /* Disable time calculation */
    // If there are 3 or more errors per minute, disable 1 minute...
    if (deltaSM_getErrorsInLastMinutes(self, 1) >= 3) self->errorCounter.disableTime = systemEpoch + 60;
    if (deltaSM_getErrorsInLastMinutes(self, 5) >= 4) self->errorCounter.disableTime = systemEpoch + 120;
    if (deltaSM_getErrorsInLastMinutes(self, 10) >= 10) self->errorCounter.disableTime = systemEpoch + 300;

    if( systemEpoch > self->errorCounter.disableTime)
    {
        if (rmmainStatus == 0 || rmmainStatus == 1 || rmmainStatus == 2)    // This is a temporal error, try a reset now
        {
            self->machine.estadoActualConvertidor = 801;
        }
        if (rmmainStatus == 2)    // This is a temporal error, but still present
        {
            self->machine.estadoActualConvertidor = 800;
        }
        if (rmmainStatus == 3)  // This is a low severity error, try a reset now
        {
            self->machine.estadoActualConvertidor = 801;
        }
        if (rmmainStatus == 4)  // This is a medium severity error, and it is still present. Wait.
        {
            self->machine.estadoActualConvertidor = 800;
        }
        if (rmmainStatus == 5)  // This is a medium severity error, and it is no longer present. Reset.
        {
            self->machine.estadoActualConvertidor = 801;
        }
        if (rmmainStatus == 6 || rmmainStatus == 8)  // This is a High severity error, wait
        {
            self->machine.estadoActualConvertidor       = 800;
        }
    }

    /**
     * If the maximum time in error is reached, try a reset
     */
    if (self->machine.timerOut2 >= MAXIMUM_TIME_IN_ERROR)
    {
        self->machine.estadoActualConvertidor = 801;
        return;
    }

    //Is there the need of a ping?
    if (self->machine.timerPing > TIMER_PING)
    {
        self->machine.estadoActualConvertidor = 850;
    }
};

/**
     *      This is the transition to reset
     */
void deltaSM_state801(DeltaModule *self){

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;
    self->statusWord                            |= ERROR_STATUS;

    if  (1)
    {
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        self->machine.estadoActualConvertidor = 802; // Proceed anyway
    }
};

/**
     *      Initializes variables to make a reset
     */
void deltaSM_state802(DeltaModule *self){
    EventLocation   event;

    self->machine.timerPower                    = 0;
    self->machine.timerOut2                     = 0;
    self->machine.timerPing                     = 0;
    self->machine.currentRequestTime            = 0;
    self->statusWord                            |= ERROR_STATUS;

    self->machine.estadoActualConvertidor       = 0;
    self->currentSetPoint                       = 0;
    self->voltageSetPoint                       = 0;
    self->powerSetPoint                         = 0;
    self->statusWord                            = 0;
    self->commandWord                           = 0;
    self->outputVoltage                         = 0;
    self->outputCurrent                         = 0;
    self->outputPower                           = 0;
    self->errorDataWord                         = LATCHED_OFF_NOT_PRESENT_UNKNOWN_ERROR_WORD;
    self->availableCurrent                      = 0;
    self->protocoloAsignado                     = 0;
    self->temperatureAir                        = 0;
    self->temperaturePrimary                    = 0;
    self->temperatureSecondary                  = 0;
    //bye bye
    self->machine.destroyExistingNode(self,self->identifier);
    self->identifier                            = 0;
    self->position                              = 0;

    event = createEvent(own_node, SUB_SYSTEM_DELTA, SUB_SUB_SYTEM_DELTA_NODE, SEVERITY_INFO, INFO_MAKING_A_RESET);
    self->machine.pushErrorWord(self, &event);

};

/**
 *      Keep asking the node
 *      */
void deltaSM_state850(DeltaModule *self){
    self->machine.currentRequestTime                   = 0;

    if  (self->can.sendVoltageCurrentPower(self))
    {
        self->machine.currentShadowCopy = self->currentSetPoint;
        self->machine.voltageShadowCopy = self->voltageSetPoint;
        self->machine.timeRunning           = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        self->machine.timeRunning           = 0;
        self->machine.estadoActualConvertidor++;
    }
};

/**
       * In this function the state machine waits for at least 20ms (as said in
       * Delta specifications). If the waiting time is greater than 1s there
       * is something wrong with the code.
       * @param self The own structure itself.
       */
void deltaSM_state851(DeltaModule *self){
    self->machine.currentRequestTime                   = 0;

    if  (self->machine.timeRunning > TIMER_80MS)
    {
        self->machine.timeRunning           = 0;
        self->machine.estadoActualConvertidor++;
    }
};

/**
       * This function sends the control command (it will force a shutdown)
       * @param self The own structure itself.
       */
void deltaSM_state852(DeltaModule *self){
    self->machine.currentRequestTime        = 0;

    if  (self->can.sendControlCommand(self,1))//Force shutdown
    {
        self->machine.timeRunning           = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        self->machine.timeRunning           = 0;
        self->machine.estadoActualConvertidor++;
    }
};


/**
       * In this function the state machine waits for at least 20ms (as said in
       * Delta specifications). If the waiting time is greater than 1s there
       * is something wrong with the code.
       * @param self The own structure itself.
       */
void deltaSM_state853(DeltaModule *self){
    self->machine.currentRequestTime        = 0;

    if  (self->machine.timeRunning > TIMER_80MS)
    {
        self->machine.timeRunning           = 0;
        self->machine.estadoActualConvertidor++;
    }
};


/**
       * This function sends the control command (it will force a shutdown)
       * @param self The own structure itself.
       */
void deltaSM_state854(DeltaModule *self){
    self->machine.currentRequestTime        = 0;

    if  (self->can.sendControlCommand(self,1))//Force shutdown
    {
        self->machine.timeRunning           = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        self->machine.timeRunning           = 0;
        self->machine.estadoActualConvertidor++;
    }
};

/**
       * In this function the state machine waits for at least 20ms (as said in
       * Delta specifications). If the waiting time is greater than 1s there
       * is something wrong with the code.
       * @param self The own structure itself.
       */
void deltaSM_state855(DeltaModule *self){
    self->machine.currentRequestTime                   = 0;

    if  (self->machine.timeRunning > TIMER_80MS)
    {
        self->machine.timeRunning           = 0;
        self->machine.estadoActualConvertidor++;
    }
};


/**
       * This function asks for temperatures
       * @param self The own structure itself.
       */
void deltaSM_state856(DeltaModule *self){
    self->machine.currentRequestTime                   = 0;

    if  (self->can.sendRequestData(self))
    {
        self->machine.timeRunning           = 0;
        self->machine.estadoActualConvertidor++;
    }
    else if (self->machine.timeRunning > TIMER_1S)
    {
        self->machine.timeRunning           = 0;
        self->machine.estadoActualConvertidor++;
    }
};

/**
       * In this function the state machine waits for at least 20ms (as said in
       * Delta specifications). If the waiting time is greater than 1s there
       * is something wrong with the code.
       * @param self The own structure itself.
       */
void deltaSM_state857(DeltaModule *self){
    self->machine.currentRequestTime                   = 0;

    if  (self->machine.timeRunning > TIMER_80MS)
    {
        self->machine.timeRunning           = 0;
        self->machine.timeRunning = 0;
        self->machine.estadoActualConvertidor = 870;
    }
};






/**
       * This function sends the ping
       * @param self The own structure itself.
       */
void deltaSM_state870(DeltaModule *self){
    self->machine.currentRequestTime = 0;

    if  (self->can.sendPing(self))
    {
        self->machine.timerPing = 0;
        self->machine.pong = 0;
        self->machine.timeRunning  = 0;
        self->machine.estadoActualConvertidor++;
        self->pings++;
    }

    else if (self->machine.timeRunning > TIMER_500MS)
    {
        self->machine.timeRunning           = 0;
        self->machine.estadoActualConvertidor++;
    }
};


/**
       * This function waits for pong
       * @param self The own structure itself.
       */
void deltaSM_state871(DeltaModule *self){
    self->machine.currentRequestTime        = 0;

    if  (self->machine.pong)
    {
        self->machine.estadoActualConvertidor = 800; // Keep the error state
        self->machine.pong = 0;
    }
    else if (self->machine.timeRunning > TIMER_1S) // Re-send
    {
        self->machine.timeRunning           = 0;
        self->machine.estadoActualConvertidor--;
    }
    else if (self->machine.timerOut2 > TIMER_PING_TIMEOUT)  //Node lost
    {
        self->machine.timeRunning           = 0;
        self->machine.estadoActualConvertidor = 802; //Destroy the node
    }
};


// Other useful functions
// This function checks if the node already exists or not. During the identification phase (0..100)
// the Delta modules propose an ID. It may happen that two different modules propose the same ID and if that
//  happens, this function will return a "true" value.
unsigned int deltaSM_checkIfNodeExists(DeltaModule *self, const unsigned int proposedNode){
    unsigned int existe = 0;
    int indexNodo;

    for (indexNodo = 0; indexNodo < maxNumberOfConvertersWorking; indexNodo++) {
        if (nodeList[indexNodo] == proposedNode) existe = 1;
    }
    return (existe);
}

// Following the same logic from the previous function, when two IDs are equal, a new ID must be provided to
// the module with repeated ID. This function returns an unused ID.
unsigned int deltaSM_getNonExistingNode(DeltaModule *self){
    unsigned int nodeNumbering;

    for (nodeNumbering = 1; nodeNumbering <= MAXIMUM_NUMBER_OF_CONVERTERS; nodeNumbering++){
        if (self->machine.checkIfNodeExists(self,nodeNumbering) == 0) return (nodeNumbering);
    };

    return (0);
};

// When a new ID is assigned to a new module, this ID must be registered to avoid duplicities. This function
// registers all used IDs.
unsigned int deltaSM_pushNewNode(DeltaModule *self, const unsigned int proposedNode){
    unsigned int pushed = 0;
    int indexNodo;

    for (indexNodo = 0; indexNodo < maxNumberOfConvertersWorking; indexNodo++) {
        if (nodeList[indexNodo] == 0)
        {
            nodeList[indexNodo] = proposedNode;
            pushed = 1;
            return (pushed);
        }
    }
    return (pushed);
};

// When a module is disconnected from the system i.e. when communication is lost with that module, the converter
// ID must be removed from the list, because the module is no longer present.
void deltaSM_destroyExistingNode(DeltaModule *self, const unsigned int nodeNumber){
    int indexNodo;

    for (indexNodo = 0; indexNodo < maxNumberOfConvertersWorking; indexNodo++) {
        if (nodeList[indexNodo] == nodeNumber) nodeList[indexNodo] = 0;
    }
};


//  Esta funci�n registra por donde ha pasado la m�quina de estados, sirve de ayuda para ver en que punto falla y
//  el camino seguido.
/*void deltaSM_pushLastState(DeltaModule *self){
//  ESTADO_ANTERIOR_CONVERTIDOR_PRE[9]  = ESTADO_ANTERIOR_CONVERTIDOR_PRE[8];
    self->machine.estadoAnteriorConvertidor[8]  = self->machine.estadoAnteriorConvertidor[7];
    self->machine.estadoAnteriorConvertidor[7]  = self->machine.estadoAnteriorConvertidor[6];
    self->machine.estadoAnteriorConvertidor[6]  = self->machine.estadoAnteriorConvertidor[5];
    self->machine.estadoAnteriorConvertidor[5]  = self->machine.estadoAnteriorConvertidor[4];
    self->machine.estadoAnteriorConvertidor[4]  = self->machine.estadoAnteriorConvertidor[3];
    self->machine.estadoAnteriorConvertidor[3]  = self->machine.estadoAnteriorConvertidor[2];
    self->machine.estadoAnteriorConvertidor[2]  = self->machine.estadoAnteriorConvertidor[1];
    self->machine.estadoAnteriorConvertidor[1]  = self->machine.estadoAnteriorConvertidor[0];
    self->machine.estadoAnteriorConvertidor[0]  = self->machine.estadoActualConvertidor;
};*/

// This function manages errors in modules. It creates an event and saves it in the system memory. The error list
// is a FIFO and all errors are listed from most recent to ancient.
void deltaSM_pushErrorWord (DeltaModule *self, EventLocation *error)
{
    EventWord errorWord;
    int i;

    if (error->severity == SEVERITY_ERROR || error->severity == SEVERITY_FATAL) {
        errorWord.eventLocation.hardware        = error->hardware;
        errorWord.eventLocation.subsystem       = error->subsystem;
        errorWord.eventLocation.subsubsystem    = error->subsubsystem;
        errorWord.eventLocation.eventCode       = error->eventCode;
        errorWord.eventLocation.severity        = error->severity;
        errorWord.epochTime                     = systemEpoch;
        //errorWord.uniqueIdentifier              = assignNewErrorIdentifier();
    }
    else return;

    for(i = 9; i >= 1; i--) {
        deltaSM_errorCpy(&self->machine.erroresAnteriores[i-1], &self->machine.erroresAnteriores[i]);
    }
    deltaSM_errorCpy(&errorWord, &self->machine.erroresAnteriores[0]);

    /* Global error register if any: */
    //registerEventLocation(error, eventBufferNode);
}

void deltaSM_errorCpy(EventWord *errorOrigin, EventWord *errorDestination){
    errorDestination->eventLocation.hardware        = errorOrigin->eventLocation.hardware;
    errorDestination->eventLocation.subsystem       = errorOrigin->eventLocation.subsystem;
    errorDestination->eventLocation.subsubsystem    = errorOrigin->eventLocation.subsubsystem;
    errorDestination->eventLocation.severity        = errorOrigin->eventLocation.severity;
    errorDestination->eventLocation.eventCode       = errorOrigin->eventLocation.eventCode;
    errorDestination->uniqueIdentifier              = 0;//errorOrigin->uniqueIdentifier;

    errorDestination->epochTime                     = errorOrigin->epochTime;
}

/** Return the errors of the converter in last minute
 */
int deltaSM_getErrorsInLastMinutes (DeltaModule *self, const unsigned int minutes){
    int retval = 0;
    int i;

    for(i = 0; i < 9; i++) {
       if ((self->machine.erroresAnteriores[i].epochTime + (60*minutes)) > systemEpoch) retval++;
       else return retval;
    }
    return retval;
}



