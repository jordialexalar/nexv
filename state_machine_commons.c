/*
 * state_machine_commons.c
 *
 *  Created on: 4 mar. 2019
 *      Author: josepmaria.fernandez
 */

#include "state_machine_commons.h"
//#include "state_machine_master.h"
#include "main.h"
#include "EventList.h"

GlobalNetworkParams networkParams;
SocketParams voidSocket;
NodeParams voidNode;


NodeParams *findNodeById(const unsigned int d){
    NodeParams *node;

    if (networkParams.node1.nodeNumber == d){
        node = &networkParams.node1;
    }
    else if (networkParams.node2.nodeNumber == d){
        node = &networkParams.node2;
    }
    else{
        node = &voidNode;
    }
    return node;
}
