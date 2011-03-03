#ifndef SMART_HANDLERS_H
#define SMART_HANDLERS_H

/*! \defgroup smartlibvcan VCAN callback functions
* \ingroup smartlibs
*/
 
/*@{*/

/*!
 *  \file handlers.h
 *
 *  \brief Call-back functions for received messages from the VehicleCAN
 * These message handlers are registered in the init branch of libcan and get called whenever a CAN message is reveived. Each handler handles a specific message ID coding the content of the packet.
 *
 * See function documentation in vCanMessageHandlers.c
 * \author Sascha Kolski  
 * \date February 2006 
 *
 */

#include "smart.h"
#include "can.h"

extern SMART_VEHICLE_STATUS smart;
extern ESX_STR esx;

void get_speed_msg_handler(int handle, const CPC_MSG_T * cpcmsg);

void get_pedal_msg_handler(int handle, const CPC_MSG_T * cpcmsg);

void get_yaw_msg_handler(int handle, const CPC_MSG_T * cpcmsg);

void get_steering_msg_handler(int handle, const CPC_MSG_T * cpcmsg);

void get_wheel_speeds_msg_handler(int handle, const CPC_MSG_T *cpcmsg);

/*@}*/
#endif
