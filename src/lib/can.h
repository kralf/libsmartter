#ifndef SMART_CAN_H
#define SMART_CAN_H

#include <libcpc/cpclib.h>

#include <libelrob/Etypes.h>

/*! \defgroup smartlibcan Library for USB CAN access
* \ingroup smartlibs
*/
 
/*@{*/
/*! \brief Maximum number of supported CPC-USB CAN devices (CAN busses) */
#define LIBCAN_MAX_CAN 4

/*! \brief CAN operation timeout in [s] */
#define LIBCAN_TIMEOUT 0.05

/*! \brief Recording wich busses are allready initialized */
int can_active[LIBCAN_MAX_CAN];

/*!
 *
 * \brief Init CAN bus access
 *
 * Initializes a given CAN bus. 
 *
 * \param busId CAN bus Number
 * \param bitrate Bitrate for the communication in Kbaud (supprted bitrates are 100, 250, 500 and 1000)
 * \param *can_device Device to connect to (eg: /dev/usb/cpc_usb0)
 * 
 */
int canHWInit(int busId,int bitrate, char *device);

int canHWCleanup(int busId);

/*!
 *
 * \brief sends a standard CAN message
 *
 * \param busId CAN bus to send the message on
 * \param can_id Identifier of the CAN message
 * \param *msg Pointer to the 8byte content of the CAN message
 * 
 */
int my_send_can_message(int busId, int can_id, char *msg);
void read_SDO_msg_handler(int handle, const CPC_MSG_T * cpcmsg);

/*!
 *
 * \brief reads a CAN message
 *
 * This function is waiting for a CAN message in the given bus. If a message arrives all registered message handlers 
 * are called one after the other. If no message is available after a time defined in SELECT, the function returns 
 * without calling any message handler.  
 * \todo update comment
 * \param busId CAN bus to read from
 * 
 * 
 */
EBOOL read_can_message(int busId);

/*!
 *
 * \brief sends a variable length CAN message
 *
 * \param busId CAN bus to send the message on
 * \param can_id Identifier of the CAN message
 * \param length Length of the CAN message 
 * \param *msg Pointer to the content of the CAN message
 * 
 * 
 */
void my_send_can_message_var_length(int busId, int can_id, int length ,char *msg);
/*@}*/
#endif
