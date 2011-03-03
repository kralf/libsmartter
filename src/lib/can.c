
/*!
 *  \file libcan.c
 *
 *  \brief low level functions for CPC-USB can device.
 *
 *The CPC-USB CAN device can read and write can messages using a cusom API delivered by EMS Wuensche, the producer of this hardware unit. This library acts as a wrapper to that API to allow simple usage of that unit. It delivers functions for initialization, reading and writing of CAN messages.
 *
 * \author Sascha Kolski  
 * \date February 2006 
 *
 */

#include <sys/types.h>
#include <sys/dir.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/file.h>
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>
#include <termios.h>

#include <libelrob/Edebug.h>

#include "smart.h"
#include "can.h"
#include "lss.h"
#include "handlers.h"


/*! Macros for watching on a CAN channel. */
#define SELECT    FD_ZERO(&readfds);		  \
                  FD_SET(0, &readfds);            \
                  FD_SET(cpcfd, &readfds);	      \
                  tv.tv_sec  = 0;	                \
                  tv.tv_usec = 500000;	/*500000*/ /*2000*/      \
                  nfds = select(cpcfd+1, &readfds, NULL, NULL, &tv);

/*! Macros for watching on write on a CAN channel. */
#define SELECT_WR FD_ZERO(&writefds);                                 \
                  /*FD_SET(0, &readfds);*/         \
                  FD_SET(0, &readfds);                                 \
                  FD_SET(0, &writefds);                                \
                  FD_SET(cpcfd, &writefds);                           \
                  tv.tv_sec  = 0;                                     \
                  tv.tv_usec = 500000;                                \
                  nfds = select(cpcfd+1, NULL, &writefds, NULL, &tv);

static unsigned char       btr0,btr1;
static int                 handle;
static int                 handle_array[LIBCAN_MAX_CAN]; 
static char                interface[32];  
static int                 zchn, nfds;
static int                 cpcfd_array[LIBCAN_MAX_CAN];
static int                 cpcfd;
static CPC_INIT_PARAMS_T * CPCInitParamsPtr;
static fd_set              readfds, writefds;
static struct timeval      tv;

/*!
 *
 * \brief sends a standard CAN message
 *
 * \param busId CAN bus to send the message on
 * \param can_id Identifier of the CAN message
 * \param *msg Pointer to the 8byte content of the CAN message
 * 
 */

int my_send_can_message(int busId, int can_id, char *msg) {
  CPC_CAN_MSG_T cmsg = {0x00L, 0, {0, 0, 0, 0, 0, 0, 0, 0}};
  struct timeval time;
  fd_set set;
  int i, error;

  cpcfd = cpcfd_array[busId];
  handle = handle_array[busId];

  cmsg.id = can_id;
  cmsg.length = 8;
  memcpy(cmsg.msg, msg, 8);

  time.tv_sec = 0;
  time.tv_usec = LIBCAN_TIMEOUT*1e6;

  FD_ZERO(&set);
  FD_SET(cpcfd, &set);

  error = select(cpcfd+1, NULL, &set, NULL, &time);
  if (error == 0)
    return -1;

  while ((error = CPC_SendMsg(handle, 0, &cmsg)) == CPC_ERR_CAN_NO_TRANSMIT_BUF)
    usleep(10);

  return error;
}

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
void my_send_can_message_var_length(int busId, int can_id, int length, char *msg)
 {
   static CPC_CAN_MSG_T cmsg = {0x00L,0,{0,0,0,0,0,0,0,0}};
   int i;
   
   cpcfd = cpcfd_array[busId];
   handle = handle_array[busId];

   cmsg.id=can_id;
   cmsg.length=length;

    for(i=0;i<cmsg.length;i++){
     cmsg.msg[i] = (unsigned char)msg[i];
    } 
    
    SELECT_WR
      if(FD_ISSET(cpcfd, &writefds)){
	CPC_SendMsg(handle, 0, &cmsg);
      }
 }

/*!
 *
 * \brief reads a CAN message
 *
 * This function is waiting for a CAN message in the given bus. If a message arrives all registered message handlers 
 * are called one after the other. If no message is available after a time defined in SELECT, the function returns 
 * without calling any message handler. 
 *
 * \param busId CAN bus to read from
 * 
 * 
 */
EBOOL read_can_message(int busId)
{
  cpcfd = cpcfd_array[busId];
  handle = handle_array[busId];  /* task for reading can */
  SELECT 
    
  //EDBG("nfds= %d\n",nfds);

  if(nfds>0)
      {
	// check, if messages have been received
	if(FD_ISSET(cpcfd, &readfds))
	  {
	    do{
	      /*EPRINT(".");*/
	    }
	    while(CPC_Handle(handle));
	    //EDBG(":\n");
	  }
	else
	  {
	    //EDBG("NO READING POSSIBLE from can bus %d\n", busId);
	    return EFALSE;
	  }
      }    
    else
      {
	EDBG("nothing to read...from can bus %d\n", busId);
	if(nfds == -1)
	  perror(interface);

	/*if(nfds == 0)
	  EDBG("Timeout");*/

	return EFALSE;
      }

  return ETRUE;
}

int canHWCleanup(int busId){
	return CPC_CloseChannel(handle_array[busId]);
}


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
int canHWInit(int busId, int bitrate, char *can_device)
{
  //EDBG_DISABLE();
  EDBG("Starting CANHWInit...");
  EDBG("configuring port %d with %d baud",busId,bitrate);
  switch (bitrate){
  case 1000:
    /* Bitrate 1 Mbit */
    btr0=0x00;
    btr1=0x14;
    break;
  case 250:
    /* Bitrate 250 Kbit */
    btr0=0x01;
    btr1=0x1c;
    break;
  case 500:
    /* Baudrate 500 used for smart */

    btr0=0x00;
    btr1=0x1c;
    break;
  case 100:
    btr0=0x04;
    btr1=0x1c;
    break;
  }

  strcpy(interface, can_device);

  /*Open the CAN*/
  if((handle_array[busId] = CPC_OpenChannel(interface)) < 0)
    {
      EDBG("ERROR: %s\n", CPC_DecodeErrorMsg(handle_array[busId]));
      return -1;
    }
  EDBG("%s is CAN interface -> handle %d\n", interface, handle_array[busId]);

  /* ############################# Init Parameters ######################*/
  
  /*Define Handlers*/
  CPC_AddHandler(handle_array[busId], get_speed_msg_handler);
  CPC_AddHandler(handle_array[busId], get_steering_msg_handler);
  CPC_AddHandler(handle_array[busId], get_pedal_msg_handler);
  CPC_AddHandler(handle_array[busId], get_wheel_speeds_msg_handler);
  CPC_AddHandler(handle_array[busId], lss_get_msg_handler);

  /* This sets up the parameters used to initialize the CAN controller */
  EDBG("Initializing CAN-Controller ... ");
  
  CPCInitParamsPtr = CPC_GetInitParamsPtr(handle_array[busId]);
  CPCInitParamsPtr->canparams.cc_type                      = SJA1000;
  CPCInitParamsPtr->canparams.cc_params.sja1000.btr0       = btr0;
  CPCInitParamsPtr->canparams.cc_params.sja1000.btr1       = btr1;
  CPCInitParamsPtr->canparams.cc_params.sja1000.outp_contr = 0xda;
  CPCInitParamsPtr->canparams.cc_params.sja1000.acc_code0  = 0xff;
  CPCInitParamsPtr->canparams.cc_params.sja1000.acc_code1  = 0xff;
  CPCInitParamsPtr->canparams.cc_params.sja1000.acc_code2  = 0xff;
  CPCInitParamsPtr->canparams.cc_params.sja1000.acc_code3  = 0xff;
  CPCInitParamsPtr->canparams.cc_params.sja1000.acc_mask0  = 0xff;
  CPCInitParamsPtr->canparams.cc_params.sja1000.acc_mask1  = 0xff;
  CPCInitParamsPtr->canparams.cc_params.sja1000.acc_mask2  = 0xff;
  CPCInitParamsPtr->canparams.cc_params.sja1000.acc_mask3  = 0xff;
  CPCInitParamsPtr->canparams.cc_params.sja1000.mode       = 0;

  unsigned char confirm;
  confirm = 0;
  int reply;

  reply = CPC_CANInit(handle_array[busId], confirm);
  EDBG(" init said: %d,%d Done!\n\n",reply,confirm);

  cpcfd_array[busId] = CPC_GetFdByHandle(handle);
  
  EDBG("cpcfd= %d\n",cpcfd_array[busId]);

  EDBG("Switch ON transimssion of CAN messages from CPC to PC\n");

  /* switch on transmission of CAN messages from CPC to PC */
  CPC_Control(handle_array[busId], CONTR_CAN_Message | CONTR_CONT_ON);
  EDBG("InitHW finished...\n");
  can_active[busId]=1;

  return reply;
}
