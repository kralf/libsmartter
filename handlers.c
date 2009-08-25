/*! \defgroup smartlibvcan VCAN callback functions
* \ingroup smartlibs
*/
 
/*@{*/

#include <math.h>

#include <elrob/Emacros.h>
#include <elrob/Edebug.h>

#include "smartMacros.h"
#include "smartSmart.h"
#include "cst.h"
#include "handlers.h"

/*!
 *  \file vCanMessageHandlers.h
 *
 *  \brief Call-back functions for received messages from the VehicleCAN
 * These message handlers are registered in the init branch of libcan and get called whenever a CAN message is reveived. Each handler handles a specific message ID coding the content of the packet.
 *
 * \author Sascha Kolski  
 * \date February 2006 
 *
 */

SMART_VEHICLE_STATUS smart;
ESX_STR esx;

unsigned char steer_high,steer_low;
int steer;


/***************************************************************************
                               MESSAGE HANDLERS
***************************************************************************/

/*! This handler reads the speed message from the CAN Bus

Message definition:
- ID 0x90
- Byte 5 v_REF_low
- Byte 6 v_REF_high
- Range 0 - 250km/h
- Normalisation 0 -4000
- Quantisation 0.0625 km/h
- update 10ms

Output:
- smart.motion.v_curr - vehicle speed in m/s

\param handle handle to the can can bus to read from
\param cpcmsg The message

*/
void get_speed_msg_handler(int handle, const CPC_MSG_T * cpcmsg)
{
  if (cpcmsg->msg.canmsg.id == 0x90)
    {
      // EDBG("got speed message\n");
      smart.status.abs_led_on = ((cpcmsg->msg.canmsg.msg[0])&0x01);
      smart.status.abs_active = ((cpcmsg->msg.canmsg.msg[0])&0x02);
      smart.status.esp_led_on = ((cpcmsg->msg.canmsg.msg[0])&0x40);
      smart.status.esp_led_blink = ((cpcmsg->msg.canmsg.msg[0])&0x80);
      smart.wheelspeed.front_left_not_plausible = ((cpcmsg->msg.canmsg.msg[4])&0x01);
      smart.wheelspeed.front_right_not_plausible = ((cpcmsg->msg.canmsg.msg[4])&0x02);
      smart.wheelspeed.rear_left_not_plausible = ((cpcmsg->msg.canmsg.msg[4])&0x04);
      smart.wheelspeed.rear_right_not_plausible = ((cpcmsg->msg.canmsg.msg[4])&0x10);
      smart.status.v_curr = KMH2MS((double) 0.0625*((cpcmsg->msg.canmsg.msg[6]<<8)+cpcmsg->msg.canmsg.msg[5]));
    }
  if (cpcmsg->msg.canmsg.id == 0x88)
    {
      esx.vehicle_state = cpcmsg->msg.canmsg.msg[0];
      esx.max_brake_stroke = cpcmsg->msg.canmsg.msg[1];
      esx.brake_init_started = cpcmsg->msg.canmsg.msg[2];
      esx.got_first_max_brake = cpcmsg->msg.canmsg.msg[3];
      esx.request_pause = cpcmsg->msg.canmsg.msg[4];
      esx.request_stop = cpcmsg->msg.canmsg.msg[5];
      esx.control_lateral = cpcmsg->msg.canmsg.msg[6];
      esx.control_longitudinal = cpcmsg->msg.canmsg.msg[7];
 }

}

/*! This handler reads the speed message from the CAN Bus

Message definition:
- ID 0x310
- Byte 1 drag torque
- Byte 2 indicated torque
- Byte 3 maximum torque
- Byte 4 minimum torque
- Byte 5 desired torque
- Byte 6 Gas Pedal
- Byte 7 engine status
- Range 0 - 250
- Quantisation 1 (1/2.5 for pedal)
- update 10ms

Output:
- smart.motion.v_curr - vehicle speed in m/s

\param handle handle to the can can bus to read from
\param cpcmsg The message

*/
void get_pedal_msg_handler(int handle, const CPC_MSG_T * cpcmsg)
{
  if (cpcmsg->msg.canmsg.id==0x310)
    {
      //EDBG("got pedal message\n");
      smart.engine.drag_torque= (double) cpcmsg->msg.canmsg.msg[0];
      smart.engine.indicated_torque=(double) cpcmsg->msg.canmsg.msg[1];
      smart.engine.max_torque=(double) cpcmsg->msg.canmsg.msg[2];
      smart.engine.min_torque=(double) cpcmsg->msg.canmsg.msg[3];
      smart.engine.desired_torque=(double) cpcmsg->msg.canmsg.msg[4];
      smart.status.pedal=(double) (cpcmsg->msg.canmsg.msg[5]/2.5);
      smart.engine.pedal=(double) (cpcmsg->msg.canmsg.msg[5]/2.5);
      smart.engine.status=cpcmsg->msg.canmsg.msg[6];
      if (cpcmsg->msg.canmsg.msg[6]&(0x04))
	  smart.status.driving_direction=SMART_FORWARDS;
      if (cpcmsg->msg.canmsg.msg[6]&(0x10))
	  smart.status.driving_direction=SMART_BACKWARDS;
      if (cpcmsg->msg.canmsg.msg[6]&(0x20))
	  smart.status.driving_direction=SMART_UNKNOWN;
      if (cpcmsg->msg.canmsg.msg[6]&(0x40)) {
	  smart.status.brake_light_on = ETRUE;
      }
      else {
	  smart.status.brake_light_on = EFALSE;
      }  
    }

  if (cpcmsg->msg.canmsg.id==0x300)
    {
      smart.engine.rpm = (double) (cpcmsg->msg.canmsg.msg[1]<<8)+cpcmsg->msg.canmsg.msg[2];
      smart.engine.gear_byte = cpcmsg->msg.canmsg.msg[3];
      smart.engine.actual_gear = smart.engine.gear_byte & 0x0F;
      smart.engine.target_gear = smart.engine.gear_byte >> 4;	
    }
}

/*! This handler reads the steering angle message from the CAN Bus

Message definition:
- ID 0xC2
- Byte 0 steering_angle_low
- Byte 1 steering_angle_high
- Range -1440 to +1440 // car delivers only -720 to +720 degree  
- Normalisation 0 - 65535
- Quantisation 0.04375 degree
- update 20ms

Output:
- smart.motion.phi_curr - Steering wheel angle on the wheel end in radiants

\param handle handle to the can can bus to read from
\param cpcmsg The message

*/
void get_steering_msg_handler(int handle, const CPC_MSG_T * cpcmsg){ 
  if (cpcmsg->msg.canmsg.id == 2+12*16)
    {
      steer_high=cpcmsg->msg.canmsg.msg[1];
      steer_low=cpcmsg->msg.canmsg.msg[0];
      steer=0;
      if (steer_high<128)//bit one in high bite codes sign
	{
	  /*turn left: positive*/ 
	  steer= (steer_high<<8)+steer_low;
	}
      else
	{
	  /*turn right*/ 
	  steer=((128-steer_high)<<8)+steer_low;
	}
      smart.status.phi_curr = DEG2RAD((double) 0.04375*steer ) / STEERING_FACTOR;
      //EDBG("Got steering of %f\n",smart.status.phi_curr);
    }
}

/* This handler reads the wheel speeds from the CAN Bus

Message definition:
- ID 0x80
- Byte 0 wheel_speed_front_right_high
- Byte 1 wheel_speed_front_right_low
- Byte 2 wheel_speed_front_left_high
- Byte 3 wheel_speed_front_left_low
- Byte 4 wheel_speed_rear_right_high
- Byte 5 wheel_speed_rear_right_low
- Byte 6 wheel_speed_rear_right_high
- Byte 7 wheel_speed_rear_right_low
- Normalisation 0 - 65535
- Quantisation 0.5 rpm
- update 20ms

Output:
- smart.wheelspeed.front_right - wheel speed in rounds per second
- smart.wheelspeed.front_left - wheel speed in rounds per second
- smart.wheelspeed.rear_right - wheel speed in rounds per second
- smart.wheelspeed.rear_left - wheel speed in rounds per second

\param handle handle to the can can bus to read from
\param cpcmsg The message

*/
void get_wheel_speeds_msg_handler(int handle, const CPC_MSG_T *cpcmsg)
{
  if (cpcmsg->msg.canmsg.id == 0x80)
    {
      smart.wheelspeed.front_right = 
	(double) (((cpcmsg->msg.canmsg.msg[0]<<8) 
		   + cpcmsg->msg.canmsg.msg[1])/(2.0*60.0));
      smart.wheelspeed.front_left  = 
	(double) (((cpcmsg->msg.canmsg.msg[2]<<8) 
		   + cpcmsg->msg.canmsg.msg[3])/(2.0*60.0));
      smart.wheelspeed.rear_right = 
	(double) (((cpcmsg->msg.canmsg.msg[4]<<8) + 
		   cpcmsg->msg.canmsg.msg[5])/(2.0*60.0));
      smart.wheelspeed.rear_left = 
	(double) (((cpcmsg->msg.canmsg.msg[6]<<8) 
		   + cpcmsg->msg.canmsg.msg[7])/(2.0*60.0));
    }
}


void get_esx_mesage_handler(int handle, const CPC_MSG_T *cpcmsg)
{
}

/*@}*/
