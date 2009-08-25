/* Library for Linear Servo System */
/* Sascha Kolski, ASL, EPFL */

#include <stdio.h>

#include <elrob/Edebug.h>

#include "lss.h"
#include "can.h"

LSS_STR lss;

void lss_init(int bus_id)
{
  EDBG("LSS initiation");
  char msg[8];
  int can_id = 0x00;
  msg[0]=0x01;
  msg[1]=0x00;
  my_send_can_message_var_length(bus_id, can_id,2, msg);

  // Ensure proper motor configuration
  lss_save_position_limits(bus_id, LSS_MIN_POSITION, LSS_MAX_POSITION);

  // Update motor properties
  lss_get_actual_position(bus_id);
  lss_get_commanded_position(bus_id);
  lss_get_max_drive_current(bus_id);

  EDBG("LSS inititation finished");

}

int lss_save_position_limits(int bus_id, double min_position, double max_position)
{
  // max position = 75mm and min position = 0mm
  if (min_position >= LSS_MIN_POSITION && max_position <= LSS_MAX_POSITION){
    if (    
      // Change pointer to Software stroke limit positive end (LIMM) adress
      !lss_send_request(bus_id, LSS_NO_ATTRIBUTE, LSS_DEST_ADDR_DATA_WRITE, (int)0x00007802, ETRUE) &&
      !lss_send_request(bus_id, LSS_NO_ATTRIBUTE, LSS_DEST_ADDR_DATA_WRITE, (int)0x00007802, EFALSE) &&
  
      // Write LIMM
      !lss_send_request(bus_id, LSS_NO_ATTRIBUTE, LSS_WRITE_DATA_TO_MEM, LSS_MM_TO_INC(max_position), ETRUE) &&
      !lss_send_request(bus_id, LSS_NO_ATTRIBUTE, LSS_WRITE_DATA_TO_MEM, LSS_MM_TO_INC(max_position), EFALSE) &&
  
      // Change pointer to Software stroke limit negative end (LIML)
      !lss_send_request(bus_id, LSS_NO_ATTRIBUTE, LSS_DEST_ADDR_DATA_WRITE, (int)0x00007803, ETRUE) &&
      !lss_send_request(bus_id, LSS_NO_ATTRIBUTE, LSS_DEST_ADDR_DATA_WRITE, (int)0x00007803, EFALSE) &&
  
      // Write LIML
      !lss_send_request(bus_id, LSS_NO_ATTRIBUTE, LSS_DEST_ADDR_DATA_WRITE, LSS_MM_TO_INC(min_position), ETRUE) &&
      !lss_send_request(bus_id, LSS_NO_ATTRIBUTE, LSS_DEST_ADDR_DATA_WRITE, LSS_MM_TO_INC(min_position), EFALSE)
    )
    return 0;
  }
  else {
    EDBG("Error: out of limit (0, 75), no change apply");
  }

  return -1;
}

int lss_send_request(int bus_id, int get_attribute_id, int set_attribute_id, int set_value, EBOOL load_data) 
{
  int result;
  char msg[8];

  msg[0]= LSS_DATA_HEADER + load_data;
  msg[1]= get_attribute_id;
  msg[2]= LSS_COMMAND_MSG_TYPE;
  msg[3]= set_attribute_id;
  msg[4]= (char)(set_value & 0x000000FF);
  msg[5]= (char)((set_value & 0x0000FF00) >> 8);
  msg[6]= (char)((set_value & 0x00FF0000) >> 16);
  msg[7]= (char)((set_value & 0xFF000000) >> 24);

  if ((result = my_send_can_message(bus_id, LSS_CAN_ID, msg)))
    usleep(10000);

  return result;
}

int lss_test_stroke_max(int bus_id)
{
  if (
    !lss_send_request(bus_id, LSS_NO_ATTRIBUTE, LSS_EXEC_HOMING, LSS_MAX_HOMING, ETRUE) &&
    !lss_send_request(bus_id, LSS_NO_ATTRIBUTE, LSS_EXEC_HOMING, LSS_MAX_HOMING, EFALSE)
  )
    return 0;
  else
    return -1;

}

int lss_test_stroke_min(int bus_id)
{
  if (
    !lss_send_request(bus_id, LSS_NO_ATTRIBUTE, LSS_EXEC_HOMING, LSS_MIN_HOMING, ETRUE) &&
    !lss_send_request(bus_id, LSS_NO_ATTRIBUTE, LSS_EXEC_HOMING, LSS_MIN_HOMING, EFALSE)
  )
    return 0;
  else
    return -1;
}

int lss_set_position(int bus_id, double position)
{
  position = (position < LSS_MIN_POSITION) ? LSS_MIN_POSITION : position;
  position = (position > LSS_MAX_POSITION) ? LSS_MAX_POSITION : position;

  if (
    !lss_send_request(bus_id, LSS_NO_ATTRIBUTE, LSS_TARGET_POSITION, LSS_MM_TO_INC(position), ETRUE) &&
    !lss_send_request(bus_id, LSS_NO_ATTRIBUTE, LSS_TARGET_POSITION, LSS_MM_TO_INC(position), EFALSE)
  )
    return 0;
  else
    return -1;
}

int lss_get_commanded_position(int bus_id) {
  if (
    !lss_send_request(bus_id, LSS_TARGET_POSITION, LSS_NO_ATTRIBUTE, 0, ETRUE) &&
    !lss_send_request(bus_id, LSS_TARGET_POSITION, LSS_NO_ATTRIBUTE, 0, EFALSE)
  )
    return 0;
  else
    return -1;
}

int lss_get_actual_position (int bus_id){
  if (
    !lss_send_request(bus_id, LSS_ACTUAL_POSITION, LSS_NO_ATTRIBUTE, 0, ETRUE) &&
    !lss_send_request(bus_id, LSS_ACTUAL_POSITION, LSS_NO_ATTRIBUTE, 0, EFALSE)
  )
    return 0;
  else
    return -1;
}

int lss_set_max_drive_current(int bus_id, double moving, double holding){
  if (moving >= 0 && moving <= 1 && holding >= 0 && holding <= 1){
    if (
      !lss_send_request(bus_id, LSS_ACTUAL_POSITION, LSS_DRIVE_CURRENT, 
        ((int)(LSS_MAX_HOLDING_CURRENT*holding)<<16) + (int)(LSS_MAX_MOVING_CURRENT*moving), ETRUE) &&
      !lss_send_request(bus_id, LSS_ACTUAL_POSITION, LSS_DRIVE_CURRENT, 
		     ((int)(LSS_MAX_HOLDING_CURRENT*holding)<<16) + (int)(LSS_MAX_MOVING_CURRENT*moving), EFALSE)
    )
      return 0;
  }
  else {
    EDBG("\n Error: current must be between 0 and 1. No change applied");
  }
  
  return -1;
}

int lss_get_max_drive_current(int bus_id){
  if (
    !lss_send_request(bus_id, LSS_DRIVE_CURRENT, LSS_NO_ATTRIBUTE, 0, ETRUE) &&
    !lss_send_request(bus_id, LSS_DRIVE_CURRENT, LSS_NO_ATTRIBUTE, 0, EFALSE)
  )
    return 0;
  else
    return -1;
}

int lss_set_brake_hold(int bus_id){
  if (
    !lss_send_request(bus_id, LSS_NO_ATTRIBUTE, LSS_BRAKE_CONTROL, LSS_BRAKE_HOLD, ETRUE) &&
    !lss_send_request(bus_id, LSS_NO_ATTRIBUTE, LSS_BRAKE_CONTROL, LSS_BRAKE_HOLD, EFALSE)
  )
    return 0;
  else
    return -1;
}

int lss_set_brake_release(int bus_id){
  if (
    !lss_send_request(bus_id, LSS_NO_ATTRIBUTE, LSS_BRAKE_CONTROL, LSS_BRAKE_RELEASE, ETRUE) &&
    !lss_send_request(bus_id, LSS_NO_ATTRIBUTE, LSS_BRAKE_CONTROL, LSS_BRAKE_RELEASE, EFALSE)
  )
    return 0;
  else
    return -1;
}


/********************************************************/
/* Message handler for answers from LSS                 */
/********************************************************/
void lss_get_msg_handler(int handle, const CPC_MSG_T * cpcmsg)
{
  //EDBG("iCan activity detected...");
  
  if (cpcmsg->msg.canmsg.id == 0x1bf) {
    //EDBG("... it's a brake motor message");

    // Store motor state
    lss.enable_moving =        (cpcmsg->msg.canmsg.msg[0] & 0x80) >> 7;
    lss.homing_done =          (cpcmsg->msg.canmsg.msg[0] & 0x20) >> 5;
    lss.shaft_moving_up =      (cpcmsg->msg.canmsg.msg[0] & 0x10) >> 4;
    lss.general_error =        (cpcmsg->msg.canmsg.msg[0] & 0x08) >> 3;
    lss.on_target_position =   (cpcmsg->msg.canmsg.msg[0] & 0x04) >> 2;
    lss.motor_moving =         (cpcmsg->msg.canmsg.msg[0] & 0x01);
    lss.command_loaded =       (cpcmsg->msg.canmsg.msg[2] & 0x80) >> 7;
    lss.error.following =      (cpcmsg->msg.canmsg.msg[2] & 0x20) >> 5;
    lss.error.negative_limit = (cpcmsg->msg.canmsg.msg[2] & 0x10) >> 4;
    lss.error.positive_limit = (cpcmsg->msg.canmsg.msg[2] & 0x08) >> 3;
    lss.error.stroke_reverse = (cpcmsg->msg.canmsg.msg[2] & 0x04) >> 2;
    lss.error.stroke_foward =  (cpcmsg->msg.canmsg.msg[2] & 0x02) >> 1;
    lss.error.input =          (cpcmsg->msg.canmsg.msg[2] & 0x01);

    switch(cpcmsg->msg.canmsg.msg[1]){

    case LSS_ACTUAL_POSITION:
      lss.actual_position = LSS_INC_TO_MM(cpcmsg->msg.canmsg.msg[4] 
					  + (cpcmsg->msg.canmsg.msg[5]<<8) 
					  + (cpcmsg->msg.canmsg.msg[6]<<16) 
					  + (cpcmsg->msg.canmsg.msg[7]<<24));
      
      if (lss.actual_position > LSS_MAX_POSITION || lss.actual_position < LSS_MIN_POSITION){
        lss.actual_position = -1;
      }

      //EDBG("actual position answer: %f", lss.actual_position);
      break;
      
    case LSS_TARGET_POSITION:
      lss.target_position = LSS_INC_TO_MM(cpcmsg->msg.canmsg.msg[4] 
					  + (cpcmsg->msg.canmsg.msg[5]<<8) 
					  + (cpcmsg->msg.canmsg.msg[6]<<16) 
					  + (cpcmsg->msg.canmsg.msg[7]<<24));
      if (lss.target_position > LSS_MAX_POSITION || lss.target_position < LSS_MIN_POSITION){
	lss.target_position = -1;
      }
      //EDBG("target position answer: %f",lss.target_position);
      break;
    
    case LSS_DRIVE_CURRENT:
      lss.max_armature_curr_moving = ((double)cpcmsg->msg.canmsg.msg[4] / (double)LSS_MAX_MOVING_CURRENT);
      lss.max_armature_curr_holding = ((double)cpcmsg->msg.canmsg.msg[6] / (double)LSS_MAX_HOLDING_CURRENT);
      //EDBG("Armature current answer: %f -- %f", lss.max_armature_curr_moving, lss.max_armature_curr_holding);
      break;

    default:
      ;
      //EDBG("LSS message unhandled");
    }
  }  
}
  
