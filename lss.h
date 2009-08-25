#ifndef SMART_LSS_H
#define SMART_LSS_H 

#include <cpc-usb/cpclib.h>

#include <elrob/Etypes.h>

#include "lssStruct.h"

// 
#define LSS_CAN_ID (0x23f)

// Headers [bit0,   bit1, bit2     , bit3       , bit4     , bit5       , bit6, bit7                   ] 
//         [Enable, 0   , Hard Stop, Smooth Stop, Direction, Incremental, 0   , Load Data/Start Profile]
#define LSS_DATA_HEADER (0x80)

// Command message type (fixed)
#define LSS_COMMAND_MSG_TYPE (0x3b)

// Atribute ID
#define LSS_NO_ATTRIBUTE (0x00)

#define LSS_OPERATING_MODE (0x03)
// Options for operating mode
#define LSS_POSITION_MODE ((int)0x00000000)
#define LSS_VELOCITY_MODE ((int)0x00000001)

#define LSS_TARGET_POSITION (0x06)
// Limits for position
#define LSS_MAX_POSITION (75) // in mm
#define LSS_MIN_POSITION (0) // in mm

#define LSS_TARGET_VELOCITY (0x07)
#define LSS_TARGET_ACCELERATION (0x08)
#define LSS_ACTUAL_POSITION (0x0d)         // GET only
#define LSS_ACTUAL_VELOCITY (0x0e)         // GET only

#define LSS_DRIVE_CURRENT (0x19)
// Limits for current
#define LSS_MAX_MOVING_CURRENT (0xff)
#define LSS_MAX_HOLDING_CURRENT (0xb4)

#define LSS_IN_POSITION_WIDTH (0x87)
#define LSS_DEST_ADDR_DATA_WRITE (0x88)    // SET only
#define LSS_WRITE_DATA_TO_MEM (0x89)       // SET only
#define LSS_ACTUATOR_STATUS (0x8a)         // GET only
#define LSS_DEST_ADDR_DATA_READ (0x8c)
#define LSS_READ_DATA_FROM_MEM (0x9c)      // GET only
#define LSS_SERVO_GAIN (0x8d)              // SET only

#define LSS_EXEC_HOMING (0x8e)             // SET only
// Options for homing:
#define LSS_MAX_HOMING ((int)0x0000000a)
#define LSS_MIN_HOMING ((int)0x00000009)

#define LSS_BRAKE_CONTROL (0x8f)           // SET only
// Options for brake
#define LSS_BRAKE_HOLD ((int)0x00000000)
#define LSS_BRAKE_RELEASE ((int)0x00000002)

#define LSS_SAVE_DATA_TO_EEPROM (0x90)     // SET only
#define LSS_RESET (0x92)                   // SET only
#define LSS_JOG_DRIVE (0x93)               // SET only
#define LSS_EXEC_POSITIONING (0x94)        // SET only

#define LSS_MM_TO_INC(mm) ((int)((mm)/0.00375)) // 75mm = 20000 increments (specification)
#define LSS_INC_TO_MM(inc) (((int)inc)*0.00375)

extern LSS_STR lss;

void lss_init(int bus_id);

int lss_send_request(int bus_id, int get_attribute_id, int set_attribute_id, int set_value, EBOOL load_data);

// Position in mm
int lss_save_position_limits(int bus_id, double min_position, double max_position);

int lss_test_stroke_max(int bus_id);
int lss_test_stroke_min(int bus_id);

int lss_set_position(int bus_id, double position);
int lss_get_commanded_position(int bus_id);
int lss_get_actual_position(int bus_id);

// Relative 1=max device current, 0=no current
int lss_set_max_drive_current(int bus_id, double moving, double holding);
int lss_get_max_drive_current(int bus_id);

int lss_set_brake_hold(int bus_id);
int lss_set_brake_release(int bus_id);

void lss_get_msg_handler(int handle, const CPC_MSG_T * cpcmsg);


#endif
