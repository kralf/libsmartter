#ifndef SMART_STRUCT_H
#define SMART_STRUCT_H

#include <elrob/Etypes.h>

#define WINDOW 3

/*! \file smartStruct.h
    \author Sascha Kolski
    \brief Data Structures for smart vehicle abstraction module.
    
    Details.
*/

/*! \brief Defines if we use the USB-CAN converters or work on the PCI cards of the rack
 */ 
typedef enum _SMART_CAN_COM_TYPE { SMART_CAN_COM_USB, SMART_CAN_COM_PCI} SMART_CAN_COM_TYPE;

typedef enum _SMART_DIRECTION {
  SMART_FORWARDS,
  SMART_BACKWARDS,
  SMART_UNKNOWN 
}SMART_DIRECTION;

typedef enum _MCTRL_CMD_SOURCE_TYPE {
  MCTRL_NO_SOURCE,
  MCTRL_NAVIGATION,
  MCTRL_JOYSTICK
}MCTRL_CMD_SOURCE_TYPE;

/*! \brief status of the smart engine */
typedef struct SMART_ENGINE {
  double drag_torque; ///< drag torque
  double indicated_torque; ///< indicated torque
  double max_torque; ///< maximal torque
  double min_torque; ///< minimal torque
  double desired_torque; ///< engine desired torque
  double status;///< Engine status
  double rpm;///< Engine RPM
  double pedal;///< How much the gas pedal is actually pressed
  char gear_byte;
  int actual_gear;
  int target_gear;
}SMART_ENGINE;

/*! \brief wheel speeds of the four wheels */
typedef struct SMART_WHEEL_SPEEDS {
  double front_right;
  EBOOL front_right_not_plausible;
  double front_left;
  EBOOL front_left_not_plausible;
  double rear_right;
  EBOOL rear_right_not_plausible;
  double rear_left;
  EBOOL rear_left_not_plausible;
}SMART_WHEEL_SPEEDS;

/*! \brief motion state of the car */
typedef struct SMART_MOTION {
  double pedal; ///< How much the gas pedal is actually pressed
  double phi_curr; ///< The actual steering angle at the wheel end in radients (NOT at the steering wheel)
  double v_curr; ///< The actual translational speed pf the car in m/s
  EBOOL esp_led_on; ///<  The esp warning light is on
  EBOOL esp_led_blink; ///< The esp earning light is blinking 
  EBOOL abs_led_on; ///< The ABS warning light is on
  EBOOL abs_active; ///< ABS is active, controlling the brake
  SMART_DIRECTION driving_direction; ///<driving direction (works only from a minimum veloctity)
  EBOOL brake_light_on; ///< indicates if the brake light is on saying if the brake is pressed or not
  TIMEVAL t_curr; ///< Timestamp
} SMART_MOTION;


/*! \brief overall status of the car */
typedef struct SMART_VEHICLE_STATUS {
  SMART_ENGINE engine;///< engine status (torques, rpms, etc)
  SMART_WHEEL_SPEEDS wheelspeed; ///< the speeds for the single wheels in turns/second
  SMART_MOTION status;///<  Pedal value, steering angle and translational speed of the car
  double update_rate;
}SMART_VEHICLE_STATUS;

/*! \brief holds global configuration of the smart module
 */ 
typedef struct SMART_CONFIG_STR {
  SMART_CAN_COM_TYPE canCommunicationType;///< Having USB or PCI can interface
  EFILENAME IcanPort; ///< CAN device connected to the ICAN 
  EFILENAME VcanPort; ///< CAN device connected to the VCAN
  EFILENAME logfile;
  EBOOL log;
  EBOOL have_esx; ///< Do we communicate to the car via the security ECU or not
  EBOOL use_carmen;
}SMART_CONFIG_STR;

/** 
 * Structure that stores temporarily value used 
 * in FuzzyLogic Controller 
*/
typedef struct MCTRL_ACC_INPUT {
  double velocity_raw[WINDOW];
  double velocity_filtered[WINDOW];
  double timestamp_v[WINDOW];
  double velocity_err[WINDOW];
 
  EBOOL brake_ready_to_serve;
  double brake_accurate_range;
  double brake_accurate_offset;

  double gas_pedal_cmd;
  double brake_pedal_cmd;
  
}MCTRL_ACC_INPUT;

/**
 * This structure is used to set different state 
 * of the motionController task.
*/
typedef struct MCTRL_CONFIG {

  EBOOL accept_brake_ajustment;

  EBOOL enable_gas_ctrl;
  EBOOL enable_brake_ctrl;
  EBOOL enable_steering_ctrl;

  //EBOOL noConnection;

  MCTRL_CMD_SOURCE_TYPE get_cmd_from;

  double car_min_acc;
  double car_max_acc;

} MCTRL_CONFIG;

typedef struct _SMART_COMMAND {
  double v;
  double steer;
} SMART_COMMAND;

typedef struct _ESX_STR {
  int vehicle_state;
  int max_brake_stroke;
  int brake_init_started;
  int got_first_max_brake;
  int request_pause;
  int request_stop;
  int control_lateral;
  int control_longitudinal;
} ESX_STR;

typedef struct _S_JOYSTICK {
  int x;
  int y;
  EBOOL id0;
  EBOOL id1;
  EBOOL id2;
  EBOOL id5;
  EBOOL id6;
  int count;
} S_JOYSTICK;

#endif
