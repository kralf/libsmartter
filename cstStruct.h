#ifndef CSTSTRUCT_H
#define CSTSTRUCT_H

#include <elrob/Etypes.h>

/*! \file cstStruct.h
    \author Sascha Kolski
    \brief Data Structures for the CST (Analog Output module) that commands the electric power steering and e-gas unit.
    
    Details.
*/
/*! \brief Struct holding P, I and D for PID Controller
 */ 
typedef struct SMART_PID_STR
{
  double p;
  double i;
  double d;
} SMART_PID_STR;

/*! \brief Debug struct for the CST (Analog Output module) that commands the electric power steering and e-gas unit
 */ 
typedef struct SMART_CST_DBG
{
  double SteeringVoltage; ///< Output voltage to the Steering System
  EBOOL dbg_steer_on_request_mode; ///< steering angle set directly using request  
  EBOOL dbg_gas_on_request_mode;
}SMART_CST_DBG;

/*! \brief Status struct for the CST (Analog Output module) that commands the electric power steering and e-gas unit
 */ 
typedef struct SMART_CST_STR
{
  EBOOL gas_from_poster_mode; ///< for autonomous driving requested pedal value read from MotionCtrl  
  EBOOL steer_from_poster_mode; ///< for autonomous driving requested steering angle read from MotionCtrl 
  EBOOL steer_on_request_mode;///< allows to manually set a steering angle (with active PID control)
  EBOOL gas_on_request_mode;///< allows to manually set a velocity (with active Fuzzy control)
  double desired_steering_angle;///< allows to manually set a steering angle (with active PID control)
  EBOOL simulate_speed_for_steering; ///<switches on/off the send of a faked velocity signal to the car
  double pedal; ///< pedal value in percent (command)
  double steering_angle; ///< steering angle on the wheels (command)
  SMART_PID_STR pid; ///< The PID controller for the steering system
  SMART_CST_DBG dbg;
} SMART_CST_STR;
#endif
