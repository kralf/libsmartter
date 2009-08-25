#ifndef SMART_CST_H
#define SMART_CST_H

#include <cpc-usb/cpclib.h>

/*! \defgroup smartlibcst Library of functions for analog output module
    \ingroup  smartlibs
    */ 
/*@{*/

/*! \file libcst.h
 *  \author Sascha Kolski
 *  \brief Interface to the analog output device controlling the steering and e-gas of the vehicle.
 *  \ingroup smartlibs             
 *  The CST is connected on the ICAN, the CAN bus we use to command the actuators attached to the vehicle. It reads CAN messages defining voltages for the different analog outputs and sets these voltages accordingly. 
   
*/

/*! The limits of the steering control output */
#define STEERING_CONTROL_MAX_OUTPUT_VOLTAGE (4.4)

/*! The limits of the steering control output */
#define STEERING_CONTROL_MIN_OUTPUT_VOLTAGE (0.6)

/*! Physical limits for the electronic power steering 
   CHANGING THIS CAN CAUSE DEMAGE OF THE POWER STEERING UNIT
*/
#define STEERING_PHYSICAL_MAX_LIMIT (4.5)

/*! Physical limits for the electronic power steering 
   CHANGING THIS CAN CAUSE DEMAGE OF THE POWER STEERING UNIT
*/
#define STEERING_PHYSICAL_MIN_LIMIT (0.5) 


/*!
 *
 * \brief sending a CAN message looking like the one send by the ABS of the car
 *
 * Allows to fake the electronic power steering unit by sending it the message normaly sent by the ABS of the car but with the minimal velocity. This makes shure that the enhancement factor of the electronic power steering is allways sufficient to actually steer the car.
 *
 * 
 */
void cst_send_speed_msg(int busId);

/*!
 *
 * \brief Setting the pedal value for egas
 *
 * \param p How much the gas pedal is pressed (in percent) 
 * 
 */
void cstSetPedalValue(int busId, double p);

/*!
 *
 * \brief Setting the pedal value for egas
 *
 * \param voltage Voltage to be apllied to the Power Steering 
 * 
 */
void cstSetSteeringVoltage(int busId, double voltage);

/*!
 *
 * \brief Initializing the CST analog output unit
 *
 * Takes care of the configuration of the device to access the two analog
 * output channels the one or the other (cannel one and two) as well as the 
 * two in one message (channel 0). 
 * \warning configuration of the two in one message does not work
 */ 
void cstInit(int busId);

/*!
 *
 * \brief Cycle of the PID control for the steering unit
 *
 * This PID controls the steering position of the car by comparing the actual steering wheel position measured by the car's sttering wheel angle sensor to a desired steering wheel angle given by the user or read from a higher level (commanding) module. 
 *
 * \param P P
 * \param I I
 * \param D D
 * \param D t the sampling time
 * \param target the desired steering angle (in radients at the wheel)
 * \param current_value the actual steering angle (in radients at the wheel)
 * \param old_steering_voltage The voltage apllied in the last cycle
 * \param the voltage to apply now
 */
void cstSteeringPID(double P, double I, double D, double t, double target, double current_value, double old_steering_voltage, double *output);
/*@}*/
#endif
