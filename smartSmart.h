#ifndef SMART_SMART_H
#define SMART_SMART_H

/*! \defgroup smart Vehicle abstraction layer

The module smart contains a complete abstraction layer of the vehicle. It provides on the one hand the motion state of the car in terms of translational and rotational velocities as well as a command interface to the steering, gas and braking system of the car. The module contains two major components: The GenoM module on the one hand and a set of libraries dealing with the specific actuators of the car. 

*/

/*! \defgroup smartlibs Library functions 

 * \ingroup smart
 */
 
 /*! \defgroup smartgenom Genom Tasks and Requests
 * \ingroup smart
 */	

/*! \file smartSmart.h
    \author Sascha Kolski
    \brief Constants for communicating with the smart
    \ingroup smart 
    Details.
*/

#define HAVE_ESX

#ifdef HAVE_ESX

/*! \brief Bus ID for the internal vehicle CAN bus (VCAN)*/
#define SMART_VCAN_BUSID 0

/*! \brief Bus ID for the private  CAN bus to communicate with the actuators (ICAN)*/
#define SMART_ICAN_BUSID 0

#else

/*! \brief Bus ID for the internal vehicle CAN bus (VCAN)*/
#define SMART_VCAN_BUSID 0

/*! \brief Bus ID for the private  CAN bus to communicate with the actuators (ICAN)*/
#define SMART_ICAN_BUSID 1

#endif

/*! \brief Baud Rate for the VCAN in Kbaud*/
#define SMART_VCAN_BAUD_RATE 500

/*! \brief Baud Rate for the ICAN in Kbaud*/
#define SMART_ICAN_BAUD_RATE 250

/*! \brief maximum voltage for CST analog output unit*/
#define V_MAX 10

#define STEERING_FACTOR 28.55 ///<Factor between steering wheel angle and wheel angle

#endif
