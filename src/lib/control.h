/**********************************************
* SmartCar project
*-------------------------------------------
* Members:
*  Patrice Gagné
*  Francois Pomerleau
* ----------------------------------------
* Description :
* - Contain useful function for the motion-
* 	controller codels.
* ----------------------------------------
*********************************************/

#ifndef SMART_CONTROL_H
#define SMART_CONTROL_H

#include "smart.h"

// Fuzzy output borders
#define FUZZY_OUT_MIN 1
#define FUZZY_OUT_MAX 1

// Gas pedal control definition
#define GAS_PEDAL_MIN_VALUE 0
#define GAS_PEDAL_MAX_VALUE 50

#define GAIN_REAL_ACC (0.7)
#define GAIN_PREDIC_ACC (1- GAIN_REAL_ACC)

#define GAS_PEDAL_MAX_DELTA 50

// Brake control definition
#define BRAKE_PEDAL_MAX_DELTA 100

/**
 * Integrate discretely the value by a delta value and a delay.
 * @author Patrice Gagne & Francois Pomerleau
 * @date 2006-01-17
 * @param value First value of integration.
 * @param delta Delta to add to the first value. [v(t-2) - v(t)]
 * @param delay Time(ms) between the first and last value that created the delta. [(t-(t-2)]
 * @return Return the value integrated.
 * @note The integration is done over 3 values.
 * @todo The integration should be done each 10 ms and filtered over 3 values.
*/
double discrete_integrate(double value, const double delta, const double delay);


/**
 * Derivate discretely the DeltaValue by a Delay.
 * @author Patrice Gagne & Francois Pomerleau
 * @date 2006-01-17
 * @param deltaValue The difference of those two values. [v(t) - v(t-delay)]
 * @param delay Time(ms) between the first and last value that created the deltaValue. [(t-(t-delay)]
 * @return Return the derivative value.
*/
double discrete_derivative(double deltaValue, const float delay);


/**
 * PID controller function.
 * Mathematical function that looks like :
 * signal_output = P * signal_err + I * integral(signal_error) + D * derivative(signal_error)
 * @author Patrice Gagne & Francois Pomerleau
 * @date 2006-01-17
 * @param input Eror signal of the input (Command - Read value)
 * @param integrate Error signal of the input integrated
 * @param derivative Error signal of the input derivated
 * @param kp Constant multiplier of the input function (P)
 * @param ki Constant multiplier of the integral member (I)
 * @param kd Constant multiplier of the derivative member (D)
 * @return Control value based on these inputs.
 * @note Can be use as a P, Pi, PD by putting '0' in the unused field.
*/
double pid_ctrl(double input, double integrate, double derivative, double kp, double ki, double kd);


/**
 * Saturate the value between two points.
 * @author Patrice Gagne & Francois Pomerleau
 * @date 2006-01-17
 * @param value Value to saturate.
 * @param min Minimum value possible.
 * @param max Maximum value possible.
 * @return Return the value saturated.
*/
double saturation (double value, double min, double max);


/**
 * Shift up values in table.
 * @author Patrice Gagne & Francois Pomerleau
 * @date 2006-01-17
 * @param table[] Table where are those values.
 * @param newValue Value to add in table.
 * @param size Size of the table.
 * @return nothing.
 * @note The table is updated and the values are shifted inside of it.
 * @note First value entered is lost after the shifting.
*/
void valueShiftinTable(double table[], double newValue,int size);


/**
 * Predict the acceleration based on the acceleration pedal by using
 * a sigmoid fonction to do it. This function is custom and user
 * made based on data gathered on the car.
 * @author Patrice Gagne & Francois Pomerleau
 * @date 2006-01-17
 * @param acc_pedal Value of the acceleration pedal.
 * @param gear Current gear of the car.
 * @return Predicted acceleration of the car.
*/
double predictAcc(double acc_pedal, int gear);


void mctrl_accelerationControl(MCTRL_ACC_INPUT *mctrlAccInput, MCTRL_CONFIG*
  config, SMART_MOTION* smartMotion, double v_command, SMART_ENGINE* engine,
  double t_curr);

#endif
