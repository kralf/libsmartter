/* libsct.c 
   
   code for Analog output module
   
   This module commands the E-Gas and Active Steering unit
*/

/*! \file libcst.c
 *  \author Sascha Kolski
 *  \brief Interface to the analog output device controlling the steering and e-gas of the vehicle.
 *
 *  The CST is connected on the ICAN, the CAN bus we use to command the actuators attached to the vehicle. It reads CAN messages defining voltages for the different analog outputs and sets these voltages accordingly. 
   
*/
#include <sys/time.h>
#include <time.h>
#include <math.h>

#include <stdio.h>

#include <libelrob/Edebug.h>

#include "smart.h"
#include "cst.h"
#include "can.h"

struct timeval      tv;

/* PID controller variables */
static double e, old_e;
static double h;
static double Integral;
double tmp;

/* can message */ 
char msg[8];
int canId;


/*!
 *
 * \brief Initializing the CST analog output unit
 *
 * Takes care of the configuration of the device to access the two analog
 * output channels the one or the other (channel one and two) as well as the 
 * two in one message (channel 0). 
 * \warning configuration of the two in one message does not work
 */ 
void cstInit(int busId)
{
  int i;

  //Syncronization
  canId=0x7e5;
  msg[0]=0x1;
  msg[1]=0x2;
  msg[2]=0x3;  
  msg[3]=0x4;
  msg[4]=0x5;
  msg[5]=0x6;
  msg[6]=0x7;
  msg[7]=0x8;

  EDBG("CST initialisation: should take around 30 sec.");
  for (i=0;i<500;i++)
    {
      my_send_can_message(busId,canId, msg);
      //EDBG(".\n");
      usleep(200);
    }

  EDBG("CST configuration");

  /* Switch mode global: only one CST present in network */

  /* Switch to configuration mode */
  msg[0]=0x4;
  msg[1]=0x1;
  my_send_can_message_var_length(busId,canId,2, msg);

  /* Check presence of module */
  msg[0]=0x82;
  my_send_can_message_var_length(busId, canId,1, msg);

  /* Make module tell Manufacturer */
  msg[0]=0x24;
  my_send_can_message_var_length(busId,canId,1, msg);
  
  usleep(5000);

  /* Make module tell product name */
  msg[0]=0x25;
  my_send_can_message_var_length(busId, canId,1, msg);
   
  usleep(5000);

  /* Make module tell serial number */
  msg[0]=0x26;
  my_send_can_message_var_length(busId, canId,1, msg);

  msg[0]=0x4; //switch to operation mode
  msg[1]=0x0;
  my_send_can_message_var_length(busId, canId,2, msg);


  //configuring variables
  msg[0]=0x4; //switch to config mode
  msg[1]=0x1;
  my_send_can_message_var_length(busId, canId, 2, msg);

  /* Var0 : read status
     Var0 : write control
     Var1 : write all channels
     Var2 : write channel 0
     Var3 : write channel 1
  */

  /* Configure write access to Var0 by CAN_ID 0x10 */
  msg[0]=0x80;
  msg[1]=0x0;
  msg[2]=0x0;  
  msg[3]=0x10;
  msg[4]=0x0;
  my_send_can_message_var_length(busId, canId, 5, msg);

  /* Configure reading access to Var0 by CAN_ID 0x11 */
  msg[0]=0x80;
  msg[1]=0x1;
  msg[2]=0x0;  
  msg[3]=0x11;
  msg[4]=0x0;
  my_send_can_message_var_length(busId, canId, 5, msg); 

  /* Configure byte variable Var1 for write access by CAN_ID 0x33 */
  msg[0]=0x80;
  msg[1]=0x0;
  msg[2]=0x1;  
  msg[3]=0x33;
  msg[4]=0x0;
  my_send_can_message_var_length(busId, canId, 5, msg);

  /* Configure byte variable Var2 for write access by CAN_ID 0x35 */
  msg[0]=0x80;
  msg[1]=0x0;
  msg[2]=0x2;  
  msg[3]=0x35; //CAN ID for voltage set pedal
  msg[4]=0x0;
  my_send_can_message_var_length(busId, canId, 5, msg);

  /* Configure byte variable Var3 for write access by CAN_ID 0x36 */
  msg[0]=0x80;
  msg[1]=0x0;
  msg[2]=0x3;  
  msg[3]=0x36; //CAN ID for voltage set steering
  msg[4]=0x0;
  my_send_can_message_var_length(busId, canId, 5, msg);
  
  msg[0]=0x4; //switch to operation mode
  msg[1]=0x0;
  my_send_can_message_var_length(busId, canId,2, msg);
  EDBG("CST configuration finished\n");
  old_e=0;
}

/*!
 *
 * \brief sending a CAN message looking like the one send by the ABS of the car
 *
 * Allows to fake the electronic power steering unit by sending it the message normaly sent by the ABS of the car but with the minimal velocity. This makes shure that the enhancement factor of the electronic power steering is allways sufficient to actually steer the car.
 *
 * 
 */
void cst_send_speed_msg(int busId)
{
  canId = 0x90;
  msg[0] = 0;
  msg[1] = 0x81;
  msg[2] = 0x75;
  msg[3] = 0x76;
  msg[4] = 0x0;
  //0x2c equals 2.75km/h, the minimal speed detected by the encoders
  msg[5] = 0x2c;
  msg[6] = 0x0;
  msg[7] = 0;
  //EDBG("sending speed msg\n");
  my_send_can_message(busId, canId,msg);
}

void cstIntSetVoltage(int busId,int voltage, int channel)
{
  /*sends a can message to the CST analoge output device containing
    a voltage as a 12bit integer voltage 
    The maximal voltage of 10V coresponds to an int of 4096 (decimal)*/

  if (voltage<4096) //check if more then 12bit is requested
    
    {
      if (channel==0) //pedal
	canId=0x35; 
      else if (channel==1) //steering
	canId=0x36;
      
      msg[0]=(voltage & 0x00ff);
      msg[1]=(voltage & 0xff00)>>8;

      my_send_can_message_var_length(busId, canId,2, msg);
    }
  else
    EDBG("Error: voltage is 12bit. Maximum voltage in int: 4096. You requested %d\n",voltage);
}

void cstDoubleSetVoltage(int busId, double voltage, int channel)
/* Sets a voltage given as double on the CST analoge
   output device */
{
  tmp = (voltage/V_MAX)*4096;
  cstIntSetVoltage(busId,(int) tmp, channel);
}


/* public function to set the voltage for the active steering */
void cstSetSteeringVoltage(int busId, double voltage)
{
  if ((voltage<=STEERING_PHYSICAL_MAX_LIMIT)&&(voltage>=STEERING_PHYSICAL_MIN_LIMIT))
    cstDoubleSetVoltage(busId,voltage,1);
  else
    EDBG("ERROR: voltage for steering out of physical limits\n");
}

/* public function to set pedal value in percent */
void cstSetPedalValue(int busId,double p)
/* sets a pedal value in percent of pedal pressed*/
{
  p=p*2.5;
  if ((p>=0)&&(p<=250))
    cstDoubleSetVoltage(busId, 0.0182 * p + 0.5325,0);
  else
    EDBG("ERROR: pedal value too high!\n");
}

const  double arw = 0.22;
static double x1;
static double tempo;

/* Optimal PID values according to Lamon Method:

P = 25
I = 0.2
D = 0.4
*/

/* Takes a target steering angle and current steering angle in radiants on the wheel and calulates the needed voltage output */
void cstSteeringPID(double P, double I, double D, double t, double target, 
  double current_value, double old_steering_voltage, double *output) {
  e = current_value - target;

  x1 = Integral + e*t;
  
  if(fabs(x1) > arw)
    x1 = arw;

  tempo = 2.5 + P*e + I*x1 + D*(e-old_e)/t; 

  Integral = x1;
  old_e = e;

  *output = tempo;

  //EDBG("Steering PID:\n error=%f P=%f I=%f D=%f tempo=%f \n",e,P,I,D,tempo);
}
