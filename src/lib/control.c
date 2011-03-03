/**********************************************
*********************************************/  

#include "control.h"

#include <stdio.h>
#include <math.h>
#include <sys/time.h>

#include <libelrob/Edebug.h>
#include <libelrob/Etime.h>

#include "smart.h"
#include "fuzzy_control.h"

#define VELOCITY_DER_WINDOW 3
#define VELOCITY_INT_WINDOW 3

// define AccPredict factor
#define a -0.15
#define b -0.5
#define c 6 // Can be adjust ...


double discrete_integrate(double value, const double delta, const double delay){

  double value1 = 0;
  // Incrementing value discretely
  value1 = value + delta*(delay/1000); //Delay in second
  return (double)value1;
}

double discrete_derivative(double deltaValue, const float delay){

  double value = 0;

  // Finding the derivative discretely
	if(delay != 0) value = deltaValue/(delay/1000); // prevent division by 0 because of patch in motionCtrl.

  return (double)(value);
}

double pid_ctrl(double cmd_err, double derivative, double integrate, double kp, double ki, double kd)
{
	double tmp = 0;
	tmp = 		kp * cmd_err;
	tmp = tmp + ki * integrate;
	tmp = tmp + kd * derivative;
	return tmp;
}

double saturation (double value, double min, double max)
{
	if(value < min){
		value = min;
	}
  	else if(value > max){ 
		value = max;
	}
	return value;
}

void valueShiftinTable(double table[], double newValue,int size){
	int i;
	for( i = 0; i<size ; i++ ){
		table[i] = table[i+1];
	}
	table[size-1] = newValue;
}

double predictAcc(double acc_pedal, int gear){
  
  double d = 0; // UserMade factor based on test made on the car
  double a_predicted;
  
  switch(gear) {
  case 0:
    a_predicted = 0;
    EDBG("\nWarning: car in gear position N\n  acc predicted forced to zero");
    break;
  case 1:
    d = 2.7; // depending of gear
    a_predicted = (1 / (1 + exp((-(acc_pedal+b)*c))))*(-a+d)+a;
    break;
  case 2:
    d = 1.87;
    a_predicted = (1 / (1 + exp((-(acc_pedal+b)*c))))*(-a+d)+a;
    break;		
  case 3:
    d = 1.27;
    a_predicted = (1 / (1 + exp((-(acc_pedal+b)*c))))*(-a+d)+a;
    break;
  case 4:
    d = 0.87;
    a_predicted = (1 / (1 + exp((-(acc_pedal+b)*c))))*(-a+d)+a;
    break;
  case 5:
    d = 0.56;
    a_predicted = (1 / (1 + exp((-(acc_pedal+b)*c))))*(-a+d)+a;
    break;
  default:
    a_predicted = 0;
    EDBG("\n Warning: Unkown gear, predicted acc forced to zero");
  }

  return a_predicted;
}



/*
void bin_prnt_byte(int number){
	int n;
	printf("Byte of %d: ",number);
	for(n=0; n<8; n++){
		if((number & 0x80) != 0) printf("1");
		else printf("0");
		if (n == 3) printf(" ");
		
		number = number << 1;
	}
	printf(" \n");
}
void bin_prnt_int(int number){
	int hi, lo;
	printf("Int of %d: ",number);
	hi = (number >> 8) & 0xff;
	lo = number & 0xff;
	bin_prnt_byte(hi);
	printf(" ");
	bin_prnt_byte(lo);
	printf("\n");
}
*/

void mctrl_accelerationControl(MCTRL_ACC_INPUT *mctrlAccInput, 
             MCTRL_CONFIG *config, 
             SMART_MOTION *smartMotion, 
             double v_command, 
             SMART_ENGINE *engine,
             double t_curr)
{
//   EDBG_ENABLE();
  /* Modifying some data */
  // Building table history of Velocity with time [Vi, Vi-1, Vi-2, ... , Vi-b] RAW DATA

  // Building timestamp history at each pass.
  valueShiftinTable(mctrlAccInput->timestamp_v, t_curr, WINDOW);

  valueShiftinTable(mctrlAccInput->velocity_raw,
        smartMotion->v_curr,
        WINDOW);

  double velocity_filt = (mctrlAccInput->velocity_raw[0] 
        + 2*mctrlAccInput->velocity_raw[1] 
        + mctrlAccInput->velocity_raw[2])/4;  
    
  valueShiftinTable(mctrlAccInput->velocity_filtered,
        velocity_filt,
        WINDOW);
    
  // Calculating velocity error
  double velocity_err = (v_command 
       - mctrlAccInput->velocity_filtered[WINDOW-1]);
  
  valueShiftinTable(mctrlAccInput->velocity_err,
        velocity_err,
        WINDOW);
    
  //Calculating Xaccel by derivating velocity
  double delay_acc = (mctrlAccInput->timestamp_v[VELOCITY_DER_WINDOW-1] 
          - mctrlAccInput->timestamp_v[0]) * 1e3;
//   EDBG("delay_acc = %f, new = %f, old = %f", delay_acc, mctrlAccInput->timestamp_v  [WINDOW-1], mctrlAccInput->timestamp_v[0] );

  // DIZAN
  if (delay_acc > 1000) delay_acc = 0; // PATCH ... TO CLEAN

  double acc = discrete_derivative((mctrlAccInput->velocity_filtered[VELOCITY_DER_WINDOW-1]
            - mctrlAccInput->velocity_filtered[0]), delay_acc);
    
  // Calculating delay for Integration
  double delay_int = (mctrlAccInput->timestamp_v[VELOCITY_INT_WINDOW-1]
          - mctrlAccInput->timestamp_v[VELOCITY_INT_WINDOW-2]) * 1e3;
// DIZAN
  if (delay_int > 1000) delay_int = 0; // PATCH ... TO CLEAN
    
//   EDBG("velocity_err = %f", mctrlAccInput->velocity_err[WINDOW-1]);
//   EDBG("acc = %f", acc);
//   EDBG("delay_int = %f", delay_int);

  // -----------------------------
  // Brake control
  // -----------------------------
  if (config->enable_brake_ctrl) {
//     EDBG("AUTO brake pedal controle: ON");
    
    // Ensure that the brake motor is functional and responding
    if (mctrlAccInput->brake_ready_to_serve){
      
      // Take relay when the gas pedal is released
      if (mctrlAccInput->gas_pedal_cmd <= 0){
  double delta_brake_pedal = (mctrlAccInput->brake_accurate_range * 
            fuzzy_acc_ctl(-acc, -mctrlAccInput->velocity_err[WINDOW-1]));
//   EDBG("delta brake pedal value = %f", delta_brake_pedal);
  
  // Stop pressing the brake pedal if decceleration is too hight
  if (acc > config->car_min_acc || delta_brake_pedal < 0) {
    mctrlAccInput->brake_pedal_cmd = discrete_integrate(mctrlAccInput->brake_pedal_cmd, 
                    delta_brake_pedal, delay_int);
    EDBG("brake_cmd: %f", mctrlAccInput->brake_pedal_cmd);
    mctrlAccInput->brake_pedal_cmd = saturation(mctrlAccInput->brake_pedal_cmd,
                  mctrlAccInput->brake_accurate_offset,
                  (mctrlAccInput->brake_accurate_offset + 
                   mctrlAccInput->brake_accurate_range));
    EDBG("brake_cmd: %f", mctrlAccInput->brake_pedal_cmd);
  }
  else {
//     EDBG("Car deceleration too hight, not pressing any more the brake pedal");
  }
      }
    }
    else {
//       EDBG("\n Error: brake not in proper configuration.\n -> Must stop the car AND give authorisation.");
    }
  }
  else {
//     EDBG("AUTO brake pedal controle: OFF");
//     EDBG("Forcing brake cmd to %f",mctrlAccInput->brake_accurate_offset);
    mctrlAccInput->brake_pedal_cmd = mctrlAccInput->brake_accurate_offset;
  }
         
  // -----------------------------   
  // Gas pedal control
  // -----------------------------
  if (config->enable_gas_ctrl) {
//     EDBG("AUTO gas pedal control: ON");

    if (engine->actual_gear == 1 ||
  engine->actual_gear == 2 ||
  engine->actual_gear == 3 ||
  engine->actual_gear == 4 ||
  engine->actual_gear == 5 ||
  engine->actual_gear == 6 ) {

      double acc_predicted = (GAIN_REAL_ACC * acc 
            + GAIN_PREDIC_ACC *(predictAcc((mctrlAccInput->gas_pedal_cmd/GAS_PEDAL_MAX_VALUE),
                   engine->actual_gear)));
      
      // Using FuzzyLogic Controller
      double delta_gas_pedal =  (GAS_PEDAL_MAX_VALUE
         * (fuzzy_acc_ctl(acc_predicted, mctrlAccInput->velocity_err[WINDOW-1])));
//       EDBG("fuzzy input: (modif Acc; vel err) = %f; %f", acc_predicted, mctrlAccInput->velocity_err[WINDOW-1]);

      delta_gas_pedal  = saturation(delta_gas_pedal, -GAS_PEDAL_MAX_DELTA, GAS_PEDAL_MAX_DELTA);
//       EDBG("delta gas pedal value = %f", delta_gas_pedal);

      // Integrating to have a position instead of velocity
      //if (acc > config->car_max_acc && delta_gas_pedal < 0) {
//   EDBG("Car acceleration too hight, not pressing any more the gas pedal");
  //}
      //else {
  mctrlAccInput->gas_pedal_cmd = discrete_integrate(mctrlAccInput->gas_pedal_cmd, delta_gas_pedal, delay_int);
  mctrlAccInput->gas_pedal_cmd = saturation(mctrlAccInput->gas_pedal_cmd, GAS_PEDAL_MIN_VALUE, GAS_PEDAL_MAX_VALUE);
  
  //}
    }
    else {
//       EDBG("\nWarning: gear not in a controlable position\n  Forcing gas pedal value to zero");
      mctrlAccInput->gas_pedal_cmd = 0;
    }
  }
  else {
//     EDBG("AUTO gas pedal controle: OFF");
//     EDBG("  Forcing gas pedal value to zero");
    mctrlAccInput->gas_pedal_cmd = 0;
  }
  
/*  fprintf(stdout, "gas=%6.2f  brake=%6.2f\n", 
    mctrlAccInput->gas_pedal_cmd, mctrlAccInput->brake_pedal_cmd);
  fflush(stdout);*/
//   EDBG_DISABLE();
}
