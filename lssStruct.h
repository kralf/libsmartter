#ifndef SMART_LSS_STRUCT
#define SMART_LSS_STRUCT 

#include <elrob/Etypes.h>

typedef struct LSS_ERROR {
  EBOOL following;
  EBOOL negative_limit;
  EBOOL positive_limit;
  EBOOL stroke_reverse;
  EBOOL stroke_foward;
  EBOOL input;
}LSS_ERROR;


typedef struct LSS_STR{
  EBOOL accept_ajustment;

  double max_position;
  double min_position;
  double actual_position;
  double target_position;
  double max_armature_curr_moving;
  double max_armature_curr_holding;

  int testingMenu; // to be remove after test on brake
  double fakeCmd; // to be remove

  // New Ternary Linear Servo System internal state
  EBOOL enable_moving;
  EBOOL homing_done;
  EBOOL shaft_moving_up;
  EBOOL on_target_position;
  EBOOL motor_moving;
  EBOOL command_loaded;
  EBOOL general_error;
  LSS_ERROR error;
}LSS_STR;

#endif
