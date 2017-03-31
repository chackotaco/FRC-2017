#ifndef PARAMS_H
#define PARAMS_H
#include "WPILib.h"

//Params.h: Preferences for the robot
static bool SQUARE_DRIVE_AXIS_INPUT                    = true;
static const bool USE_ARCADE_DRIVE                     = true;

static double GLOBAL_DRIVE_SPEED_MULTIPLIER            = 1.0;

static const double CLIMBER_HARDSET_MOTOR_SPEED        = 1.0;

//GEAR HOLDER PARAMS
static const double GEAR_WHEELS_RESTING_MOTOR_SPEED    = 0.0;
static const double GEAR_WHEELS_ACTIVE_MOTOR_SPEED     = 0.75;
static const double GEAR_TILTER_MAX_MOTOR_SPEED        = 0.4;
static const double GEAR_POT_UP_POSITION               = 0.294;
static const double GEAR_POT_DOWN_POSITION             = 0.320;

//DRIVE PID PARAMS

static const double DRIVE_Y_PID_VALUES[]               = {1.0, 0.0, 0.0}; // P, I, D
static const double DRIVE_Y_PID_SCALE_VALUES[] 		   = {0.125, 1.0, 1.0}; //P, I, D

static const int DRIVE_Y_PID_TOLERANCE                 = 0.5;
static const int DRIVE_Y_PID_SAMPLES_AVERAGE           = 1;

static const double DRIVE_X_PID_VALUES[]               = {0.0, 0.0, 0.0}; // P, I, D
static const double DRIVE_X_PID_SCALE_VALUES[] 		   = {1.0, 1.0, 1.0}; //P, I, D

static const int DRIVE_X_PID_TOLERANCE                 = 10;
static const int DRIVE_X_PID_SAMPLES_AVERAGE           = 10;


//PINI UPDATES
static double PINI_P;
static double PINI_D;
static double PINI_I;

#endif
