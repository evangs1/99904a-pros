#include "main.h"
#ifndef global
#include "global.h"
#endif

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::ADIAnalogIn potCatapult(POT_CATAPULT_PORT);

//Motor Setup
pros::Motor driveRightFront(DRIVE_RIGHT_FRONT_PORT, pros::E_MOTOR_GEARSET_18, false);
pros::Motor driveRightBack(DRIVE_RIGHT_BACK_PORT, pros::E_MOTOR_GEARSET_18, false);
pros::Motor driveLeftFront(DRIVE_LEFT_FRONT_PORT, pros::E_MOTOR_GEARSET_18, true);
pros::Motor driveLeftBack(DRIVE_LEFT_BACK_PORT, pros::E_MOTOR_GEARSET_18, true);
pros::Motor intake(INTAKE_PORT);
pros::Motor catapult(CATAPULT_PORT);
