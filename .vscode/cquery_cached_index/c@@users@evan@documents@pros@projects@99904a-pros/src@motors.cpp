#include "main.h"
#ifndef global
#include "global.h"
#endif
#include "okapi/api.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::ADIAnalogIn potCatapult(POT_CATAPULT_PORT);
okapi::ADIGyro gyro(GYRO_PORT, 0.928);

//Motor Setup
pros::Motor driveRightFront(DRIVE_RIGHT_FRONT_PORT, pros::E_MOTOR_GEARSET_18, false);
pros::Motor driveRightBack(DRIVE_RIGHT_BACK_PORT, pros::E_MOTOR_GEARSET_18, false);
pros::Motor driveLeftFront(DRIVE_LEFT_FRONT_PORT, pros::E_MOTOR_GEARSET_18, true);
pros::Motor driveLeftBack(DRIVE_LEFT_BACK_PORT, pros::E_MOTOR_GEARSET_18, true);

pros::Motor strafe(STRAFE_PORT, pros::E_MOTOR_GEARSET_18);

pros::Motor intake(INTAKE_PORT);
pros::Motor catapult(CATAPULT_PORT);
pros::Motor catapult2(CATAPULT2_PORT);


double gyroCurrent = 0;
double gyroLast = 0;
double difference;
float gyroOutput = 0;
float diff = 0;
double gyroOutputReal = 0;

int pinValues[4] = {0, 0, 0, 0};

int getAutonNumber() {
  pinValues[0] = pin1.get_value();
  pinValues[1] = pin2.get_value();
  pinValues[2] = pin3.get_value();
  pinValues[3] = pin4.get_value();

  int value = pinValues[0] + (pinValues[1] * 2) + (pinValues[2] * 4) + (pinValues[3] * 8);

  return value;
}

//pin declaration
pros::ADIDigitalIn pin1(1);
pros::ADIDigitalIn pin2(2);
pros::ADIDigitalIn pin3(3);
pros::ADIDigitalIn pin4(4);
