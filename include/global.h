#include "okapi/api.hpp"
extern pros::Controller master;
extern pros::ADIAnalogIn potCatapult;
extern okapi::ADIGyro gyro;

extern pros::Motor driveRightFront;
extern pros::Motor driveRightBack;
extern pros::Motor driveLeftFront;
extern pros::Motor driveLeftBack;

extern pros::Motor strafe;

extern pros::Motor intake;
extern pros::Motor catapult;
extern pros::Motor catapult2;



extern double gyroCurrent;
extern double gyroLast;
extern float gyroOutput;
extern float diff;
extern double gyroOutputReal;

extern int pinValues[4];

extern int getAutonNumber();

//pin declaration
extern pros::ADIDigitalIn pin1;
extern pros::ADIDigitalIn pin2;
extern pros::ADIDigitalIn pin3;
extern pros::ADIDigitalIn pin4;
