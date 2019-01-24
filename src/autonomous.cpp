#include "main.h"
#ifndef global
#include "global.h"
#endif
#include "okapi/api.hpp"
#include "MiniPID.h"

using namespace okapi::literals;
/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */




 void gyroadj(void* param) {
 	gyroCurrent = gyro.get();
 	gyroLast = gyro.get();
    while (true) {
 		 gyroCurrent = gyro.get();
 		 diff = gyroCurrent - gyroLast;
 		 gyroLast = gyroCurrent;
     if (abs(diff) < 100) {
       gyroOutput = gyroOutput + diff;
     }

 		 std::cout << gyro.get() << "    " << gyroOutput << std::endl;
      pros::delay(1);

 	}
 }





void turnPID (double target) {
  MiniPID pid = MiniPID(0.21, 0.000, 0.5);
  pid.setOutputLimits(-127, 127);
  pid.setMaxIOutput(30);
  pid.setSetpointRange(900);

  //reset the gyro value
  //gyro.reset();
  gyroOutput = 0;

  //double target = 900;
  int iterations = 0;

  while(iterations < 2000) {
     //std::cout << driveRightFront.get_actual_velocity() << std::endl;
     double output = pid.getOutput(gyroOutput / 1.075, target);
     driveRightFront.move(-output);
     driveRightBack.move(-output);
     driveLeftFront.move(output);
     driveLeftBack.move(output);
     pros::delay(10);
     if(driveRightFront.get_actual_velocity() == 0 && iterations > 30) {
        break;
     }
     //pros::lcd::print(0, "Gyro: %f\n", (gyro.get()) -900);
     //pros::lcd::print(1, "PID : %f\n", output);
     iterations = iterations + 10;


  }
}





void drivePID (double target) {
  int leftBias = 25;
  int rightBias = 0;
  if (target < 0) {
     leftBias = leftBias * -1;
     rightBias = rightBias * -1;
  }

  MiniPID leftPID = MiniPID(0.21, 0.000, 0.5);
  leftPID.setOutputLimits(-127, 127);
  leftPID.setMaxIOutput(30);
  leftPID.setSetpointRange(900);


  MiniPID rightPID = MiniPID(0.24, 0.000, 0.6);
  rightPID.setOutputLimits(-127, 127);
  rightPID.setMaxIOutput(30);
  rightPID.setSetpointRange(900);


  driveRightFront.tare_position();
  driveRightBack.tare_position();
  driveLeftFront.tare_position();
  driveLeftBack.tare_position();

  //double target = 900;
  int iterations = 0;
  float motorEncoderAverage;
  while(iterations < 4000) {
     motorEncoderAverage = (driveRightFront.get_position() + driveRightBack.get_position() + driveLeftFront.get_position() + driveLeftBack.get_position()) / 4;
     double leftOutput = leftPID.getOutput(motorEncoderAverage, target + leftBias);
     double rightOutput = rightPID.getOutput(motorEncoderAverage, target + rightBias);
     driveRightFront.move(rightOutput);
     driveRightBack.move(rightOutput);
     driveLeftFront.move(leftOutput);
     driveLeftBack.move(leftOutput);
     pros::delay(10);
     if(driveRightFront.get_actual_velocity() == 0 && iterations > 30) {
        break;
     }
     iterations = iterations + 10;


  }
  pros::lcd::print(1, "PID : %f\n", motorEncoderAverage);
}

void autonomous() {


pros::Task task_gyroadj(gyroadj);

switch(getAutonNumber()) {
  case 0:
    //red primary auton
    drivePID(3200);
    intake.move(-127);
    pros::delay(100);
    drivePID(-2500);
    intake.move(0);

    turnPID(1800);
    catapult.move(127);
    pros::delay(400);
    catapult.move(0);
    turnPID(900+420);
    intake.move(127);
    drivePID(1900);
    intake.move(0);
    drivePID(-2000);
    pros::delay(500);
    turnPID(1450);
    drivePID(1550);
    turnPID(-900);
    drivePID(4000);
    break;

  case 1:
    //red back
    drivePID(3200);
    intake.move(-127);
    pros::delay(300);
    drivePID(-500);
    turnPID(-1800-400);
    intake.move(0);
    drivePID(2450);
    turnPID(400);
    driveRightFront.move(50);
    driveRightBack.move(50);
    driveLeftFront.move(50);
    driveLeftBack.move(50);
    pros::delay(800);
    driveRightFront.move(0);
    driveRightBack.move(0);
    driveLeftFront.move(0);
    driveLeftBack.move(0);
    drivePID(-300);
    turnPID(275);
    pros::delay(200);
    catapult.move(127);
    pros::delay(400);
    catapult.move(0);
    turnPID(700);
    drivePID(3120);
    turnPID(900);
    drivePID(3200);
    break;

  case 2:
    //blue primary auton
    drivePID(3200);
    intake.move(-127);
    pros::delay(100);
    drivePID(-2810);
    intake.move(0);
    catapult.move(127);
    pros::delay(400);
    catapult.move(0);
    turnPID(490);
    intake.move(127);
    drivePID(2600);
    intake.move(0);
    drivePID(-2400);
    turnPID(-1300);
    drivePID(2000);
    turnPID(900);
    drivePID(4000);
    break;

  case 3:
    //Blue back
    drivePID(3200);
    intake.move(-127);
    pros::delay(300);
    drivePID(-500);
    turnPID(400);
    intake.move(0);
    drivePID(-2450);
    turnPID(-400);
    driveRightFront.move(-50);
    driveRightBack.move(-50);
    driveLeftFront.move(-50);
    driveLeftBack.move(-50);
    pros::delay(1000);
    driveRightFront.move(0);
    driveRightBack.move(0);
    driveLeftFront.move(0);
    driveLeftBack.move(0);
    drivePID(300);
    turnPID(-259);
    pros::delay(200);
    catapult.move(127);
    pros::delay(400);
    catapult.move(0);
    pros::delay(200);
    turnPID(900+259);
    drivePID(3120);
    turnPID(-900);
    drivePID(3650);
    break;
}

















}
