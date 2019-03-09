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

void turnPID (double target) {
  MiniPID pid = MiniPID(0.27, 0.000, 1.8);
  pid.setOutputLimits(-127, 127);
  pid.setMaxIOutput(30);
  pid.setSetpointRange(900);
  //int bias = std::nearbyint(0.059722*target);
  int bias = 0;
  float gyroCalibrationConst = 1;
  if(target < 0) {
    gyroCalibrationConst = 1.01;
  } else if (target > 0) {
    gyroCalibrationConst = 1.01;
  }
  //reset the gyro value
  gyroOutputReal = 0;
  int iterations = 0;
  int timeout = 0;
  while(iterations < 2000) {
     double output = pid.getOutput(gyroOutput/gyroCalibrationConst, target + bias);
     driveRightFront.move(-output);
     driveRightBack.move(-output);
     driveLeftFront.move(output);
     driveLeftBack.move(output);
     pros::delay(10);

     if(driveRightFront.get_actual_velocity() == 0 && iterations > 30) {
        timeout = timeout + 10;
        if(timeout >= 50) {
          break;
        }
     }
     pros::lcd::print(0, "Gyro: %f\n", gyroOutput);
     pros::lcd::print(1, "Gyroadj : %f\n", gyroOutput / gyroCalibrationConst);
     pros::lcd::print(1, "Error : %f\n", abs(target) - abs(gyroOutput / gyroCalibrationConst));
     iterations = iterations + 10;
  }
  driveRightFront.move(0);
  driveRightBack.move(0);
  driveLeftFront.move(0);
  driveLeftBack.move(0);
}

void turnPIDTime (double target, double time, bool reset = true) {
  MiniPID pid = MiniPID(0.27, 0.000, 1.8);
  pid.setOutputLimits(-127, 127);
  pid.setMaxIOutput(30);
  pid.setSetpointRange(900);
  //int bias = std::nearbyint(0.059722*target);
  int bias = 0;
  //bias was 38
  float gyroCalibrationConst = 1;
  if(target < 0) {
    gyroCalibrationConst = 1.01;
  } else if (target > 0) {
    gyroCalibrationConst = 1.01;
  }
  //reset the gyro value
  if(reset){
    gyroOutputReal = 0;
  }
  int iterations = 0;
  int timeout = 0;
  while(iterations < time) {
     double output = pid.getOutput(gyroOutput/gyroCalibrationConst, target + bias);
     driveRightFront.move(-output);
     driveRightBack.move(-output);
     driveLeftFront.move(output);
     driveLeftBack.move(output);
     pros::delay(10);

     pros::lcd::print(0, "Gyro: %f\n", gyroOutput);
     pros::lcd::print(1, "Gyroadj : %f\n", gyroOutput / gyroCalibrationConst);
     pros::lcd::print(1, "Error : %f\n", abs(target) - abs(gyroOutput / gyroCalibrationConst));
     iterations = iterations + 10;
  }
  driveRightFront.move(0);
  driveRightBack.move(0);
  driveLeftFront.move(0);
  driveLeftBack.move(0);
}

void drivePID (double target, double setPointRange = 900, double rightBias = 10, double sideBias = 1) {
  int leftBias = 0;

  if (target < 0) {
     leftBias = leftBias * -1;
     rightBias = -10;
  }

  MiniPID leftPID = MiniPID(0.24, 0.000, 0.50);
  leftPID.setOutputLimits(-127, 127);
  leftPID.setMaxIOutput(30);
  leftPID.setSetpointRange(setPointRange);

  MiniPID rightPID = MiniPID(0.24, 0.000, 0.50);
  rightPID.setOutputLimits(-127, 127);
  rightPID.setMaxIOutput(30);
  rightPID.setSetpointRange(setPointRange);



/*
  MiniPID leftPID = MiniPID(0.21, 0.000, 0.5);
  leftPID.setOutputLimits(-127, 127);
  leftPID.setMaxIOutput(30);
  leftPID.setSetpointRange(setPointRange);


  MiniPID rightPID = MiniPID(0.24, 0.000, 0.6);
  rightPID.setOutputLimits(-127, 127);
  rightPID.setMaxIOutput(30);
  rightPID.setSetpointRange(setPointRange);
*/

  driveRightFront.tare_position();
  driveRightBack.tare_position();
  driveLeftFront.tare_position();
  driveLeftBack.tare_position();

  //double target = 900;
  int iterations = 0;
  int timeout = 0;
  float leftMotorEncoderAverage;
  float rightMotorEncoderAverage;

  while(iterations < 2500) {
     leftMotorEncoderAverage = (driveLeftFront.get_position());// + driveLeftBack.get_position()) / 2;
     rightMotorEncoderAverage = (driveRightFront.get_position());// + driveRightBack.get_position()) / 2;
     double leftOutput = leftPID.getOutput(leftMotorEncoderAverage, target + leftBias);
     double rightOutput = sideBias * rightPID.getOutput(rightMotorEncoderAverage, target + rightBias);
     driveRightFront.move(rightOutput);
     driveRightBack.move(rightOutput);
     driveLeftFront.move(leftOutput);
     driveLeftBack.move(leftOutput);
     pros::delay(10);
     if(driveRightFront.get_actual_velocity() == 0 && iterations > 30) {
        timeout = timeout + 10;
        if(timeout >= 50) {
          break;
        }
     }
     iterations = iterations + 10;


  }
  //pros::lcd::print(1, "PID : %f\n", motorEncoderAverage);
  std::cout << "Left Error: " << leftMotorEncoderAverage << std::endl;
  std::cout << "Right Error: " << rightMotorEncoderAverage << std::endl;
}

void drivePIDTime (double target, double time, double setPointRange = 900, double rightBias = 10, double sideBias = 1) {
  int leftBias = 0;

  if (target < 0) {
     leftBias = leftBias * -1;
     rightBias = -10;
  }

  MiniPID leftPID = MiniPID(0.24, 0.000, 0.50);
  leftPID.setOutputLimits(-127, 127);
  leftPID.setMaxIOutput(30);
  leftPID.setSetpointRange(setPointRange);

  MiniPID rightPID = MiniPID(0.24, 0.000, 0.50);
  rightPID.setOutputLimits(-127, 127);
  rightPID.setMaxIOutput(30);
  rightPID.setSetpointRange(setPointRange);

  driveRightFront.tare_position();
  driveRightBack.tare_position();
  driveLeftFront.tare_position();
  driveLeftBack.tare_position();

  int iterations = 0;
  int timeout = 0;
  float leftMotorEncoderAverage;
  float rightMotorEncoderAverage;

  while(iterations < time) {
     leftMotorEncoderAverage = (driveLeftFront.get_position());// + driveLeftBack.get_position()) / 2;
     rightMotorEncoderAverage = (driveRightFront.get_position());// + driveRightBack.get_position()) / 2;
     double leftOutput = leftPID.getOutput(leftMotorEncoderAverage, target + leftBias);
     double rightOutput = sideBias * rightPID.getOutput(rightMotorEncoderAverage, target + rightBias);
     driveRightFront.move(rightOutput);
     driveRightBack.move(rightOutput);
     driveLeftFront.move(leftOutput);
     driveLeftBack.move(leftOutput);
     pros::delay(10);
     iterations = iterations + 10;
  }
  //pros::lcd::print(1, "PID : %f\n", motorEncoderAverage);
  std::cout << "Left Error: " << leftMotorEncoderAverage << std::endl;
  std::cout << "Right Error: " << rightMotorEncoderAverage << std::endl;
}

void red_front_auton() {
  drivePIDTime(3000, 1700);
  intake.move(127);
  pros::delay(400);
  drivePIDTime(-2250, 1500);
  turnPIDTime(1800, 1500, false);
  intake.move(0);
  fired = true;
  turnPIDTime(3100, 1000, false);
  pros::delay(300);
  intake.move(-127);
  drivePIDTime(900, 800, 900, -60, 1);
  drivePIDTime(890, 1500, 280, 10, 1);
  intake.move(0);
  drivePIDTime(-1700, 1300);
  turnPIDTime(4550, 1000, false);
  intake.move(127);
  drivePID(1850, 1400);
  turnPIDTime(3550, 1000, false);
  intake.move(0);
  drivePIDTime(3500, 5000);
}
void red_back_auton() {
  //red back
  drivePID(3100);
  pros::delay(2000);
  drivePID(-3100);
  pros::delay(2000);
  turnPID(3600);
  pros::delay(2000);
  drivePID(1000);
  turnPID(1800);
  drivePID(1000);
  pros::delay(2000);
  return;
  drivePID(3200);
  intake.move(-127);
  pros::delay(300);
  drivePID(-500);
  turnPID(-1800-400);
  intake.move(0);
  drivePID(2300);
  turnPID(400);
  driveRightFront.move(50);
  driveRightBack.move(50);
  driveLeftFront.move(50);
  driveLeftBack.move(50);
  pros::delay(1400);
  driveRightFront.move(0);
  driveRightBack.move(0);
  driveLeftFront.move(0);
  driveLeftBack.move(0);
  drivePID(-335);
  turnPID(285);
  pros::delay(200);
  fired = true;
  turnPID(700);
  drivePID(3350);
  turnPID(900);
  drivePID(3200);
}
void blue_front_auton() {
  //blue primary auton
  drivePID(3100);
  intake.move(127);
  pros::delay(100);
  drivePID(-2700);
  intake.move(0);
  fired = true;
  turnPID(600);
  intake.move(-127);
  drivePID(1250);
  drivePID(800, 250);
  intake.move(0);
  drivePID(-1550);
  turnPID(-1200);
  drivePID(1750);
  turnPID(900);
  drivePID(4000);
}
void blue_back_auton() {
  //Blue back

  drivePID(3100);
  intake.move(127);
  pros::delay(300);
  drivePID(-500);
  intake.move(0);


}
void red_front_nopark_auton() {

  drivePIDTime(3000, 1700);
  intake.move(127);
  pros::delay(400);
  drivePIDTime(-2250, 1500);
  turnPIDTime(1800, 1500, false);
  intake.move(0);
  fired = true;
  pros::delay(400);
  turnPIDTime(2635, 1000, false);

  intake.move(127);
  drivePIDTime(3500, 2100, false);
  drivePIDTime(-1700, 900);

  turnPIDTime(3620, 900, false);
  intake.move(-127);
  drivePIDTime(600, 700, 900, -60, 1);
  drivePIDTime(750, 900, 280, 10, 1);
  intake.move(0);

  turnPIDTime(2340, 1200, false);
  fired = true;
  pros::delay(400);

}

void blue_front_nopark_auton() {
  drivePID(3200);
  intake.move(-127);
  pros::delay(100);
  drivePID(-2810);
  intake.move(0);
  fired = true;

  turnPID(900);
  drivePID(1600);
  turnPID(-900);
  driveRightFront.move(-50);
  driveRightBack.move(-50);
  driveLeftFront.move(-50);
  driveLeftBack.move(-50);
  pros::delay(700);
  driveRightFront.move(0);
  driveRightBack.move(0);
  driveLeftFront.move(0);
  driveLeftBack.move(0);

  intake.move(127);
  drivePID(700);
  drivePID(1400, 250);
  intake.move(0);
  pros::delay(200);
  drivePID(-1465);
  turnPID(1010);
  driveRightFront.move(50);
  driveRightBack.move(50);
  driveLeftFront.move(50);
  driveLeftBack.move(50);
  pros::delay(2000);
  driveRightFront.move(0);
  driveRightBack.move(0);
  driveLeftFront.move(0);
  driveLeftBack.move(0);

}
void red_front_mid_nopark_auton() {
  drivePID(3200);
  intake.move(-127);
  pros::delay(100);
  drivePID(-2500);
  intake.move(0);

  turnPID(1800);
  fired = true;

  turnPID(900);
  drivePID(1600);
  turnPID(900);
  /*
  driveRightFront.move(-50);
  driveRightBack.move(-50);
  driveLeftFront.move(-50);
  driveLeftBack.move(-50);
  pros::delay(800);
  driveRightFront.move(0);
  driveRightBack.move(0);
  driveLeftFront.move(0);
  driveLeftBack.move(0);
  */
  intake.move(127);
  drivePID(700);
  drivePID(850, 250);
  intake.move(0);
  drivePID(-150);
  turnPID(-750);
  drivePID(1500);
  turnPID(860);
  drivePID(1250, 500);
}
void prog_skills() {
  //red primary auton
  //basically just red primary with extended drive
  // line up with the left side of the tile
  drivePID(3200);
  intake.move(-127);
  pros::delay(100);
  drivePID(-2500);
  intake.move(0);

  turnPID(1800);
  fired = true;
  turnPID(900+420);
  intake.move(127);
  drivePID(1100);
  drivePID(800, 250);
  intake.move(0);
  drivePID(-2000);
  pros::delay(500);
  turnPID(1400);
  drivePID(1550);
  turnPID(-900);
  drivePID(7000);
}

void testing() {

  auto myChassis = okapi::ChassisControllerFactory::create(
  {-2, -4}, // Left motors
  {1, 3},   // Right motors
  okapi::AbstractMotor::gearset::green,
  {4_in, 12.5_in} // 4 inch wheels, 12.5 inch wheelbase width
);

  auto profileController = okapi::AsyncControllerFactory::motionProfile(
  1.0,  // Maximum linear velocity of the Chassis in m/s
  2.0,  // Maximum linear acceleration of the Chassis in m/s/s
  10.0, // Maximum linear jerk of the Chassis in m/s/s/s
  myChassis  // Chassis Controller
 );
  profileController.generatePath({okapi::Point{0_ft, 0_ft, 0_deg}, okapi::Point{4_ft, 0_ft, 0_deg}}, "A");
  profileController.generatePath({okapi::Point{0_ft, 0_ft, 0_deg}, okapi::Point{4_ft, 0_ft, 0_deg}}, "B");
  profileController.setTarget("A");
  profileController.waitUntilSettled();
  profileController.setTarget("B");
  profileController.waitUntilSettled();


}

void autonomous() {


  switch(getAutonNumber()) {
    case 0:
      red_front_auton();
      break;
    case 1:
      red_back_auton();
      break;
    case 2:
      blue_front_auton();
      break;
    case 3:
      blue_back_auton();
      break;
    case 4:
      red_front_nopark_auton();
      break;
    case 5:
      blue_front_nopark_auton();
      break;
    case 6:
      prog_skills();
      break;
    case 7:
      testing();
      break;
}

}
