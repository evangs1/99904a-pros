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

okapi::ADIUltrasonic ultrasonicLeft(3, 4);
okapi::ADIUltrasonic ultrasonicRight(5, 6);



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

void drivePID (double target, double setPointRange = 900, double rightBias = 30) {
  int leftBias = 0;
  //int rightBias = 30;
  if (target < 0) {
     leftBias = leftBias * -1;
     rightBias = rightBias * -1;
  }

  MiniPID leftPID = MiniPID(0.28, 0.000, 0.55);
  leftPID.setOutputLimits(-127, 127);
  leftPID.setMaxIOutput(30);
  leftPID.setSetpointRange(setPointRange);


  MiniPID rightPID = MiniPID(0.20, 0.000, 0.55);
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

void ultrasonicAlignPID () {
  MiniPID sonarPID = MiniPID(0.08, 0.000, 0.5);
  sonarPID.setOutputLimits(-127, 127);
  int iterations = 0;
  while (iterations < 4000) {
    double error = ultrasonicLeft.get() - ultrasonicRight.get();
    double sonarOutput = sonarPID.getOutput(error);

    driveRightFront.move(-sonarOutput);
    driveRightBack.move(-sonarOutput);
    driveLeftFront.move(sonarOutput);
    driveLeftBack.move(sonarOutput);
    pros::delay(10);
    if(driveRightFront.get_actual_velocity() == 0 && iterations > 50) {
      break;
    }
  }
}

void red_front_auton() {
  //red primary auton
  // line up with the left side of the tile
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
  drivePID(1100);
  drivePID(800, 250);
  intake.move(0);
  drivePID(-2000);
  pros::delay(500);
  turnPID(1400);
  drivePID(1550);
  turnPID(-900);
  drivePID(4000);
}
void red_back_auton() {
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
}
void blue_front_auton() {
  //blue primary auton
  drivePID(3200);
  intake.move(-127);
  pros::delay(100);
  drivePID(-2750);
  intake.move(0);
  catapult.move(127);
  pros::delay(400);
  catapult.move(0);
  turnPID(490);
  intake.move(127);
  drivePID(1100);
  drivePID(800, 250);
  intake.move(0);
  drivePID(-1900);
  turnPID(-1300);
  drivePID(1900);
  turnPID(900);
  drivePID(4000);
}
void blue_back_auton() {
  //Blue back
  drivePID(3200);
  intake.move(-127);
  pros::delay(300);
  drivePID(-500);
  turnPID(400);
  intake.move(0);
  drivePID(-2250);
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
}
void red_front_nopark_auton() {
  drivePID(3200);
  intake.move(-127);
  pros::delay(100);
  drivePID(-2500);
  intake.move(0);

  turnPID(1800);
  catapult.move(127);
  pros::delay(400);
  catapult.move(0);

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

  drivePID(1400, 260);
  intake.move(0);
  pros::delay(200);
  drivePID(-1400);
  turnPID(-1000);
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
void blue_front_nopark_auton() {
  drivePID(3200);
  intake.move(-127);
  pros::delay(100);
  drivePID(-2810);
  intake.move(0);
  catapult.move(127);
  pros::delay(400);
  catapult.move(0);

  turnPID(900);
  drivePID(1600);
  turnPID(-900);

  intake.move(127);

  drivePID(1400, 260);
  intake.move(0);
  pros::delay(200);
  drivePID(-1400);
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
  catapult.move(127);
  pros::delay(400);
  catapult.move(0);

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
  catapult.move(127);
  pros::delay(400);
  catapult.move(0);
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

void prog_skills_1() {
  //red primary auton
  //basically just red primary with extended drive
  // line up with the left side of the tile

  turnPID(-1800);
  /*
  drivePID(3200);
  intake.move(-127);
  pros::delay(300);
  drivePID(-500, 900, 0);
  intake.move(127);
  drivePID(800, 250, 0);
  intake.move(0);
  */
}

void autonomous() {
  pros::Task task_gyroadj(gyroadj);

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
      prog_skills_1();
      break;
}

}
