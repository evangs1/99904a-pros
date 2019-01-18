#include "main.h"
#include <iostream>
#ifndef global
#include "global.h"
#endif
//#include "define.hpp"

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */




void catapultAutomation(void* param) {
	bool catapultIsPrimed = true;
	bool aborted = false;
   while (true) {
		  if(master.get_digital(DIGITAL_X)) {
 				aborted = !aborted;
				catapult.move(0);
				pros::delay(400);
 			}
      if (!catapultIsPrimed && !aborted) {
				catapult.move(127);
      }
			if(catapultIsPrimed) {
				catapult.move(0);
				catapult.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			}
      if(master.get_digital(DIGITAL_L2) && catapultIsPrimed) {
			catapult.move(127);
         pros::delay(400);
      }
		catapultIsPrimed = !(potCatapult.get_value() < 3100);
      pros::delay(20);
	}
}





void opcontrol() {

	pros::Task task_catapult_automation(catapultAutomation);

	while (true) {
   	//pros::lcd::print("potentiometer: %d", potCatapult.get_value());
		int turn = master.get_analog(ANALOG_LEFT_X);
		int power = master.get_analog(ANALOG_LEFT_Y);
		int left = power + turn;
		int right = power - turn;
		//change to move_velocity?

		if (abs(turn) + abs(power) < 1) {
			driveRightFront.move(0);
			driveRightBack.move(0);
			driveLeftFront.move(0);
			driveLeftBack.move(0);
			driveRightFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			driveRightBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			driveLeftFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			driveLeftBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
		} else {
			driveRightFront.move(right);
			driveRightBack.move(right);
			driveLeftFront.move(left);
			driveLeftBack.move(left);
		}




	//	std::cout << gyro.get() << std::endl;

		strafe.move(master.get_analog(ANALOG_RIGHT_X));

		if (master.get_digital(DIGITAL_R1)) {
			intake.move_velocity(600);
			// This is 100 because it's a 100rpm motor
		}
		else if (master.get_digital(DIGITAL_R2)) {
			intake.move_velocity(-600);
		}
		else {
			intake.move_velocity(0);
		}

		if(master.get_digital_new_press(DIGITAL_Y)) {
			autonomous();
		}



		pros::delay(20);
	}
}
