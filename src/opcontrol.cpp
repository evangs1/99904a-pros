
#include <iostream>
#include "define.hpp"

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

/*
 * Motor Definition Section
 */




void opcontrol() {


	while (true) {
		//pros::lcd::print("potentiometer: %d", potCatapult.get_value());
		std::cout << "Pot value: " << potCatapult.get_value() << std::endl;
		int turn = master.get_analog(ANALOG_LEFT_X);
		int power = master.get_analog(ANALOG_LEFT_Y);
		int left = power + turn;
		int right = power - turn;
		driveRightFront.move(right);
		driveRightBack.move(right);
		driveLeftFront.move(left);
		driveLeftBack.move(left);

		if (master.get_digital(DIGITAL_R1)) {
      intake.move_velocity(600); // This is 100 because it's a 100rpm motor
    }
    else if (master.get_digital(DIGITAL_R2)) {
      intake.move_velocity(-600);
    }
    else {
      intake.move_velocity(0);
    }

		pros::delay(20);
	}
}
