#include "main.h"
#include "pros/motors.hpp"

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

#define DRIVE_RIGHT_FRONT_PORT 1
#define DRIVE_RIGHT_BACK_PORT 3
#define DRIVE_LEFT_FRONT_PORT 2
#define DRIVE_LEFT_BACK_PORT 4
#define INTAKE_PORT 7

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	pros::Motor driveRightFront(DRIVE_RIGHT_FRONT_PORT, pros::E_MOTOR_GEARSET_18, false);
	pros::Motor driveRightBack(DRIVE_RIGHT_BACK_PORT, pros::E_MOTOR_GEARSET_18, false);
	pros::Motor driveLeftFront(DRIVE_LEFT_FRONT_PORT, pros::E_MOTOR_GEARSET_18, true);
	pros::Motor driveLeftBack(DRIVE_LEFT_BACK_PORT, pros::E_MOTOR_GEARSET_18, true);
	pros::Motor intake(INTAKE_PORT);

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);

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
