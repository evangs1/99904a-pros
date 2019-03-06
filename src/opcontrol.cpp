#include "main.h"
#include <iostream>
#ifndef global
#include "global.h"
#endif

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
		 //std::cout << catapultIsPrimed << "    " << potCatapult.get_value() << std::endl;
		  if(master.get_digital(DIGITAL_X)) {
 				aborted = !aborted;
				catapult.move(0);
				catapult2.move(0);
				pros::delay(400);
 			}
      if (!catapultIsPrimed && !aborted) {
				catapult.move(127);
				catapult2.move(-127);
      }
			if(catapultIsPrimed) {
				catapult.move(0);
				catapult2.move(0);
				catapult.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
				catapult2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			}
      if(master.get_digital(DIGITAL_L2) && catapultIsPrimed) {
			catapult.move(127);
			catapult2.move(-127);
         pros::delay(400);
      }
		catapultIsPrimed = !(potCatapult.get_value() < 3050);
      pros::delay(20);
	}
}





void opcontrol() {
	pros::Task task_catapult_automation(catapultAutomation);
	pros::ADIDigitalIn pin1 (1);
	int strafeTimer = 0;
	bool broke = false;
	while (true) {
		int turn = master.get_analog(ANALOG_LEFT_X);
		if (master.get_analog(ANALOG_RIGHT_X) > 2) {
			turn = turn + 11;
		} else if (master.get_analog(ANALOG_RIGHT_X) < -1) {
			turn = turn - 11;
		}
		int power = master.get_analog(ANALOG_LEFT_Y);
		int left = power + turn;
		int right = power - turn;
		//change to move_velocity?



		if (abs(turn) + abs(power) < 1 && !broke) {
			driveRightFront.move(0);
			driveRightBack.move(0);
			driveLeftFront.move(0);
			driveLeftBack.move(0);
			driveRightFront.move_velocity(0);
			driveRightBack.move_velocity(0);
			driveLeftFront.move_velocity(0);
			driveLeftBack.move_velocity(0);
			driveRightFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			driveRightBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			driveLeftFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			driveLeftBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
			broke = true;
		} else {
			driveRightFront.move(right);
			driveRightBack.move(right);
			driveLeftFront.move(left);
			driveLeftBack.move(left);
			broke = false;
		}


		//pros::lcd::print(0, "Pin1: %d\n", pin1.get_value());
		//pros::lcd::print(0, "Auton: %d\n", getAutonNumber());

	//	std::cout << gyro.get() << std::endl;
		if(abs(master.get_analog(ANALOG_RIGHT_X)) > 1){
			strafe.move(master.get_analog(ANALOG_RIGHT_X));
		} else {
			strafe.move(0);
			strafe.move(0);
			strafe.move(0);
			strafe.move(0);
			strafe.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
		}


		if (master.get_digital(DIGITAL_R1)) {
			intake.move_velocity(-600);
			// This is 100 because it's a 100rpm motor
		}
		else if (master.get_digital(DIGITAL_R2)) {
			intake.move_velocity(600);
		}
		else {
			intake.move_velocity(0);
		}

		if(master.get_digital_new_press(DIGITAL_Y)) {
			autonomous();
		}

		switch(getAutonNumber()) {
		  case 0:
		    pros::lcd::print(0, "Auton: RED FRONT PARK");
				break;
		  case 1:
		    pros::lcd::print(0, "Auton: RED BACK MID PARK");
				break;
		  case 2:
		    pros::lcd::print(0, "Auton: BLUE FRONT PARK");
				break;
		  case 3:
		    pros::lcd::print(0, "Auton: BLUE BACK MID PARK");
				break;
		  case 4:
		    pros::lcd::print(0, "Auton: RED FRONT NOPARK");
				break;
			case 5:
				pros::lcd::print(0, "Auton: BLUE FRONT NOPARK");
				break;
			case 6:
				pros::lcd::print(0, "Auton: PROG SKILLS 1");
				break;
			case 7:
				pros::lcd::print(0, "Auton: PROG SKILLS 2");
				break;
		}

		pros::lcd::print(1, "Strafe %f", strafe.get_temperature() );
		pros::lcd::print(2, "Gyro %f", gyroOutput );
		pros::lcd::print(3, "Gyroadj %f", gyroOutput / 1.061 );
		pros::lcd::print(4, "Gyroraw %f", gyro.get() );
		pros::delay(20);
	}
}
