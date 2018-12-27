#include "main.h"
#ifndef global
#include "global.h"
#endif
#include "okapi/api.hpp"

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
void autonomous() {

   okapi::ChassisControllerPID driveChassis = okapi::ChassisControllerFactory::create(
      {-DRIVE_LEFT_BACK_PORT, -DRIVE_LEFT_FRONT_PORT}, {DRIVE_RIGHT_BACK_PORT, DRIVE_RIGHT_FRONT_PORT},
      okapi::IterativePosPIDController::Gains{0.0028, 0, 10},
      okapi::IterativePosPIDController::Gains{0.0025, 0, 20},
      okapi::AbstractMotor::gearset::green
   );





   pros::lcd::initialize();

   driveChassis.moveDistance(1000);
   pros::lcd::print(0, "After: %f", driveRightFront.get_position());
   std::cout << driveRightFront.get_position()  << std::endl;



}
