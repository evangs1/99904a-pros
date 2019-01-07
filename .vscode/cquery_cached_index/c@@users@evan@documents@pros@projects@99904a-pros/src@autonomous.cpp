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
void autonomous() {
   /*
   okapi::ChassisControllerPID driveChassis = okapi::ChassisControllerFactory::create(
      {-DRIVE_LEFT_BACK_PORT, -DRIVE_LEFT_FRONT_PORT}, {DRIVE_RIGHT_BACK_PORT, DRIVE_RIGHT_FRONT_PORT},
      okapi::IterativePosPIDController::Gains{0.0028, 0, 10},
      okapi::IterativePosPIDController::Gains{0.0025, 0, 20},
      okapi::AbstractMotor::gearset::green
   );
   */

   //pros::lcd::initialize();

   //driveChassis.moveDistance(1000);
   //pros::lcd::print(0, "After: %f", driveRightFront.get_position());
   //std::cout << driveRightFront.get_position()  << std::endl;

   /*
   okapi::ADIGyro gyro(GYRO_PORT, 1);
   auto controller = okapi::AsyncControllerFactory::posPID({-DRIVE_LEFT_BACK_PORT, -DRIVE_LEFT_FRONT_PORT, -DRIVE_RIGHT_BACK_PORT, -DRIVE_RIGHT_FRONT_PORT}, gyro, 100000000000, 1, 0.005);

   controller.setTarget(400);
   controller.waitUntilSettled();
*/
   //double tunerOutput;
   //auto tuner = okapi::PIDTunerFactory::create(0, tunerOutput, 4000_ms, 900, 0.0001, 0.1, 0, 1, 0.00001, 0.1, 10, 16);


/*
   auto gyroTurnController = okapi::IterativeControllerFactory::posPID(0.0018, 0.0, 1000, 0, std::make_unique<okapi::AverageFilter<3>>());

   //configure motors
   okapi::Motor rf(-DRIVE_RIGHT_FRONT_PORT);
   okapi::Motor rb(-DRIVE_RIGHT_BACK_PORT);
   okapi::Motor lf(-DRIVE_LEFT_FRONT_PORT);
   okapi::Motor lb(-DRIVE_LEFT_BACK_PORT);

   //set sample time and target
   gyroTurnController.setSampleTime(10_ms);

   gyroTurnController.setTarget(400);

   //run loop
   while (!gyroTurnController.isSettled()) {
      double newInput = gyro.get();
      double newOutput = gyroTurnController.step(newInput);
      rf.move(newOutput*127);
      rb.move(newOutput*127);
      lf.move(newOutput*127);
      lb.move(newOutput*127);


      pros::delay(10);
   }
*/


   MiniPID pid = MiniPID(0.21, 0.000, 0.5);
   pid.setOutputLimits(-127, 127);
   pid.setMaxIOutput(30);
   pid.setSetpointRange(900);

   double target = 900;
   int iterations = 0;

   while(iterations < 2000) {
      double output = pid.getOutput(gyro.get() / 1.075, target);
      driveRightFront.move(-output);
      driveRightBack.move(-output);
      driveLeftFront.move(output);
      driveLeftBack.move(output);
      pros::delay(10);
      std::cout << gyro.get() << std::endl;
      pros::lcd::print(0, "Gyro: %f\n", (gyro.get() / 1.075) -900);
      pros::lcd::print(1, "PID : %f\n", output);
      iterations = iterations + 10;
   }





}
