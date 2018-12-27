#include "main.h"
#ifndef global
#include "global.h"
#endif

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
/*
void my_task_fn(void* param) {
    while(true) {
        //std::cout << "Hello" << std::endl;
        catapult.move(100);
        pros::delay(300);
    }
}
*/
/*
void catapultAutomation(void* param) {
   bool catapultIsPrimed = false;
   //std::cout << "Hello" << (char*)param << std::endl;
   while (true) {
      std::cout << potCatapult.get_value() << std::endl;
      pros::delay(300);
      if (!catapultIsPrimed) {
         while (potCatapult.get_value() < 3100) {
            catapult.move(100);
         }
         catapult.move(0);
         catapult.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
         catapultIsPrimed = true;
      }
      if(master.get_digital(DIGITAL_L2) && catapultIsPrimed) {
         catapult.move(100);
         pros::delay(400);
         catapultIsPrimed = false;
      }
      pros::delay(20);
   }
}
*/


void initialize() {
   pros::lcd::initialize();
   okapi::ADIGyro gyro(GYRO_PORT, 1);
	pros::delay(1400);
   std::string text("PROS");
	//pros::Task my_task(my_task_fn, &text);
   //pros::Task task_catapult_automation(catapultAutomation, &text);
	//pros::lcd::set_text(1, "Hello PROS User!");
	//pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}
