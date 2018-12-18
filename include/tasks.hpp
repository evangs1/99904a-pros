
#include "define.hpp"

void catapultAutomation() {
 bool catapultIsPrimed = false;

 while (true) {
   if (!catapultIsPrimed) {

     while (potCatapult.get_value() < 3100) {
      catapult.move(100);
     }
      //catapult.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
      catapultIsPrimed = true;
   }

   //if ()
 }
}
