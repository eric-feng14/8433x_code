#include "Intake.hpp"

/*
roller_5 -> left
roller_6 -> right
roller_7 -> center

Button mappings (in opcontrol):
  X (held)  -> scoreMid: score middle goal
  L1 (held) -> intake: stack/intake
  L2 (held) -> scoreTop: score top goal + activates hood
  R1 (held) -> descore: descore from matchload
*/

Intake::Intake(pros::Motor& roller_5, pros::Motor& roller_6, pros::Motor& roller_7)
    : left(roller_5), right(roller_6), center(roller_7) {}

void Intake::telOP(bool intake, bool scoreTop, bool scoreMid, bool descore) {
  if (descore) { //score low
    // R1: Descore everything from matchload
    left.move_velocity(300);   // roller_5 positive
    right.move_velocity(600);
    center.move_velocity(600);
  } else if (intake) {
    // L1: Stack/intake
    left.move_velocity(-600);    // roller_5 negative
    right.move_velocity(0);
    center.move_velocity(0);
  } else if (scoreTop) { 
    // L2: Score top goal + hood
    left.move_velocity(-600);    // roller_5 negative
    right.move_velocity(-600);   // roller_6 negative
    center.move_velocity(-600);  // roller_7 negative
  } else if (scoreMid) { //confirmed to be working
    // X: Score middle goal
    left.move_velocity(-600);    // roller_5 negative
    right.move_velocity(-600);   // roller_6 negative
    center.move_velocity(600);   // roller_7 positive
  } else {
    // Stop all rollers
    left.move_velocity(0);
    right.move_velocity(0);
    center.move_velocity(0);
  }
}
