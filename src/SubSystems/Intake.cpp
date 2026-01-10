#include "Intake.hpp"

/*
roller_5 -> left
roller_6 -> right
roller_7 -> center
*/

Intake::Intake(pros::Motor& roller_5, pros::Motor& roller_6, pros::Motor& roller_7)
    : left(roller_5), right(roller_6), center(roller_7) {}

void Intake::telOP(bool intake, bool scoreTop, bool scoreMid, bool outtake) {
  if (outtake) {
    left.move_velocity(600);   // roller_5 positive
    right.move_velocity(0);
    center.move_velocity(0);
  } else if (intake) {
    left.move_velocity(-600);    // roller_5 neative
    right.move_velocity(0);
    center.move_velocity(0);
  } else if (scoreTop) {
    left.move_velocity(-600);    // roller_5 negative
    right.move_velocity(-600);  // roller_6 negative
    center.move_velocity(-600); // roller_7 negative
  } else if (scoreMid) {
    left.move_velocity(-600);    // roller_5 negative
    right.move_velocity(-600);  // roller_6 negative
    center.move_velocity(600); // roller_7 positive
  } else {
    left.move_velocity(0);
    right.move_velocity(0);
    center.move_velocity(0);
  }
}
