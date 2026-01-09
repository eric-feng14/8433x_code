#include "Intake.hpp"

/*
roller_5 -> left
roller_6 -> right
piston1 -> bot
piston2 -> top
*/

Intake::Intake(pros::Motor roller_5, pros::Motor roller_6, pros::adi::DigitalOut piston1,
               pros::adi::DigitalOut piston2)
    : left(roller_5), right(roller_6), bot(piston1), top(piston2) {}

// void Intake::telOP(bool intake, bool scoreTop, bool scoreMid, bool outtake, bool park, bool prime) {
void Intake::telOP(bool intake, bool scoreTop, bool scoreMid, bool outtake) {
  if (outtake) {
    left.move_velocity(600);
    right.move_velocity(600);
  } else if (intake) {
    left.move_velocity(-600);
    right.move_velocity(-600);
    bot.set_value(true);
    top.set_value(false);
  } else if (scoreTop) {
    bot.set_value(false);
    top.set_value(false);
    left.move_velocity(600);
    right.move_velocity(600);
  } else if (scoreMid) {
    left.move_velocity(300);
    right.move_velocity(300);
    bot.set_value(true);
    top.set_value(true);
//   } else if (prime) {
//     left.move_velocity(0);
//     right.move_velocity(0);
//   } else if (park) {
//     left.move_velocity(0);
//     right.move_velocity(0);
  } else {
    left.move_velocity(0);
    right.move_velocity(0);
  }
}
