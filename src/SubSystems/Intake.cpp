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
    left.move(-FULL_POWER);
    right.move(-FULL_POWER);
    center.move(-FULL_POWER);
  } else if (intake) {
    left.move(FULL_POWER);
    right.move(FULL_POWER);
    center.move(FULL_POWER);
  } else if (scoreTop) { //unverified -> check later if this works, other conditions should be good now
    left.move(-FULL_POWER);
    right.move(-FULL_POWER);
    center.move(-FULL_POWER);
  } else if (scoreMid) {
    left.move(-FULL_POWER);   // reverse
    right.move(-FULL_POWER);  // reverse
    center.move(FULL_POWER);  // forward
  } else {
    stop();
  }
}

void Intake::update(bool r1, bool r2, bool l2, bool leftArrow) {
  // Priority: leftArrow (mid score) > r1 (center only) > r2/l2 (intake/outtake)
  
  if (leftArrow) {
    // Mid score mode: left & right reverse, center forward
    left.move(-FULL_POWER);
    right.move(-FULL_POWER);
    center.move(FULL_POWER);
    return;
  }
  
  // Left & Right rollers: controlled by R2 (forward) or L2 (reverse)
  int power56 = 0;
  if (r2 && !l2) {
    power56 = FULL_POWER;   // Forward (intake)
  } else if (l2 && !r2) {
    power56 = -FULL_POWER;  // Reverse (outtake)
  }
  left.move(power56);
  right.move(power56);
  
  // Center roller: R1 takes priority, then follows R2/L2
  int power7 = 0;
  if (r1) {
    power7 = FULL_POWER;      // R1 forces center forward
  } else if (r2 && !l2) {
    power7 = FULL_POWER;      // Follow intake
  } else if (l2 && !r2) {
    power7 = -FULL_POWER;     // Follow outtake
  }
  center.move(power7);
}

void Intake::stop() {
  left.move(0);
  right.move(0);
  center.move(0);
}
