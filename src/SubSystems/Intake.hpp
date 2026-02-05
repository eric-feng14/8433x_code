#pragma once

#include "api.h"

class Intake {
public:
  Intake(pros::Motor& roller_5, pros::Motor& roller_6, pros::Motor& roller_7, pros::adi::DigitalOut& hood_piston);
  void telOP(bool intake, bool scoreTop, bool scoreMid, bool descore);

private:
  pros::Motor& left;   // roller_5
  pros::Motor& right;  // roller_6
  pros::Motor& center; // roller_7
  pros::adi::DigitalOut& hood_piston; // hood piston
  
  static constexpr int FULL_POWER = 127;
};