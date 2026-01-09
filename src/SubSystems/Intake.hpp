#pragma once

#include "api.h"

class Intake {
public:
  Intake(pros::Motor LEFT, pros::Motor RIGHT, pros::adi::DigitalOut BOT,
         pros::adi::DigitalOut TOP);
  void telOP(bool intake, bool scoreTop, bool scoreMid, bool outtake);

private:
  pros::Motor left;
  pros::Motor right;
  pros::adi::DigitalOut bot;
  pros::adi::DigitalOut top;
};