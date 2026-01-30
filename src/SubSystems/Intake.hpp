#pragma once

#include "api.h"

class Intake {
public:
  Intake(pros::Motor& roller_5, pros::Motor& roller_6, pros::Motor& roller_7);
  
  /// @brief Legacy autonomous control with boolean flags
  void telOP(bool intake, bool scoreTop, bool scoreMid, bool outtake);
  
  /// @brief Driver control using controller button states directly
  /// @param r1 R1 button - center roller forward only
  /// @param r2 R2 button - all rollers forward (intake)
  /// @param l2 L2 button - all rollers reverse (outtake)
  /// @param leftArrow Left arrow - mid score (left/right reverse, center forward)
  void update(bool r1, bool r2, bool l2, bool leftArrow);
  
  /// @brief Stop all intake motors
  void stop();

private:
  pros::Motor& left;   // roller_5
  pros::Motor& right;  // roller_6
  pros::Motor& center; // roller_7
  
  static constexpr int FULL_POWER = 127;
};