#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

pros::MotorGroup
    left_motors({-18, -19, -20},
                pros::MotorGearset::blue); // left motors use 600 RPM cartridges
pros::MotorGroup right_motors(
    {1, 2, 3}, pros::MotorGearset::blue); // right motors use 200 RPM cartridges
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors,               // left motor group
                              &right_motors,              // right motor group
                              11.25,                      // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2    // horizontal drift is 2 (for now)
);
// create an imu on port 10
pros::Imu imu(10);

pros::Motor intake(6);
pros::Motor hook(-11);

pros::ADIDigitalOut clamp('H');

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve
    throttle_curve(3,    // joystick deadband out of 127
                   10,   // minimum output where drivetrain will move out of 127
                   1.019 // expo curve gain
    );

// input curve for steer input during driver control
lemlib::ExpoDriveCurve
    steer_curve(3,    // joystick deadband out of 127
                10,   // minimum output where drivetrain will move out of 127
                1.019 // expo curve gain
    );

// lateral motion controller
// lemlib::ControllerSettings
//     lateral_controller(10,  // proportional gain (kP)
//                        0,   // integral gain (kI)
//                        3,   // derivative gain (kD)
//                        3,   // anti windup
//                        1,   // small error range, in inches
//                        100, // small error range timeout, in milliseconds
//                        3,   // large error range, in inches
//                        500, // large error range timeout, in milliseconds
//                        20   // maximum acceleration (slew)
//     );

// // angular motion controller
// lemlib::ControllerSettings
//     angular_controller(2,   // proportional gain (kP)
//                        0,   // integral gain (kI)
//                        10,  // derivative gain (kD)
//                        3,   // anti windup
//                        1,   // small error range, in degrees
//                        100, // small error range timeout, in milliseconds
//                        3,   // large error range, in degrees
//                        500, // large error range timeout, in milliseconds
//                        0    // maximum acceleration (slew)
//     );

lemlib::OdomSensors sensors(
    nullptr, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a
             // second one
    &imu);   // inertial sensor

// create the chassis (no custom PID settings)
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        sensors,    // odometry sensors (left, middle, right)
                        &throttle_curve, &steer_curve);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "working");
  } else {
    pros::lcd::clear_line(2);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();  // initialize brain screen
  chassis.calibrate();      // calibrate sensors
  chassis.setPose(0, 0, 0); // Set initial pose after calibration
  // print position to brain screen
  pros::Task screen_task([&]() {
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
      // delay to save resources
      pros::delay(20);
    }
  });
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
  pros::lcd::set_text(0, "AUTON START");

  // Ensure starting pose is defined for LemLib (x, y in inches, theta deg)
  chassis.setPose(0, 0, 0);

  // Run intake while driving to the target point
  intake.move(127);
  chassis.moveToPoint(0, 50, 3000, {.forwards = true});
  intake.move(0);

  // Then drive forward using direct motor control for 2 seconds
  left_motors.move(100);
  right_motors.move(100);
  pros::delay(2000);

  left_motors.move(0);
  right_motors.move(0);

  pros::lcd::set_text(1, "AUTON END");
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {
  static bool clamp_state = false;

  while (true) {
    // get left y and right x positions
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // move the robot
    chassis.curvature(leftY, rightX);

    // Intake control with L1 (forward) and L2 (reverse)
    if (controller.get_digital(DIGITAL_R1)) {
      intake.move(127); // Full speed forward
      hook.move(-100);  // Full speed forward
    } else if (controller.get_digital(DIGITAL_R2)) {
      hook.move(100);    // Full speed forward
      intake.move(-127); // Full speed reverse
    } else {
      hook.move(0);   // Full speed forward
      intake.move(0); // Stop if neither button is pressed
    }

    // Set solenoid based on toggled state
    clamp.set_value(clamp_state);

    // Single-action solenoid control for clamp
    static bool clamp_last_a_state = false;
    bool clamp_current_a_state = controller.get_digital(DIGITAL_A);

    // Toggle state on button press (not hold)
    if (clamp_current_a_state && !clamp_last_a_state) {
      clamp_state = !clamp_state;
    }
    clamp_last_a_state = clamp_current_a_state;

    pros::delay(20);
  }
}