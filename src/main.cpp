#include "main.h"
#include "lemlib/api.hpp"
#include <cmath>
#include "SubSystems/intake.hpp"
#include "pros/motors.hpp"


using namespace std;

// ======================= PATH.JERRYIO ASSET =======================
// Keep this if you want; it will NOT affect driver.
// ASSET(test_txt);

// ============================================================
// PORTS
// ============================================================

// Drive motors
pros::MotorGroup left_motors({-1, -2, -3}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({4, 8, 9}, pros::MotorGearset::blue);

// Intake / rollers
pros::Motor roller_5(-5, pros::MotorGearset::blue);
pros::Motor roller_6(6, pros::MotorGearset::blue);
pros::Motor roller_7(7, pros::MotorGearset::blue);

/*
roller_5 -> left
roller_6 -> right
piston1 -> bot
piston2 -> top
*/

// Pneumatics
pros::adi::DigitalOut piston1('A');    
pros::adi::DigitalOut piston2('H');

Intake intake(roller_5, roller_6, roller_7);

constexpr bool REV_LEFT_DRIVE  = false;
constexpr bool REV_RIGHT_DRIVE = false;


constexpr bool REV_ROLLER_5 = false;
constexpr bool REV_ROLLER_6 = false;
constexpr bool REV_ROLLER_7 = false;


// START STATES (piston2 starts retracted)
bool piston1Extended = false;
bool piston2Extended = false;

// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Rotation sensor for vertical tracking wheel (port 11)
// If tracking counts backwards, flip sign.
pros::Rotation vertRot(10);

// IMU (port 10)
pros::Imu imu(20);

// ============================================================
// BASIC HELPERS
// ============================================================
static inline int clamp127(int v) {
    if (v > 127) return 127;
    if (v < -127) return -127;
    return v;
}


static inline int maybeRev(int power, bool rev) {
    return rev ? -power : power;
}

static inline int deadband(int v, int db = 6) {
    return (std::abs(v) <= db) ? 0 : v;
}

static inline void setRollers56(int power) {
    roller_5.move(power);
    roller_6.move(power);
}

static inline void setRoller7(int power) {
    roller_7.move(power);
}



// ============================================================
// LEMLIB ODOM SETUP (1 vertical tracking wheel + IMU heading)
// ============================================================

constexpr auto TRACK_WHEEL_TYPE = lemlib::Omniwheel::NEW_2;
constexpr float VERT_OFFSET_IN = -1.125; // <-- measure later

lemlib::TrackingWheel verticalTrack(&vertRot, TRACK_WHEEL_TYPE, 0);

lemlib::OdomSensors sensors(
    &verticalTrack,
    nullptr,
    nullptr,
    nullptr,
    &imu
);

// Drivetrain
const int MOTOR_TEETH = 36;
const int WHEEL_TEETH = 60;
const int MOTOR_CARTRIDGE_RPM_BLUE = 600;
const int DRIVETRAIN_RPM = MOTOR_CARTRIDGE_RPM_BLUE * MOTOR_TEETH / WHEEL_TEETH;

lemlib::Drivetrain drivetrain(
    &left_motors,
    &right_motors,
    12.625,
    lemlib::Omniwheel::NEW_325,
    DRIVETRAIN_RPM,
    2
);
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              10 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

// ============================================================
// PROS DEFAULTS
// ============================================================
// initialize function. Runs on program startup
// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            // pros::lcd::print(0, "Xxxx: %f", chassis.getPose().x); // x
            // pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            // pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            
            // cout << "X: " << chassis.getPose().x << " Y: " << chassis.getPose().y << " Theta: " << chassis.getPose().theta << endl;
            // delay to save resources
            pros::delay(100);
        }
    });
}

void disabled() {}
void competition_initialize() {}

void rightSideAuton() {
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    intake.telOP(true, false, false, false); // intake
    // queue balls (intake)
    chassis.moveToPose(9, 26, 24, 2000);
    chassis.turnToHeading(114, 1000);
    chassis.moveToPoint(34.73, 6, 2000);
    chassis.turnToHeading(180, 2000);
    chassis.moveToPose(34.73, -20, 180, 1500, {.maxSpeed = 150});
    chassis.moveToPose(36.73, 25, 180, 2000, {.forwards = false});
    pros::delay(1000);
    intake.telOP(false, true, false, false); // outtake
}

void leftSideAuton() {
  chassis.setPose(0, 0, 0);

  //keep piston a, "the wing" activated
  piston1.set_value(true);

  intake.telOP(true, false, false, false);
  chassis.moveToPose(-8.6, 37, -21, 2000, {.minSpeed = 50}, false);
  pros::delay(300);
  chassis.turnToHeading(-131, 1000); // fix
  chassis.moveToPose(7, 44, -131, 1500, {.forwards = false});
  roller_7.move(5);
  pros::delay(1300);

  // Activate piston H for middle goal match load and score
  intake.telOP(false, false, true, false);
  pros::delay(400);

  // intake.telOP(true, false, false, false);
  pros::delay(2000);
  chassis.moveToPoint(-34.73, 8, 2000);
  chassis.turnToHeading(180, 2000);
  piston2.set_value(true);
  chassis.moveToPoint(-36, -20, 2000, {.maxSpeed = 40});
  chassis.moveToPoint(-35.5, 30, 2000, {.forwards = false, .maxSpeed = 80},
                      false);
  piston2.set_value(false);

  // Activate piston H for long goal match load and score
  piston2.set_value(true);
  intake.telOP(false, true, false, false);
  pros::delay(2000);
  piston2.set_value(false);

  chassis.moveToPoint(-35.5, 17, 1000, {.minSpeed = 60}, false);
  chassis.moveToPoint(-35.5, 40, 1000, {.forwards = false, .minSpeed = 200},
                      false);
}

void leftside() {
  chassis.setPose(0, 0, 0);

  intake.telOP(true, false, false, false);
  chassis.moveToPose(-8.6, 37, -21, 2000, {.minSpeed = 50}, false);
  pros::delay(300);
  chassis.turnToHeading(-131, 1000); // fix
  chassis.moveToPose(7, 44, -131, 1300, {.forwards = false});
  pros::delay(1300);
  intake.telOP(false, false, true, false);
  pros::delay(400);
  intake.telOP(true, false, false, false);
  pros::delay(200);
  chassis.moveToPoint(-34.73, 8, 2000);
  chassis.turnToHeading(180, 1000);
  chassis.moveToPoint(-36, -20, 1600, {.maxSpeed = 40});
  chassis.moveToPoint(-35.5, 30, 1000, {.forwards = false, .maxSpeed = 80},
                      false);
  intake.telOP(false, true, false, false);
  pros::delay(2000);
  chassis.moveToPoint(-35.5, 17, 1000, {.minSpeed = 60}, false);
  chassis.moveToPoint(-35.5, 40, 1000, {.forwards = false, .minSpeed = 200},
                      false);
}

void long_goal_score(bool active) {
  if (active) {
    intake.telOP(false, true, false, false);
  } else {
    intake.telOP(false, false, false, false);
  }
}

void middle_goal_score(bool active) {
  if (active) {
    intake.telOP(false, false, true, false);
  } else {
    intake.telOP(false, false, false, false);
  }
}

void matchload_activate(bool active) {
  if (active) {
    intake.telOP(true, false, false, false);
  } else {
    intake.telOP(false, false, false, false);
  }
}

void skills() {
  chassis.setPose(0, 0, 0);
  
  // Point 1: move forward
  chassis.moveToPoint(-0.08, 22.48, 3000);
  
  // Point 2: turn in place
  chassis.turnToHeading(-103.22, 1500);
  
  // Point 3: move
  chassis.moveToPoint(-25.80, 16.09, 3000);
  
  // Point 4: turn in place
  chassis.turnToHeading(-186.07, 1500);
  
  // Point 5: move
  chassis.moveToPoint(-23.79, -8.58, 3000);
  
  // Point 6: turn in place
  chassis.turnToHeading(-83.48, 1500);
  
  // Point 7: move
  chassis.moveToPoint(-51.63, -7.29, 3000);
  
  // Point 8: move
  chassis.moveToPoint(-76.47, -9.37, 3000);
  
  // Point 9: move
  chassis.moveToPoint(-89.25, -6.79, 3000);
  
  // Point 10: move
  chassis.moveToPoint(-104.23, 13.31, 4000);
  
  // Point 11: turn in place
  chassis.turnToHeading(7.97, 1500);
  
  // Point 12: move
  chassis.moveToPoint(-102.34, 28.36, 2000);
  
  // Point 13: move
  chassis.moveToPoint(-100.12, -4.18, 3000);
  
  // Point 14: move
  chassis.moveToPoint(-100.55, 16.77, 3000);
  
  // Point 15: move
  chassis.moveToPoint(-59.07, 35.15, 4000);
}


// ============================================================
// AUTON (WAYPOINTS)
// ============================================================
void autonomous() {

    // skills();
    // leftSideAuton();
    leftside();

    // chassis.setPose(0, 0, 0); // or 0  
    // chassis.moveToPoint(0, 48, 100000); // turn to face heading 90 with a very long timeout

    
    // Keep intake running upwards throughout auton
    // setRollers56(127);
    // setRoller7(127);
    
    // chassis.moveToPoint(0, 0, 5000);
    // chassis.moveToPoint(0, 24.76, 5000);
    // chassis.moveToPoint(-23.109, -18.629, 5000);
    // chassis.moveToPoint(-23.109, 22.637, 5000);
    // chassis.moveToPoint(-47.171, 22.057, 5000); //initial starting position

    // chassis.moveToPoint(-22.647, 21.822, 5000);
    // chassis.turnToHeading(300, 5000;
    // chassis.moveToPoint(-60.612, 46.11, 5000);
    // chassis.turnToHeading(90, 5000);
    // chassis.moveToPoint(-23.118, 46.817, 5000);


    // left_motors.move(0);
    // right_motors.move(0);

    //eddie code
    // chassis.setPose(-47.407, -22.392, 90);
    // chassis.moveToPoint(-22.175, -22.628, 5000);
    // chassis.moveToPoint(-68.393, -46.916, 5000);
    // chassis.moveToPoint(-24.769, -47.152, 5000);

    //madhav
    //chassis.moveToPoint(-66.035, -0.462, 1000);
    // chassis.moveToPoint(-57.546, -0.462, 1000);
    // chassis.moveToPoint(-23.59, 25.241, 1000);
    // chassis.moveToPoint(-10.385, 10.621, 1000);
    // chassis.moveToPoint(-46.935, 46.935, 5000);
    // chassis.moveToPoint(-66.978, 46.463, 5000);
    // chassis.moveToPoint(-27.835, 63.677, 5000);
    // chassis.moveToPoint(41.492, 63.441, 5000);
    // chassis.moveToPoint(41.964, 47.407, 5000);
    // chassis.moveToPoint(29.231, 46.699, 5000);
    // chassis.moveToPoint(66.488, 47.642, 5000);
    // chassis.moveToPoint(28.287, 45.992, 5000);

}

// ============================================================
// DRIVER CONTROL
// ============================================================

pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {
    // State variables for pistons (if not defined globally)
    static bool piston1Extended = false;
    static bool piston2Extended = false;

    while (true) {
        // 1. DRIVE CONTROL (Arcade Drive)
        // Using Left Y for forward/backward and Right X for turning
        int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn    = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        int leftPower  = clamp127(forward + turn);
        int rightPower = clamp127(forward - turn);

        left_motors.move(maybeRev(leftPower, REV_LEFT_DRIVE));
        right_motors.move(maybeRev(rightPower, REV_RIGHT_DRIVE));

        // 2. ROLLER CONTROL (Motors 5, 6, and 7)
        bool r1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool r2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool l2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        // Logic for Rollers 5 & 6
        int power56 = 0;
        if (r2 && !l2) power56 = 127;  // Forward
        else if (!r2 && l2) power56 = -127; // Reverse        
        setRollers56(power56);

        // Logic for Roller 7 (Prioritizes R1, then follows R2/L2)
        int power7 = 0;
        if (r1) {
            power7 = 127;
        } else if (r2 && !l2) {
            power7 = 127;
        } else if (l2 && !r2) {
            power7 = -127;
        } else {
            power7 = 0;
        }
        setRoller7(power7);

        // 3. PISTON CONTROL (Toggles)
        // .get_digital_new_press() returns true only on the initial press, 
        // removing the need for 'lastA' or 'lastB' variables.
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            piston1Extended = !piston1Extended;
            piston1.set_value(piston1Extended);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            piston2Extended = !piston2Extended;
            piston2.set_value(piston2Extended);
        }

        // 4. TELEMETRY / DEBUGGING
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            auto pose = chassis.getPose();
            std::cout << "[DEBUG] X: " << pose.x 
                      << " Y: " << pose.y 
                      << " Theta: " << pose.theta << std::endl;
        }

        // Loop delay to prevent CPU hogging (50Hz)
        pros::delay(20);
    }
}

