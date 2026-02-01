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

/*driver
roller_5 -> left
roller_6 -> right
piston1 -> bot
piston2 -> top
*/

// Pneumatics
pros::adi::DigitalOut piston1('A'); //hood 
pros::adi::DigitalOut piston2('B'); //wing
// Additional piston on port C
pros::adi::DigitalOut piston3('C'); //matchloader

Intake intake(roller_5, roller_6, roller_7);

constexpr bool REV_LEFT_DRIVE  = false;
constexpr bool REV_RIGHT_DRIVE = false;


constexpr bool REV_ROLLER_5 = true;
constexpr bool REV_ROLLER_6 = true;
constexpr bool REV_ROLLER_7 = false;


// START STATES (piston2 starts retracted)
bool piston1Extended = false;
bool piston2Extended = false;

// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Rotation sensor for vertical tracking wheel (port 11)
// If tracking counts backwards, flip sign (use -11).
pros::Rotation vertRot(11);

// IMU (port 20)
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
  roller_5.move(maybeRev(power, REV_ROLLER_5));
  roller_6.move(maybeRev(power, REV_ROLLER_6));
}

static inline void setRoller5(int power) {
  roller_5.move(maybeRev(power, REV_ROLLER_5));
}

static inline void setRoller6(int power) {
  roller_6.move(maybeRev(power, REV_ROLLER_6));
}

static inline void setRoller7(int power) {
  roller_7.move(maybeRev(power, REV_ROLLER_7));
}


// ============================================================
// LEMLIB ODOM SETUP (1 vertical tracking wheel + IMU heading)
// ============================================================

constexpr auto TRACK_WHEEL_TYPE = lemlib::Omniwheel::NEW_2;
constexpr float VERT_OFFSET_IN = -1.125; // <-- measure later

lemlib::TrackingWheel verticalTrack(&vertRot, TRACK_WHEEL_TYPE, 0);

// Create a distance sensor on port 10
pros::Distance front_dist(11);
pros::Distance left_dist(13);

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
lemlib::ControllerSettings lateral_controller(7.5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              15, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              10 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(1.25, // proportional gain (kP)
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

// Other autonomous routines disabled — using only `autonomous()` below
#if 0
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

  // Activate piston B for middle goal match load and score
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

  // Activate piston B for long goal match load and score
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
#endif
void tuningtest() {
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 5000);
    // chassis.moveToPoint(0, 48, 5000, {.forwards = true}); // turn to face heading 90 with a very long timeout
    pros::delay(3500);
     auto pose = chassis.getPose();
            std::cout << "[DEBUG] X: " << pose.x 
                      << " Y: " << pose.y 
                      << " Theta: " << pose.theta << std::endl;

}

void leftSideAuton() { // eric
    chassis.setPose(0, 0, 0);
    intake.telOP(true, false, false, false); // keep intake on for start

    chassis.moveToPoint(-13.60, 47.6518, 3000, {.maxSpeed = 40});
    pros::delay(1000);
    intake.telOP(false, false, false, false); // turn off intake

    chassis.moveToPose(5.20893, 55.1891,-130, 3000, {.forwards = false, .maxSpeed = 40});
    intake.telOP(false, false, true, false); // score intaked balls into mid
    pros::delay(1500);

    chassis.moveToPoint(-30.3277, 10.443, 3000, {.maxSpeed = 40});
    chassis.turnToHeading(180, 2000);
    chassis.moveToPoint(-30.6277, -2.36389, 3000, {.maxSpeed = 40});
    intake.telOP(true, false, false, false);
    // piston1.set_value(true);
    // piston2.set_value(true);

    chassis.moveToPoint(-30.687, 48.551, 3000, {.forwards = false, .maxSpeed = 40});
    intake.telOP(false, true, false, false);
}
// ============================================================
// AUTON (WAYPOINTS)
// ============================================================



// void KennyAuton(){
//     chassis.setPose(0, 0, 0);
//     // doinker.set_value(true);
//     // piston3.set_value(false);
//     piston2.set_value(false); // wing piston out
//     intake.telOP(true, false, false, false); // keep intake on
//     chassis.moveToPose(-1.5, 30.3, -11.4, 2000, {.minSpeed = 50}, false);
//     pros::delay(300);
//     chassis.turnToHeading(-135, 1000);
//     chassis.moveToPose(-1.5, 55.8, -135, 3000, {.forwards=false, .minSpeed = 60}, false);
//     // intake.telOP(false, false, true, false, false, false);
//     intake.telOP(false, false, true, false); // score mid

//     pros::delay(1200);
//     // intake.telOP(true, false, false, false, false, false);
//     intake.telOP(true, false, false, false); // intake

//     pros::delay(200);
//     chassis.moveToPoint(-30.8, 11.3, 2000);
//     pros::delay(200);
//     chassis.turnToHeading(180, 1000);
//     // tongue.set_value(true);
//     piston3.set_value(true); // matchloader piston out

//     piston3.set_value(true); //
//     piston1.set_value(true); //hopefully its the hood
//     chassis.moveToPoint(-27.3, -3.9, 1700, {.maxSpeed = 40});
//     chassis.moveToPoint(-35.8, 26.9, 1000, {.forwards=false, .maxSpeed = 80}, false);
//     // intake.telOP(false, true, false, false, false, false); // bro wth does this do
//     intake.telOP(false, true, false, false); // score top long

//     pros::delay(2000);
//     // tongue.set_value(false);
//     // piston3.set_value(true); // matchloader piston in

//     chassis.moveToPoint(-30.9, 11.6, 1000, {.minSpeed = 60}, false);
//     chassis.turnToHeading(70, 1000);
//     chassis.moveToPoint(-18.3, 18.2, 1000, {.forwards = false});
//     chassis.turnToHeading(150, 1000);
//     // doinker.set_value(false);
//     piston1.set_value(false); // wing piston in

//     chassis.moveToPoint(-36.1, 43.7, 3000, {.forwards=false, .maxSpeed = 60}, false);
// }

void intakeTest() {
  intake.telOP(false, false, true, false); //score mid testing
}

// void skills() {
//   KennyAuton();
  
// }

void oldKennyAuton() {
    chassis.setPose(0, 0, 0);
    // doinker.set_value(true);
    // piston3.set_value(false);
    piston2.set_value(false); // wing piston out
    intake.telOP(true, false, false, false); // keep intake on
    chassis.moveToPoint( -9, 37, 2000, {.minSpeed = 35}, false);

    pros::delay(300);
    chassis.turnToHeading(-136, 1000); // fix
    chassis.moveToPose(4, 41, -133, 3000,{.forwards=false, .minSpeed = 60}, false);
    // intake.telOP(false, false, true, false, false, false);
    intake.telOP(false, false, true, false); // score mid

      chassis.moveToPoint( 4.5, 45, 2000, {.forwards = false, .minSpeed = 50}, false);
       chassis.turnToHeading(-138, 1000); // fix
    pros::delay(4000);
    // intake.telOP(true, false, false, false, false, false);
    intake.telOP(true, false, false, false); // intake

    pros::delay(200);
    chassis.moveToPoint(-29, 8, 3000, {.maxSpeed = 60});
    pros::delay(200);
    chassis.turnToHeading(180, 1000);
    // tongue.set_value(true);
    piston3.set_value(true); // matchloader piston out
    piston1.set_value(true); //hopefully its the hood
    chassis.moveToPoint(-29.5, -10, 2000, {.minSpeed = 40});
    chassis.moveToPoint(-32, 48, 3000, {.forwards=false, .maxSpeed = 40}, false);
    // intake.telOP(false, true, false, false, false, false); // bro wth does this do
    intake.telOP(false, true, false, false); // score top long

    pros::delay(2000);
    // tongue.set_value(false);
    // piston3.set_value(true); // matchloader piston in

    // chassis.moveToPoint(-34, 17, 1000, {.minSpeed = 60}, false);
    // chassis.turnToHeading(130, 1000);
    // chassis.moveToPoint(-44, 22, 1000, {.forwards = false});
    // chassis.turnToHeading(180, 1000);
    // doinker.set_value(false);
//     piston1.set_value(false); // wing piston in

//     chassis.moveToPoint(-44, 45, 3000, {.forwards=false, .maxSpeed = 60}, false);

}


void autonomous() {
    // leftSideAuton();
    // oldKennyAuton();
    
    chassis.setPose(0, 0, 0);
    // doinker.set_value(true);
    // piston3.set_value(false);
    piston2.set_value(false); // wing piston out
    intake.telOP(true, false, false, false); // keep intake on
    chassis.moveToPose( -12, 37, -21, 2000, {.minSpeed = 40}, false);
    pros::delay(300);
    chassis.turnToHeading(-131, 1000); // fix
    chassis.moveToPose(7, 44, -131, 1600,{.forwards=false, .minSpeed = 60}, false);
    // intake.telOP(false, false, true, false, false, false);
    intake.telOP(false, false, true, false); // score mid

    pros::delay(400);
    // intake.telOP(true, false, false, false, false, false);
    intake.telOP(true, false, false, false); // intake

    pros::delay(200);
    chassis.moveToPoint(-30, 8, 2000);
    pros::delay(200);
    chassis.turnToHeading(180, 1000);
    // tongue.set_value(true);
    piston3.set_value(true); // matchloader piston out

    piston3.set_value(true); //
    piston1.set_value(true); //hopefully its the hood
    chassis.moveToPoint(-30, -20, 1700, {.maxSpeed = 40});
    chassis.moveToPoint(-30, 30, 1000, {.forwards=false,.maxSpeed = 80}, false);
    // intake.telOP(false, true, false, false, false, false); // bro wth does this do
    intake.telOP(false, true, false, false); // score top long

    pros::delay(2000);
    // tongue.set_value(false);
    // piston3.set_value(true); // matchloader piston in

    // chassis.moveToPoint(-34, 17, 1000, {.minSpeed = 60}, false);
    // chassis.turnToHeading(130, 1000);
    // chassis.moveToPoint(-44, 22, 1000, {.forwards = false});
    // chassis.turnToHeading(180, 1000);
    // doinker.set_value(false);
//     piston1.set_value(false); // wing piston in

//     chassis.moveToPoint(-44, 45, 3000, {.forwards=false, .maxSpeed = 60}, false);
    
    // intakeTest();
    // tuningtest();

}

// ============================================================
// DRIVER CONTROL
// ============================================================

void opcontrol() {
  // Use global `piston1Extended`/`piston2Extended`; keep `piston3Extended`
  // local to opcontrol to manage the extra piston.
  static bool piston3Extended = false;

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
        bool scoreMidBtn = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);      // X: score middle goal
        bool intakeBtn = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);       // L1: stack/intake
        bool scoreTopBtn = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);     // L2: score top goal + hood
        bool descoreBtn = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);      // R1: descore from matchload

        if (scoreMidBtn) {
          // Score middle goal
          setRoller5(127);
          setRoller6(127);
          setRoller7(127);
        } else if (intakeBtn || scoreTopBtn) {
          // L1: stack/intake, L2: score top goal (activates hood)
          setRoller5(127);
          setRoller6(127);
          setRoller7(-127);
        } else if (descoreBtn) {
          // Descore everything from matchload
          setRoller5(-127);
          setRoller6(-127);
          setRoller7(127);
        } else {
          // Stop all rollers
          setRoller5(0);
          setRoller6(0);
          setRoller7(0);
        }
        */

        // 3. PISTON CONTROL (Toggles)
        // .get_digital_new_press() returns true only on the initial press, 
        // removing the need for 'lastA' or 'lastB' variables.
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            piston1Extended = !piston1Extended;
            piston1.set_value(piston1Extended);
        }

        // Toggle piston on port B when R2 is pressed
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
          piston2Extended = !piston2Extended;
          piston2.set_value(piston2Extended);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
          piston3Extended = !piston3Extended;
          piston3.set_value(piston3Extended);
        }

        // 4. PISTON shortcuts
        // L2: hold-to-open piston A (overrides A-toggle while held)
        if (scoreTopBtn) {
          piston1.set_value(true);
        } else {
          piston1.set_value(piston1Extended);
        }

        // piston2 follows the toggled state (R2 toggles it)
        piston2.set_value(piston2Extended);

        // 4. TELEMETRY / DEBUGGING
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            auto pose = chassis.getPose();
            std::cout << "[DEBUG] X: " << pose.x 
                      << " Y: " << pose.y 
                      << " Theta: " << pose.theta << std::endl;
        }


        // Loop delay to prevent CPU hogging (50Hz)
        pros::delay(20);
    }
}