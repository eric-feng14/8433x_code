#include "main.h"
#include "lemlib/api.hpp"
#include <cmath>
#include <future>
#include "SubSystems/intake.hpp"
#include "pros/distance.hpp"
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

// Pneumatics
pros::adi::DigitalOut piston1('A'); //hood 
pros::adi::DigitalOut piston2('B'); //wing
// Additional piston on port C
pros::adi::DigitalOut piston3('C'); //matchloader

Intake intake(roller_5, roller_6, roller_7, piston1);

constexpr bool REV_LEFT_DRIVE  = false;
constexpr bool REV_RIGHT_DRIVE = false;

constexpr bool REV_ROLLER_5 = true;
constexpr bool REV_ROLLER_6 = true;
constexpr bool REV_ROLLER_7 = false;


// START STATES (piston2 starts retracted)
bool piston1Extended = true;
bool piston2Extended = false;
bool piston3Extended = false;

// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Rotation sensor for vertical tracking wheel (port 11)
// If tracking counts backwards, flip sign (use -11).
pros::Rotation vertRot(11);

// IMU (port 20)
pros::Imu imu(10);

//Distance sensor 
pros::Distance dist(14);

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
lemlib::ControllerSettings lateral_controller(8, // proportional gain (kP)
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

const int MOVE_TIMEOUT = 5000, TURN_TIMEOUT = 1000;

// ============================================================
// PROS DEFAULTS
// ============================================================
// initialize function. Runs on program startup
// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    piston1.set_value(true); // start with hood down
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

void leftside() { //arjun auton
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

void tuningtest() {
    chassis.setPose(0, 0, 0);
    
    //ANGULAR TUNE
    // chassis.turnToHeading(90, 5000);
    
    //LATERAL TUNE
    chassis.moveToPoint(0, 48, 5000, {.forwards = true}); // turn to face heading 90 with a very long timeout

    pros::delay(3500);
     auto pose = chassis.getPose();
            std::cout << "[DEBUG] X: " << pose.x 
                      << " Y: " << pose.y 
                      << " Theta: " << pose.theta << std::endl;

}

// ============================================================
// AUTON (WAYPOINTS)
// ============================================================

void intakeTest() {
  //TEST LATER  
  intake.telOP(false, false, true, false); //score mid testing ->works fine 
  intake.telOP(true, false, false, false);
  intake.telOP(false, true, false, false);

}

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

void matchload() {
  intake.telOP(true, false, false, false);
  pros::delay(2500);
}

void scoreTop() {
  intake.telOP(false, true, false, false);
  pros::delay(2500);
}

/*
Started completely from scratch on feb 1 2026
*/
void newAuton() {
  /*
  For future reference:
  piston1 = hood
  piston2 = wing -> booleans are reversed
  piston3 = matchload
  */
  chassis.setPose(0,0,0);
  
  piston3.set_value(true); //start with matchloader retracted
  piston2.set_value(false); //lift the wing so it doesn't interfere
  chassis.moveToPoint(-5, 27, 5000, {.maxSpeed = 40}, false); //move to the first 3 blocks
  
  //After arriving at the blocks, put matchloader down and switch intake on, moving forward a bit too
  piston3.set_value(false);
  pros::delay(400);
  intake.telOP(true, false, false, false); //leave intake on
  chassis.moveToPoint(-10, 37, 5000, {.forwards = true, .maxSpeed = 80}); //move forward to gather the balls
  pros::delay(500);

  //Intermediate point to help align with middle goal
  chassis.moveToPoint(-9, 32, 5000, {.forwards = false});

  chassis.turnToHeading(225, 1300);
  chassis.moveToPoint(4, 45, 5000, {.forwards = false, .maxSpeed = 40});

  //Score mid
  intake.telOP(false, false, true, false);
  pros::delay(1500); //wait for the balls to enter the tube
  intake.telOP(false, false, false, false); //stop scoring mid

  chassis.moveToPoint(-32, 10, 5000);
  chassis.turnToHeading(180, 1300);
  //Note matchloader is already activated
  chassis.moveToPoint(-32, -5, 2500, {.minSpeed = 60});
  //Move to matchload with low velocity so it doesn't mess up odom
  // chassis.moveToPoint(-32, -15, 5000, {.maxSpeed = 30});

  //Matchload
  matchload();

  chassis.moveToPoint(-32, 27, 5000, {.forwards = false, .maxSpeed = 50});
  //Score top
  piston1.set_value(true); //open up the hood
  intake.telOP(false, true, false, false);
  pros::delay(2000);

  //WING PART
  piston2.set_value(true);
  chassis.moveToPoint(-32, 20, 5000); //move back a bit
  chassis.turnToHeading(90, 1300);
  chassis.moveToPoint(-20, 20, 5000);
  chassis.turnToHeading(180, 1300);
  chassis.moveToPoint(-20, 35, 3000, {.forwards = false, .minSpeed = 60});
}

void autonomous() {
    // leftSideAuton();
    // oldKennyAuton();
    newAuton();
    // tuningtest();
    
}


/*
Autonomous skills started on feb 4 2026. Relies purely on odom, without any distance/optical sensors.
*/
void skills_cycle() {
  /*
  piston1 = hood
  piston2 = wing
  psiton3 = matchload
  Sidenote: maybe jiggling the matchloader back and forth can allow it to intake more easily?
  */

  //Piston starting setup
  piston3.set_value(false); //keep matchload down
  piston2.set_value(false); // keep wing up

  chassis.moveToPoint(2, 40, MOVE_TIMEOUT);
  chassis.turnToHeading(270, TURN_TIMEOUT);
  //move to the first matchload and matchload
  chassis.moveToPoint(-15, 40, MOVE_TIMEOUT);
  matchload();
  //Move back
  chassis.moveToPoint(2, 40, MOVE_TIMEOUT, {.forwards = false});
  chassis.turnToHeading(225, TURN_TIMEOUT);
  
  chassis.moveToPoint(22, 55, MOVE_TIMEOUT,{.forwards = false});
  chassis.turnToHeading(270, TURN_TIMEOUT);
  
  chassis.moveToPoint(95, 55, MOVE_TIMEOUT,{.forwards = false});
  chassis.turnToHeading(180, TURN_TIMEOUT);
  
  chassis.moveToPoint(95, 40, MOVE_TIMEOUT);
  chassis.turnToHeading(90, TURN_TIMEOUT);

  //SCORE !!!
  chassis.moveToPoint(74, 40, MOVE_TIMEOUT, {.forwards = false});
  scoreTop(); //should open up hood

  //move to second matchload 
  chassis.moveToPoint(115, 40, MOVE_TIMEOUT);
  matchload();

  //SCORE AGAIN!!
  chassis.moveToPoint(74, 40, MOVE_TIMEOUT, {.forwards = false});
  scoreTop();

  chassis.moveToPoint(95, 40, MOVE_TIMEOUT);
  chassis.turnToHeading(180, TURN_TIMEOUT);

    //Then reset and repeat (eg call this function again, after setPose(0,0,0)
}

void skills() {
  chassis.setPose(0,0,0);
  skills_cycle();
  chassis.moveToPoint(99, -13, MOVE_TIMEOUT);
  chassis.setPose(0,0,0);
  skills_cycle();
  chassis.moveToPoint(99, -6, MOVE_TIMEOUT);
  //Go for park at the end
  chassis.turnToHeading(90, TURN_TIMEOUT);
  chassis.moveToPoint(120, -6, MOVE_TIMEOUT);
  intake.telOP(true, false, false, false);
}


void distanceSensorTest() {
  const double TARGET_DIST_INCHES = 24.0;  // Stop 24 inches (1 tile) from wall
  const int DRIVE_SPEED = 60;              // Motor power (adjust as needed)
  
  // Keep moving forward until we're within target distance of the wall
  while (true) {
    double dist_mm = dist.get_distance();
    double dist_inches = dist_mm / 25.4;
    
    // Check if we've reached target distance
    if (dist_inches <= TARGET_DIST_INCHES) {
      break;
    }
    
    // Drive forward (positive = forward for both motor groups)
    left_motors.move(DRIVE_SPEED);
    right_motors.move(DRIVE_SPEED);
    
    // Small delay to prevent CPU hogging
    pros::delay(10);
  }
  
  // Stop the motors once we reach the target distance
  left_motors.move(0);
  right_motors.move(0);
}

void recallibrate() {
  distanceSensorTest();
  //turn right
  chassis.turnToHeading(90, 5000);
  //turn left
  chassis.turnToHeading(-90, 5000);
  distanceSensorTest();
}


// ============================================================
// DRIVER CONTROL
// ============================================================

void opcontrol() {
  
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
        } else if (intakeBtn) {
          // L1: stack/intake, L2: score top goal (activates hood)
          setRoller5(127);
          setRoller6(127);
          setRoller7(-127);
        } else if (scoreTopBtn) {
          piston1.set_value(false); // hood up while L2 held
          piston1Extended = false;

          setRoller5(127);
          setRoller6(127);
          setRoller7(-127);
        }
        else if (descoreBtn) {
          // Descore everything from matchload
          setRoller5(-127);
          setRoller6(-127);
          setRoller7(127);
        } else {
          // Stop all rollers
          setRoller5(0);
          setRoller6(0);
          setRoller7(0);
          
          // Hood down when not scoring top
          piston1.set_value(true);
          piston1Extended = true;
        }

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