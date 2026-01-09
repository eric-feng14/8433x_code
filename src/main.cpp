#include "main.h"
#include "lemlib/api.hpp"
#include <cmath>

// ======================= PATH.JERRYIO ASSET =======================
// Keep this if you want; it will NOT affect driver.
// ASSET(test_txt);

// ============================================================
// PORTS
// ============================================================

// Drive motors
pros::MotorGroup left_motors({-1, -2, -3}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({4, 8, 20}, pros::MotorGearset::blue);

// Intake / rollers
pros::Motor roller_5(-5, pros::MotorGearset::blue);
pros::Motor roller_6(6, pros::MotorGearset::blue);
pros::Motor roller_7(7, pros::MotorGearset::blue);

// Pneumatics
pros::adi::DigitalOut piston1('A');    
pros::adi::DigitalOut piston2('B');

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
pros::Rotation vertRot(-11);

// IMU (port 10)
pros::Imu imu(12);

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

lemlib::TrackingWheel verticalTrack(&vertRot, TRACK_WHEEL_TYPE, VERT_OFFSET_IN);

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
    360,
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
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    
    // Reset and initialize the rotation sensor
    vertRot.reset_position();
    
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

            // delay to save resources
            std::cout << "x: " << chassis.getPose().x << std::endl;
            std::cout << "y: " << chassis.getPose().y << std::endl;
            std::cout << "theta: " << chassis.getPose().theta << std::endl;
            std::cout << "RotSensor Pos: " << vertRot.get_position() << std::endl;
            std::cout << "RotSensor Installed: " << vertRot.is_installed() << std::endl;
            std::cout << "IMU Heading: " << imu.get_heading() << std::endl;

            pros::delay(1000); // increased from 20ms to avoid LVGL rendering conflicts
        }
    });
}

void disabled() {}
void competition_initialize() {}

// ============================================================
// AUTON (WAYPOINTS)
// ============================================================
void autonomous() {
    chassis.setPose(0, 0, 0); // or 0  
    // turn to face heading 90 with a very long timeout
    chassis.turnToHeading(90, 100000);
    
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
    bool lastA = false;
    bool lastB = false;




    while (true) {
        // ---------------- DRIVE (ARCADE) ----------------
        int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn    = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);




        int leftPower  = clamp127(forward + turn);
        int rightPower = clamp127(forward - turn);




        left_motors.move(maybeRev(leftPower, REV_LEFT_DRIVE));
        right_motors.move(maybeRev(rightPower, REV_RIGHT_DRIVE));




        // ---------------- MANUAL ROLLERS 5/6 (R2 forward, L2 reverse) ----------------
        bool r2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool l2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);




        int power56 = 0;
        if (r2 && !l2) power56 = 127;
        else if (l2 && !r2) power56 = -127;
        setRollers56(power56);




        // ---------------- MOTOR 7 ----------------
        bool r1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);




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




        // ---------------- PISTONS (toggle) ----------------
        bool currA = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        if (currA && !lastA) {
            piston1Extended = !piston1Extended;
            piston1.set_value(piston1Extended);
        }
        lastA = currA;




        bool currB = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        if (currB && !lastB) {                 // ✅ toggle on press
            piston2Extended = !piston2Extended;
            piston2.set_value(piston2Extended);
        }
        lastB = currB;




        pros::delay(20);
    }
}

