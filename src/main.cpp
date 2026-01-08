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
pros::MotorGroup right_motors({4, 8, 9}, pros::MotorGearset::blue);

// Intake / rollers
pros::Motor roller_5(-5, pros::MotorGearset::blue);
pros::Motor roller_6(6, pros::MotorGearset::blue);
pros::Motor roller_7(7, pros::MotorGearset::blue);

// Pneumatics
pros::ADIDigitalOut piston1('A');
pros::ADIDigitalOut piston2('B');

// START STATES (piston2 starts retracted)
bool piston1Extended = false;
bool piston2Extended = false;

// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Rotation sensor for vertical tracking wheel (port 16)
// If tracking counts backwards, flip sign.
pros::Rotation vertRot(-16);

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
constexpr float VERT_OFFSET_IN = 0.0; // <-- measure later

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
    DRIVETRAIN_RPM,
    2
);

// PID settings (starting values only)
lemlib::ControllerSettings lateral_controller(
    8, 0, 25,
    3,
    1, 100,
    3, 500,
    20
);

lemlib::ControllerSettings angular_controller(
    2.2, 0, 12,
    3,
    1, 100,
    3, 500,
    0
);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

// ============================================================
// PROS DEFAULTS
// ============================================================
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(0, "BOOTING...");

    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    roller_5.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    roller_6.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    roller_7.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    piston1.set_value(piston1Extended);
    piston2.set_value(piston2Extended);

    // --------- SAFE IMU CALIBRATION (WITH TIMEOUT) ----------
    pros::lcd::set_text(1, "IMU reset...");
    imu.reset();

    const int start = pros::millis();
    while (imu.is_calibrating() && (pros::millis() - start) < 3000) {
        pros::delay(20);
    }

    if (imu.is_calibrating()) {
        // IMU still calibrating (or not detected) -> DO NOT BLOCK
        pros::lcd::set_text(1, "IMU FAIL/STALL -> driver OK");
    } else {
        pros::lcd::set_text(1, "IMU OK");
        // LemLib calibrate can still take time; keep it short and safe:
        // If this ever causes problems again, comment this line out.
        chassis.calibrate();
    }

    pros::lcd::set_text(0, "READY");
}

void disabled() {}
void competition_initialize() {}

// ============================================================
// AUTON (WAYPOINTS)
// ============================================================
void autonomous() {
    chassis.setPose(-47.171, 22.057, 90); // or 0  
    
    // Keep intake running upwards throughout auton
    setRollers56(127);
    setRoller7(127);
    
    // chassis.moveToPoint(0, 0, 5000);
    // chassis.moveToPoint(0, 24.76, 5000);
    // chassis.moveToPoint(-23.109, -18.629, 5000);
    // chassis.moveToPoint(-23.109, 22.637, 5000);
    // chassis.moveToPoint(-47.171, 22.057, 5000); //initial starting position

    chassis.moveToPoint(-22.647, 21.822, 5000);
    chassis.turnToHeading(300, 5000);
    chassis.moveToPoint(-60.612, 46.11, 5000);
    chassis.turnToHeading(90, 5000);
    chassis.moveToPoint(-23.118, 46.817, 5000);


    left_motors.move(0);
    right_motors.move(0);
}

// ============================================================
// DRIVER CONTROL
// ============================================================
void opcontrol() {
    bool lastA = false;
    bool lastB = false;

    while (true) {
        int forward = deadband(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        int turn = deadband(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        int leftPower  = clamp127(forward + turn);
        int rightPower = clamp127(forward - turn);

        left_motors.move(leftPower);
        right_motors.move(rightPower);

        bool r2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool l2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        bool r1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);

        if (r2 && !l2) setRollers56(127);
        else if (l2 && !r2) setRollers56(-127);
        else setRollers56(0);

        if (r1) setRoller7(-127);
        else if (r2 && !l2) setRoller7(127);
        else if (l2 && !r2) setRoller7(-127);
        else setRoller7(0);

        bool currA = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        if (currA && !lastA) {
            piston1Extended = !piston1Extended;
            piston1.set_value(piston1Extended);
        }
        lastA = currA;

        bool currB = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        if (currB && !lastB) {
            piston2Extended = !piston2Extended;
            piston2.set_value(piston2Extended);
        }
        lastB = currB;

        pros::lcd::print(2, "LY:%d RX:%d", forward, turn);
        pros::lcd::print(3, "IMU cal:%d", (int)imu.is_calibrating());

        pros::delay(20);
    }
} 
