#include "main.h"
#include "lemlib/api.hpp"

// ======================= ASSETS / PATHS ===========================
ASSET(test_txt); // compiled path file from LemLib (test.txt)

// ======================= HARDWARE SETUP ===========================

// Drive motors (blue cartridges, 3.25" wheels)
// Negative port on left_motors means reversed motor
pros::MotorGroup right_motors({1, 2, -8}, pros::MotorGearset::blue);
pros::MotorGroup left_motors({-3, -4, 7}, pros::MotorGearset::blue);

// Scoring motors (ports 6 and 7) – can spin both directions
pros::MotorGroup scoring_motors({6, -7 });

// Intake motor
pros::Motor intake(5);

// Inertial sensor
pros::Imu imu(10);

// ==================== PNEUMATICS / PISTONS ================`========
// OPTION 1: pistons plugged directly into Brain 3-wire ports A and B:
pros::ADIDigitalOut piston1('A'); // solenoid 1 -> piston 1
pros::ADIDigitalOut piston2('B'); // solenoid 2 -> piston 2

// OPTION 2 (if you use an ADI expander on smart port X):
// pros::ADIDigitalOut piston1(X, 'A');
// pros::ADIDigitalOut piston2(X, 'B');

// Global states for pistons (so initialize() and opcontrol() match)
bool piston1Extended = true; // piston1 starts retracted/closed
bool piston2Extended = true;  // piston2 starts extended/open (like you said)

// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// ==================== DRIVETRAIN / LEMLIB SETUP ===================

// Drivetrain gearing: blue cart 600 RPM, 36T -> 60T external
const int MOTOR_TEETH = 36;
const int WHEEL_TEETH = 60;
const int MOTOR_CARTRIDGE_RPM_BLUE = 600;
const int DRIVETRAIN_RPM =
    MOTOR_CARTRIDGE_RPM_BLUE * MOTOR_TEETH / WHEEL_TEETH; // = 360 RPM

// 3.25" drive wheels (2 omnis + 1 traction each side)
lemlib::Drivetrain drivetrain(
    &left_motors,               // left motor group
    &right_motors,              // right motor group
    10,                         // track width (inches, tune if needed)
    lemlib::Omniwheel::NEW_325, // using 3.25" wheel model
    DRIVETRAIN_RPM,             // computed rpm
    2                           // lateral drift (tune later)
);

// Odometry sensors – IMU ONLY, no tracking wheels right now
lemlib::OdomSensors sensors(
    nullptr, // vertical tracking 1
    nullptr, // vertical tracking 2
    nullptr, // horizontal tracking 1
    nullptr, // horizontal tracking 2
    &imu     // inertial sensor
);

// Lateral PID
lemlib::ControllerSettings lateral_controller(
    10,   // kP
    0,    // kI
    3,    // kD
    3,    // anti-windup
    1,    // small error (in)
    100,  // small error timeout (ms)
    3,    // large error (in)
    500,  // large error timeout (ms)
    20    // slew (max accel)
);

// Angular PID
lemlib::ControllerSettings angular_controller(
    2,    // kP
    0,    // kI
    10,   // kD
    3,    // anti-windup
    1,    // small error (deg)
    100,  // small error timeout (ms)
    3,    // large error (deg)
    500,  // large error timeout (ms)
    0     // slew (0 = disabled)
);

// Chassis object
lemlib::Chassis chassis(
    drivetrain,
    lateral_controller,
    angular_controller,
    sensors
);

// ======================= HELPER CONSTANTS =========================

// Driver period + endgame reminder
constexpr int DRIVER_PERIOD_MS = 105000;      // 1:45
constexpr int ENDGAME_WARN_OFFSET_MS = 20000; // last 20 s
constexpr int ENDGAME_WARN_MS = DRIVER_PERIOD_MS - ENDGAME_WARN_OFFSET_MS;

// =========================== INITIALIZE ===========================

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(0, "8433X - PROS + LemLib (3.25\" drive)");

    // Calibrate IMU + LemLib
    chassis.calibrate();

    // Brake modes
    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    scoring_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // Set pistons to known startup states
    // piston1: retracted/closed (false)
    // piston2: extended/open (true) – matches "always open until I retract"
    piston1Extended = true;
    piston2Extended = true;
    piston1.set_value(piston1Extended);
    piston2.set_value(piston2Extended);

    // Screen task to show pose
    pros::Task screen_task([] {
        while (true) {
            lemlib::Pose pose = chassis.getPose();
            pros::lcd::print(0, "X: %.2f in", pose.x);
            pros::lcd::print(1, "Y: %.2f in", pose.y);
            pros::lcd::print(2, "Th: %.2f deg", pose.theta);
            pros::delay(20);
        }
    });
}

void disabled() {}
void competition_initialize() {}

// ============================ AUTON ===============================

void autonomous() {
    pros::lcd::print(3, "Auton running...");
    chassis.setPose(0, 0, 0);            // starting pose
    chassis.follow(test_txt, 15, 14000); // lookahead 15, timeout 14s
    pros::lcd::print(3, "Auton done.     ");
}

// ========================== DRIVER CONTROL ========================

void opcontrol() {
    // Endgame rumble timing
    const int startTime = pros::millis();
    bool endgameRumbled = false;

    // For button edge detection
    bool lastA = false;
    bool lastB = false;

    // Make sure drive motors are ready
    left_motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // Neutral actuators
    intake.move(0);
    scoring_motors.move(0);

    pros::lcd::print(3, "Driver control...");

    while (true) {
        // ================== DRIVING (RAW ARCADE) ==================
        // Left stick Y = forward/back  (we negate so up = forward)
        // Right stick X = turn left/right
        int forward = -master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn    =  master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Arcade mix
        int leftPower  = forward + turn;
        int rightPower = forward - turn;

        // Clamp to [-127, 127] just to be safe
        if (leftPower > 127) leftPower = 127;
        if (leftPower < -127) leftPower = -127;
        if (rightPower > 127) rightPower = 127;
        if (rightPower < -127) rightPower = -127;

        // Directly command the motor groups (no LemLib here)
        left_motors.move(leftPower);
        right_motors.move(rightPower);

        // ================== INTAKE ==================
        // L1 = intake in, L2 = intake out
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move(-127);
        } else {
            intake.move(0);
        }

        // ================== SCORING MOTORS (PORT 6) ==================
        // R1 = scoring forward, R2 = scoring reverse
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            scoring_motors.move(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            scoring_motors.move(-127);
        } else {
            scoring_motors.move(0);
        }

        // ================== PISTON 1 (PORT A) – button A ==================
        bool currA = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        if (currA && !lastA) { // rising edge
            piston1Extended = !piston1Extended;
            piston1.set_value(piston1Extended);
            master.rumble("."); // small haptic feedback
        }
        lastA = currA;

        // ================== PISTON 2 (PORT B) – button B ==================
        bool currB = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        if (currB && !lastB) { // rising edge
            piston2Extended = !piston2Extended;
            piston2.set_value(piston2Extended);
            master.rumble("."); // small haptic feedback
        }
        lastB = currB;

        // ================== ENDGAME REMINDER ==================
        int elapsed = pros::millis() - startTime;
        if (!endgameRumbled && elapsed >= ENDGAME_WARN_MS) {
            master.rumble(".."); // quick double rumble near endgame
            endgameRumbled = true;
        }

        pros::delay(20);
    }
}
