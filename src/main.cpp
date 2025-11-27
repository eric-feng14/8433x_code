#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

pros::MotorGroup left_motors({-18, -19, -20}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges
pros::MotorGroup right_motors({1, 2, 3}, pros::MotorGearset::blue);		 // right motors use 200 RPM cartridges
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11.25, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);
// create an imu on port 10
pros::Imu imu(10);

pros::Motor intake(6);
pros::Motor hook(-11);
pros::Motor arm(5);


pros::ADIDigitalOut clamp('H');
pros::ADIDigitalOut doinker('G');
pros::ADIEncoder arm_encoder('A', 'B', false);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// // lateral PID controller
// lemlib::ControllerSettings lateral_controller(0, // proportional gain (kP)
//                                               0, // integral gain (kI)
//                                               0, // derivative gain (kD)
//                                               0, // anti windup
//                                               0, // small error range, in inches
//                                               0, // small error range timeout, in milliseconds
//                                               0, // large error range, in inches
//                                               0, // large error range timeout, in milliseconds
//                                               0 // maximum acceleration (slew)
// );

// // angular PID controller
// lemlib::ControllerSettings angular_controller(21, // proportional gain (kP)
//                                               0, // integral gain (kI)
//                                               155,// derivative gain (kD)
//                                               0, // anti windup
//                                               0, // small error range, in degrees
//                                               0, // small error range timeout, in milliseconds
//                                               0, // large error range, in degrees
//                                               0, // large error range timeout, in milliseconds
//                                               0 // maximum acceleration (slew)
// );

// lateral motion controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
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


lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
							nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
							nullptr, // horizontal tracking wheel 1
							nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
							&imu); // inertial sensor

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
						lateral_controller, // lateral PID settings
						angular_controller, // angular PID settings
						sensors, // odometry sensors (left, middle, right)
						&throttle_curve, 
                        &steer_curve
);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(2, "working");
	}
	else
	{
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
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
void autonomous()
{
	// Set the initial pose of the robot (x, y, heading)
	chassis.setPose(0, 0,-40); // Start position: 1 tile from right edge, facing backwards
	
	// Moves the arm only
	clamp.set_value(false);
	arm.move(90);
	pros::delay(1500);
	arm.move(-90);
	pros::delay(1000);
	arm.move(0);
	// left_motors.move(-80);
	// right_motors.move(-80);


	chassis.moveToPoint(1, -35, 3000, {.forwards = false});

	// Clamp the mobile goal
	clamp.set_value(true);
	pros::delay(1500);
	clamp.set_value(false);
	chassis.moveToPoint(1, -47, 2000, {.forwards = false});
	chassis.moveToPoint(1, -41, 500);
	chassis.moveToPoint(1, -46.5, 500, {.forwards = false});


	intake.move(127);
	hook.move(-100);


	chassis.moveToPoint(20, -41.5, 3000);
	chassis.moveToPoint(20, -57, 3000);
	chassis.moveToPoint(25, -57, 3000);
	
}
i


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

void opcontrol()
{
	while (true)
	{
		// get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.curvature(leftY, rightX);

		// Intake control with L1 (forward) and L2 (reverse)
		if (controller.get_digital(DIGITAL_R1))
		{
			intake.move(127); // Full speed forward
			hook.move(-100);  // Full speed forward
		}
		else if (controller.get_digital(DIGITAL_R2))
		{
			hook.move(100);	   // Full speed forward
			intake.move(-127); // Full speed reverse
		}
		else
		{
			hook.move(0);	// Full speed forward
			intake.move(0); // Stop if neither button is pressed
		}

		// Auto-arm toggle using Y button
		bool currentY = controller.get_digital(DIGITAL_Y);
		if (currentY && !autoArmLastY)
		{
			autoArmEnabled = !autoArmEnabled;
			if (autoArmEnabled)
				autoTargetArm = initialArm + 40;
		}
		autoArmLastY = currentY;

		int currentArm = arm_encoder.get_value();

		pidIntegral = 0;
		lastError = 0;
		if (controller.get_digital(DIGITAL_L1))
		{
			arm.move(60);
		}
		else if (controller.get_digital(DIGITAL_L2))
		{
			arm.move(-60);
		}
		else
		{
			arm.move(0);
		}

		// Set solenoid based on toggled state
		clamp.set_value(clamp_state);

		// Single-action solenoid control for doinker
		static bool doinker_state = false;
		static bool doinker_last_x_state = false;
		bool doinker_current_x_state = controller.get_digital(DIGITAL_X);

		// Toggle state on button press (not hold)
		if (doinker_current_x_state && !doinker_last_x_state)
		{
			doinker_state = !doinker_state;
		}
		doinker_last_x_state = doinker_current_x_state;

		// Set solenoid based on toggled state
		doinker.set_value(doinker_state);

		pros::delay(20);
	}
}