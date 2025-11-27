#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

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
		pros::lcd::set_text(2, "I was pressed!");
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
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
void autonomous() {}

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
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


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

		// // hook control with R1 (forward) and R2 (reverse)

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

		// When auto-arm is enabled, use PID (adjust target faster with L buttons)
		if (autoArmEnabled)
		{
			// Adjust the target if L1/L2 is pressed (faster adjustment)
			if (controller.get_digital(DIGITAL_L1))
				autoTargetArm += 3; // Increase target faster
			if (controller.get_digital(DIGITAL_L2))
				autoTargetArm -= 3; // Decrease target faster

			// PID control to hold the arm at the autoTargetArm position
			int error = autoTargetArm - currentArm;
			pidIntegral += error * dt; // dt = 20ms for rotaitonal sensor, we are using 10ms for shaft encoder
			int derivative = (error - lastError) / dt;
			int pidOutput = static_cast<int>(Kp * error + Ki * pidIntegral + Kd * derivative);
			lastError = error;
			// Clamp output
			if (pidOutput > 127)
				pidOutput = 127;
			if (pidOutput < -127)
				pidOutput = -127;
			arm.move(pidOutput);
		}
		else // When auto-arm is off, manual control is full (PID is bypassed)
		{
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
		}

		// Single-action solenoid control for clamp
		static bool clamp_state = false;
		static bool clamp_last_a_state = false;
		bool clamp_current_a_state = controller.get_digital(DIGITAL_A);

		// Toggle state on button press (not hold)
		if (clamp_current_a_state && !clamp_last_a_state)
		{
			clamp_state = !clamp_state;
		}
		clamp_last_a_state = clamp_current_a_state;

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