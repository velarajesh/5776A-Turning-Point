#include "main.h"
#include "subsystems/angler.hpp"
#include "subsystems/drive.hpp"
#include "subsystems/differential.hpp"
#include "subsystems/puncher.hpp"


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management	 System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void states()
{
	auto pose = drive::odometry.getPose();
	double x = pose.position.getX().convert(okapi::inch);
	double y = pose.position.getY().convert(okapi::inch);
	double angle = pose.heading.convert(okapi::degree);

	printf("%f,%f\n", x, -y);

	// pros::lcd::print(1, "Drive state: %c | Drive temp: %d", drive::currState, (int)drive::driveR1.getTemperature());
	pros::lcd::print(2, "Drive state: %c | Drive temp: %d", drive::currState, (int)drive::driveR1.getTemperature());
	pros::lcd::print(3, "X: %.2f", x);
	pros::lcd::print(4, "Y: %.2f", y);
	pros::lcd::print(5, "A: %.2f", angle);
	// pros::lcd::print(2, "Puncher state: %c | Puncher temp: %d", puncher::currState, (int)puncher::puncher.getTemperature());
	pros::lcd::print(7, "Ang Target: %d | Ang Enc: %d | Ang Temp: %d", (int)angler::target, (int)angler::angler.getPosition(), (int)angler::angler.getTemperature());
	// pros::lcd::print(4, "Diff state: %c | Diff Left temp: %d", differential::currState, (int)differential::diffLeft.getTemperature());
	// pros::lcd::print(5, "Macro state: %c", macro::currState);
}

void opcontrol()
{
	// macroActTask->resume(); // could this be null??
	pros::lcd::initialize();
	// drive::odometry.setPose({84_in, 13.25_in, 90_deg});

	while (true)
	{
		// printf("Ball Sensor: %d\n", (int)differential::line.get_value());
		states();

		// printf("isLoaded: %d | hasBall: %d\n", puncher::puncherIsLoaded, differential::intakeHasBall);
		pros::delay(10);
	}
}