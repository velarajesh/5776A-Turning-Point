#include "main.h"

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
    drive::currState = drive::driveStates::yield;
    differential::currState = differential::differentialStates::yield;
    puncher::currState = puncher::puncherStates::yield;
    angler::currState = angler::anglerStates::yield;

    drive::chassisController.setBrakeMode(AbstractMotor::brakeMode::brake);

    drive::ramBoi.setCorrectionMode(RRLib::RamseteProfileController::CorrectionMode::Heading);

    switch (autonRoutine) // Executes specific routine based on auton selector.
    {
    case notSelected:
        break;
    case progSkills:
        executeProgSkills();
        break;

    case redNear1:
        executeRedNear1();
        break;

    case redNear2:
        executeRedNear2();
        break;

    case redFar1:
        executeRedFar1();
        break;

    case redFar2:
        executeRedFar2();
        break;

    case blueNear1:
        executeBlueNear1();
        break;

    case blueNear2:
        executeBlueNear2();
        break;

    case blueFar1:
        executeBlueFar1();
        break;

    case blueFar2:
        executeBlueFar2();
        break;
    }

    executeRedFar2();

    // drive::ramBoi.setRamseteDisabled(true);

    // pros::delay(3000);

    // drive::profileController.generatePath({
    //     Point{0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
    //     Point{24_in, 0_ft, 0_deg}}, // The next point in the profile, 3 feet forward
    //     "A" // Profile name
    // );

    // drive::profileController.setTarget("A");
    // drive::profileController.waitUntilSettled();

    // drive::ramBoi.moveTo({ 24_in, 0_in, 0_deg });

    // drive::ramBoi.moveTo({ 0_in, 0_in, 0_deg });

    // odometry::init();
    // odometry::resetAngle(0_deg);
    // currentPose.position.getX() = 0_in;
    // currentPose.position.getY() = 0_in;

    // pros::delay(500);

    //     odometry::init();
    //     odometry::resetAngle(0_deg);
    //     currentPose.position.getX() = 0_in;
    //     currentPose.position.getY() = 0_in;

    //     path::Line A1(
    //         {currentPose.position.getX(), currentPose.position.getY()},
    //         {55_in, 77.5_in},
    //         1000,
    //         1000);

    //     path::Line bingbong(
    //     {0_in, 0_in},
    //     {0_in, 20_in},
    //     1000, 500
    //   );

    //   path::Line bingbongbing(
    //     {0_in, 20_in},
    //     {-20_in, 20_in},
    //     1000, 500
    //   );

    //   path::Line bingbongbingbong(
    //     {-20_in, 20_in},
    //     {-20_in, 40_in},
    //     1000, 500
    //   );

    //   path::PathGroup bingbongGroup(
    //     {
    //       bingbong,
    //       bingbongbing,
    //       bingbongbingbong
    //     },
    //     3000, 1000
    //   );

    //     // differential::currState = differential::intakeIn;
    //     drive::appc.runPath(&bingbongGroup);

    // path::Line A2B(
    //     {currentPose.position.getX(), currentPose.position.getY()},
    //     {13.5_in, 36_in},
    //     1000,
    //     1000);

    // drive::appc.runPath(&A2B);

    // pros::delay();

    // path::Line A2A(
    //     {currentPose.position.getX(), currentPose.position.getY()},
    //     {30_in, 12_in},
    //     1000,
    //     1000);

    // path::Line A2B(
    //     {30_in, 12_in},
    //     {53_in, 12_in},
    //     1000,
    //     1000);

    // path::PathGroup A2(
    //     {A2A,
    //      A2B},
    //     2000, 1000);

    // path::Bezier A2urgay(
    //     {path::Point{currentPose.position.getX(), currentPose.position.getY()},
    //      path::Point{30_in, currentPose.position.getY()},
    //      path::Point{30_in, 12_in},
    //      path::Point{currentPose.position.getX(), 12_in}},
    //     1000,
    //     200);

    // drive::appc.runPath(&A2);

    // // // Drive forward into the cap #1 and intake ball

    // executeProgSkills();
    // executeRedNear1();
    // executeRedFar1();
    // executeRedFar2();
    // executeBlueFar1();
    // executeBlueFar2();
    // executeBlueNear1();

    // odometry::init();
    // odometry::resetAngle(90_deg);
    // currentPose.position.getX() = 13.25_in;
    // currentPose.position.getY() = 36_in;

    // pros::delay(500);

    // path::Line A1(
    //     {currentPose.position.getX(), currentPose.position.getY()},
    //     {53.25_in, 36_in},
    //     1000,
    //     1000);
    // drive::appc.runPath(&A1);
    // // Drive forward into the cap #1 and intake ball

    // path::Bezier A2(
    //     {path::Point{currentPose.position.getX(), currentPose.position.getY()},
    //      path::Point{40_in, 36_in},
    //      path::Point{20_in, 12.5_in},
    //      path::Point{50_in, 12.5_in}},
    //     1000,
    //     250);

    // drive::appc.runPathAsync(&A2);

    // pros::delay(500);

    // differential::liftTarget = 10;
    // differential::currState = differential::liftPID;

    // drive::appc.waitUntilSettled();

    // differential::liftTarget = 500;
    // differential::currState = differential::liftPID;

    // pros::delay(1000);

    // path::Bezier A3(
    //     {path::Point{currentPose.position.getX(), currentPose.position.getY()},
    //      path::Point{20_in, 12.5_in},
    //      path::Point{48_in, 24_in},
    //      path::Point{10_in, 24_in}},
    //     1000,
    //     200);

    // drive::appc.runPathAsync(&A3);

    // pros::delay(1000);

    // differential::liftTarget = 900;
    // differential::currState = differential::liftPID;

    // drive::appc.waitUntilSettled();

    // drive::turn(-3600_deg, 100, false, false);
    // lv_obj_clean(lv_scr_act());
}
