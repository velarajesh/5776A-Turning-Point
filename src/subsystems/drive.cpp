#include "main.h"
#include "vision/vision.hpp"
#include "vision/visionReader.hpp"
#include "vision/objDrawer.hpp"
#include "vision/objContainer.hpp"

using namespace okapi;

namespace drive
{

driveStates currState;

Motor driveL1(DRIVE_L1, false, AbstractMotor::gearset::green);
Motor driveL2(DRIVE_L2, false, AbstractMotor::gearset::green);
Motor driveR1(DRIVE_R1, false, AbstractMotor::gearset::green);
Motor driveR2(DRIVE_R2, false, AbstractMotor::gearset::green);

ADIEncoder leftTrackingEncoder(SPORT_LTOP, SPORT_LBOT, true);
ADIEncoder rightTrackingEncoder(SPORT_RTOP, SPORT_RBOT);

TimeUtil chassisUtil = TimeUtilFactory::withSettledUtilParams(50, 5, 100_ms);
TimeUtil profiledUtil = TimeUtilFactory::withSettledUtilParams(50, 5, 100_ms);

okapi::MotorGroup leftMotorGroup({DRIVE_L1, DRIVE_L2});
okapi::MotorGroup rightMotorGroup({DRIVE_R1, DRIVE_R2});

AsyncPosIntegratedController leftController(std::shared_ptr<MotorGroup>(&leftMotorGroup), chassisUtil);
AsyncPosIntegratedController rightController(std::shared_ptr<MotorGroup>(&rightMotorGroup), chassisUtil);

SkidSteerModel integratedChassisModel = ChassisModelFactory::create({DRIVE_L1, DRIVE_L2}, {-DRIVE_R1, -DRIVE_R2}, 200);
SkidSteerModel discreteChassisModel = ChassisModelFactory::create({DRIVE_L1, DRIVE_L2}, {-DRIVE_R1, -DRIVE_R2}, leftTrackingEncoder, rightTrackingEncoder, 200);
// ChassisScales integratedScale = std_initializer_list<ChassisScales>(4.125_in, 13.273906_in);
// ChassisScales discreteScale = std_initializer_list<ChassisScales>(2.75_in, 7.402083_in);

ChassisScales trackingWheelsScales = {2.75_in, 7.300486_in};
ChassisScales drivenWheelsScales = {4.125_in, 12.676583_in};

ChassisControllerIntegrated chassisController(
    chassisUtil,
    std::shared_ptr<SkidSteerModel>(&integratedChassisModel),
    std::unique_ptr<AsyncPosIntegratedController>(&leftController),
    std::unique_ptr<AsyncPosIntegratedController>(&rightController),
    AbstractMotor::gearset::green, drivenWheelsScales);

RRLib::KinematicConstraints kinConst(1.0, 2, 10.0);

AsyncMotionProfileController profileController = AsyncControllerFactory::motionProfile(
    // 1.05, // Maximum linear velocity of the Chassis in m/s
    // 1.5, // Maximum linear acceleration of the Chassis in m/s/s
    // 5.0, // Maximum linear jerk of the Chassis in m/s/s/s
    1,
    2,
    10.0,
    chassisController // Chassis Controller
);


RRLib::TwoWheelOdometry odometry(std::shared_ptr<SkidSteerModel>(&discreteChassisModel), trackingWheelsScales);
std::shared_ptr<RRLib::PoseEstimator> poseEstimator = std::shared_ptr<RRLib::PoseEstimator>(&odometry);
RRLib::RamseteGains rgains{ 0.8, 5.0, 0.0 };
RRLib::RamseteProfileController ramBoi(rgains, 2.0, std::shared_ptr<SkidSteerModel>(&discreteChassisModel), poseEstimator, kinConst, drivenWheelsScales, AbstractMotor::gearset::green);

AverageFilter<10> visionFilter;

int largestObjX;
int visionPasses = 0;

pros::Task _odomt([](void*) {
    while (true) {
        const double TICKSINCH = 2.75 * PI / 360.0;
        const double TICKSINCH2 = 4.125 * PI / 360.0;

        pros::lcd::print(1, "l: %.2f, r: %.2f", leftTrackingEncoder.get(), rightTrackingEncoder.get());
        // poseEstimator->printPose();
        poseEstimator->update();
        auto pose = drive::odometry.getPose();
        pros::lcd::print(3, "X: %.2f", pose.position.getX().convert(inch));
	    pros::lcd::print(4, "Y: %.2f", pose.position.getY().convert(inch));
	    pros::lcd::print(5, "A: %.2f", pose.position.getTheta().convert(degree));

        pros::lcd::print(5, "enc chassis: %f", (rightTrackingEncoder.get() * TICKSINCH - leftTrackingEncoder.get() * TICKSINCH) / (20 * PI));
        pros::lcd::print(6, "motor chassis: %f", (((drive::driveR1.get_position() + drive::driveR2.get_position())) / 2 * TICKSINCH2 - ((drive::driveL1.get_position() + drive::driveL2.get_position()) / 2) * TICKSINCH2) / (20 * PI));
        pros::delay(2);
    }
});

void update()
{
    // currState = notRunning;
    if (abs(controller.getAnalog(ControllerAnalog::leftY)) > joyDeadband ||
        abs(controller.getAnalog(ControllerAnalog::rightY)) > joyDeadband)
    {
        currState = running; // begin running drive once small threshold on joystick is reached
    }
    // currState = autoAim;
}

void act(void *)
{

    // pros::delay(500);
    // vision::vis.set_exposure(150);

    while (true)
    {
        // largestObjX = visionFilter.filter(vision::reader.get(0).x);

        // pros::lcd::print(0, "%f", drive::driveL1.getPosition());
        // pros::lcd::print(1, "%c", drive::currState);
        // printf("x: %i\n", largestObjX);

        switch (currState)
        {
        case notRunning: // the drive should not be moving; brake
            chassisController.setBrakeMode(AbstractMotor::brakeMode::coast);
            chassisController.stop();
            break;

        case running: // the drive moves according to joysticks
            chassisController.tank(
                controller.getAnalog(ControllerAnalog::leftY) * 1.0,
                controller.getAnalog(ControllerAnalog::rightY) * 1.0,
                joyDeadband * 1.0);
            currState = notRunning;
            break;
        case autoAim:
            visionPasses++;
            // largestObjX = vision::reader.get(0).x;
            // printf("x: %i\n", largestObjX);
            // printf("ang: %f\n", vision::angToTarget(largestObjX).convert(degree));
            // if (largestObjX == 0)
            // {
            //     controller.rumble(".  ");
            // }
            // else
            // {
            // drive::turn(currentPose.heading + vision::angToTarget(largestObjX), 200, true, false);
            // drive::chassisController.turnAngleAsync(vision::angToTarget(largestObjX));
            // }

            if (visionPasses >= 1)
            {
                visionPasses = 0;
                currState = notRunning;
            }
            break;
        case yield: // for macro in order to take direct control of drive
            break;
        }
        pros::delay(10);
    }
}

// Helper function to facilitate turning during auton
void turn(QAngle angle, double maxVel, bool odom, bool async)
{
    auto pose = drive::odometry.getPose();
    drive::chassisController.setMaxVelocity(maxVel);
    if (async)
    {
        if (odom)
        {
            double angleError = std::atan2(sin(angle.convert(radian) - pose.heading.convert(radian)), cos((angle.convert(radian) - pose.heading.convert(radian))));
            drive::chassisController.turnAngleAsync(angleError * radian);
        }
        else
        {
            drive::chassisController.turnAngleAsync(angle);
        }
    }
    else
    {
        if (odom)
        {
            double angleError = std::atan2(sin(angle.convert(radian) - pose.heading.convert(radian)), cos((angle.convert(radian) - pose.heading.convert(radian))));
            drive::chassisController.turnAngle(angleError * radian);
            drive::chassisController.waitUntilSettled();
        }
        else
        {
            drive::chassisController.turnAngle(angle);
            drive::chassisController.waitUntilSettled();
        }
    }
    drive::chassisController.setMaxVelocity(200);
}

} // namespace drive

// Remove path to clear up memory
void removePaths(std::string path1)
{
    drive::profileController.removePath(path1);
}

// Remove 2 paths at once to clear up memory
void removePaths(std::string path1, std::string path2)
{
    drive::profileController.removePath(path1);
    drive::profileController.removePath(path2);
}