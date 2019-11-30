#ifndef DRIVE_GUARD
#define DRIVE_GUARD

#include "okapi/api.hpp"
#include "motion/odometry.hpp"
#include "motion/ramseteProfileFollower.hpp"
#include "motion/structs.hpp"
#include "motion/adaptivepurepursuit.hpp"

using namespace okapi;

namespace drive
{

enum driveStates
{
    notRunning = 'x',
    running = 'r',
    autoAim = 'a',
    yield = 'y'
};

extern driveStates currState;

extern Motor driveR1, driveR2, driveL1, driveL2;

extern ChassisControllerIntegrated chassisController;

extern AsyncMotionProfileController profileController;

extern pathfollowing::AdaptivePurePursuit appc;

extern RRLib::RamseteProfileController ramBoi;
extern RRLib::TwoWheelOdometry odometry;

extern int largestObjX;

extern IterativePosPIDController aimController;

extern void update();

extern void act(void *);

extern void turn(QAngle angle, double maxVel = 100, bool odom = true, bool async = false);

} // namespace drive

extern void removePaths(std::string path1);
extern void removePaths(std::string path1, std::string path2);

#endif