#ifndef INIT_GUARD
#define INIT_GUARD

#include "api.h"
#include "okapi/api.hpp"

using namespace okapi;

extern Controller controller;

extern const double joyDeadband;

enum alliances
{
    red = 'r',
    blue = 'b'
};

extern alliances alliance;

extern void initActTasks();

extern void waitUntilSettled(okapi::AbstractMotor &motor, double isAtTargetError, double isAtTargetDerivative, QTime isAtTargetTime);

extern pros::Task *driveActTask;
extern pros::Task *puncherActTask;
extern pros::Task *anglerActTask;
extern pros::Task *differentialActTask;
extern pros::Task *macroActTask;
extern pros::Task *visionTask;
extern pros::Task *updateTask;

extern void updateFunc(void *);

extern bool looperoni;

#endif