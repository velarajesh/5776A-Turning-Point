#ifndef DIFF_GUARD
#define DIFF_GUARD

#include "okapi/api.hpp"

using namespace okapi;

namespace differential
{

enum differentialStates
{
    notRunning = 'x',
    liftHold = 'h',
    liftUp = '1',
    liftDown = '2',
    intakeIn = 'i',
    intakeOut = 'o',
    ballBrake = 'b',
    ballDecel = 'd',
    intakeOutNY = 'n',
    liftPID = 't',
    yield = 'y'
};

extern differentialStates currState;

extern Motor diffLeft, diffRight;

extern pros::ADILineSensor line;

extern Potentiometer liftPot;

extern AverageFilter<5> liftPotFilter;

extern bool intakeHasBall;

extern int liftVal;
extern int liftTarget;

extern const double kP, kI, kD;

extern IterativePosPIDController liftController;

extern void update();

extern double liftToDegrees(int potValue);

extern void act(void *);
} // namespace differential

extern void runIntake(int speed);

#endif