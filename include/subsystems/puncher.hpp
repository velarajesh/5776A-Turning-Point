#ifndef PUNCHER_GUARD
#define PUNCHER_GUARD

#include "okapi/api.hpp"

using namespace okapi;

namespace puncher
{

enum puncherStates
{
    notRunning = 'x',
    shooting = 's',
    cocking = 'c',
    yield = 'y'
};

extern puncherStates currState;

extern Motor puncher;

extern AsyncPosIntegratedController puncherController;

extern pros::ADILineSensor linePuncher;

extern pros::ADILineSensor lineCockPuncher;

extern bool isLoaded();

extern bool puncherIsLoaded;
extern bool puncherIsFired;

extern bool fired();

extern void update();

extern void act(void *);

} // namespace puncher

#endif