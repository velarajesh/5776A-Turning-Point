#ifndef MACRO_GUARD
#define MACRO_GUARD

#include "okapi/api.hpp"

using namespace okapi;

namespace macro
{

enum macroStates
{
    none = 'x',
    doubleShotNoWait = 'd',
    customShotDouble = 'c',
    singleShot = 's',
    anglerNearHigh = '1',
    anglerNearMid = '2',
    anglerFarMid = '3',
    anglerFarHigh = '4',
    scorePole = 'p',
};

extern macroStates currState;

extern void update();

extern void act(void *);

extern void resetSubsystems();

} // namespace macro

extern void customShotCall(int target1, int target2, bool shouldWait = false);

extern void customShotCall(int target1);

#endif