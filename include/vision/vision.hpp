#ifndef VISION_GUARD
#define VISION_GUARD

// #include "okapi/api.hpp"
// #include "vision/VisionController.hpp"

// using namespace okapi;

#include "main.h"
#include "vision/visionReader.hpp"
#include "vision/objDrawer.hpp"

namespace vision
{

extern pros::Vision vis;

extern lib7842::VisionReader reader;

// VisionController *vision = nullptr;

extern bool anglerVision;
extern bool driveVision;

extern void init();

extern void loop(void *);

extern QAngle angToTarget(int x_pix);

} // namespace vision

#endif