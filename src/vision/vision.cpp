// #include "main.h"
#include "init.hpp"
#include "ports.hpp"
#include "subsystems/drive.hpp"
#include "subsystems/angler.hpp"
#include "vision/vision.hpp"
// #include "vision/VisionController.hpp"
// #include "vision/visionReader.hpp"
// #include "vision/objDrawer.hpp"
// #include "vision/objContainer.hpp"
#include <sstream>
#include <iomanip>

// using namespace okapi;

namespace vision
{

pros::Vision vis(VISION, pros::E_VISION_ZERO_TOPLEFT);

lib7842::VisionReader reader(&vis);

bool anglerVision;
bool driveVision;
pros::vision_object_s_t currObj;

void init()
{
    pros::vision_signature_s_t RSig = pros::Vision::signature_from_utility(1, 8437, 10311, 9374, -579, 1, -289, 4.500, 1);
    // pros::vision_signature_s_t GSig = pros::Vision::signature_from_utility(2, -1129, -157, -643, -2571, -1497, -2034, 1.800, 1);
    // pros::vision_signature_s_t RSig = pros::Vision::signature_from_utility(1, 3977, 5257, 4617, 831, 2111, 1471, 3.000, 1);
    // pros::vision_signature_s_t GSig = pros::Vision::signature_from_utility(2, -1557, -1053, -1306, -1107, -85, -596, 1.600, 1);
    pros::vision_signature_s_t BSig = pros::Vision::signature_from_utility(2, -2675, -1807, -2241, 10365, 11423, 10894, 6.900, 0);
    vis.set_signature(1, &RSig);
    vis.set_signature(2, &BSig);
}

void loop(void *)
{
    drive::currState = drive::driveStates::yield;

    vis.set_exposure(150);
    // pros::vision_signature_s_t SIG_1 = {1, {1, 0, 0}, 2.800, -2129, -1369, -1749, 3221, 4401, 3811, 0, 0};
    // vis.set_signature(1, &SIG_1);
    // pros::vision_signature_s_t SIG_2 = {2, {1, 0, 0}, 7.600, 6823, 7385, 7104, -2009, -1543, -1776, 0, 0};
    // vis.set_signature(2, &SIG_2);

    // pros::vision_signature_s_t RSig = pros::Vision::signature_from_utility(1, 3977, 5257, 4617, 831, 2111, 1471, 3.000, 1);
    // pros::vision_signature_s_t GSig = pros::Vision::signature_from_utility(2, -1557, -1053, -1306, -1107, -85, -596, 1.600, 1);
    // vis.set_signature(1, &RSig);
    // vis.set_signature(2, &GSig);

    // pros::vision_color_code_t RFLAG = vision::vis.create_color_code(1, 2);

    lib7842::ObjContainer standout;

    // lib7842::ObjDrawer drawer(lv_scr_act());
    // drawer.withStyle(LV_COLOR_GRAY, LV_COLOR_WHITE);

    // drawer.withLayer(reader)
    //     .withStyle(VISION_OBJECT_ERR_SIG, LV_COLOR_YELLOW, LV_COLOR_WHITE)
    //     .withStyle(1, LV_COLOR_RED, LV_COLOR_WHITE)
    //     .withStyle(2, LV_COLOR_GREEN, LV_COLOR_WHITE);

    alliance = blue;

    while (true)
    {

        reader.reset();
        // reader.getAll();
        // if (alliance = red)
        // {
        //     reader.getSig(2); // looks for blue flag
        // }
        // if (alliance = blue)
        // {
        //     reader.getSig(1); // looks for red flag
        // }
        // reader.getCode(RFLAG);
        // reader.removeWith(lib7842::objAttr::area, 0, 5);
        // printf("counter: %f\n", reader.getCount());
        reader.sortBy(lib7842::objAttr::area);
        // drawer.draw();

        pros::delay(10);
    }
}

QAngle angToTarget(int x_pix)
{
    return (x_pix - 157.5) * 0.1898734 * degree;
}

} // namespace vision
