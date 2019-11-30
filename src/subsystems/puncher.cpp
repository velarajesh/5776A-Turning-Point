#include "main.h"

using namespace okapi;

namespace puncher
{

puncherStates currState;

Motor puncher(PUNCHER, true, AbstractMotor::gearset::red);

pros::ADILineSensor lineP(SPORT_PUNCHER);

pros::ADILineSensor lineCock(SPORT_COCKER);

bool puncherIsLoaded;
bool puncherIsFired;

// Detect whether puncher has pulled back to properly shoot ball
bool isCocked()
{
    if (lineCock.get_value() > 2300 && puncher.get_torque() > 1)
    {
        // printf("is cocked\n");
        return true;
    }
    return false;
}

// Detect whether puncher currently has a ball that it can shoot
bool isLoaded()
{
    if (lineP.get_value() < 1500)
    {
        return true;
    }
    return false;
}

bool isFired()
{
    if (puncher.get_torque() < .3) // Must be a low value.
    {
        return true;
    }
    return false;
}

void update()
{
    puncherIsLoaded = isLoaded();
    puncherIsFired = isFired();
}

void act(void *)
{
    while (true)
    {
        // printf("%d\n", lineCock.get_value());
        switch (currState)
        {
        case notRunning:
            // Immobilize puncher
            // printf("shooting, %f\n", puncher.get_torque());
            puncher.setBrakeMode(AbstractMotor::brakeMode::coast);
            puncher.moveVoltage(0);
            break;
        case shooting:
            printf("shooting, %f\n", puncher.get_torque());
            // Turn puncher gear to shoot the ball
            puncher.moveVoltage(12000);
            //Automatically cock the puncher to prepare for next shot.
            pros::delay(30);
            if (puncherIsFired)
            {
                currState = cocking;
            }
            break;
        case cocking:
            if (!isCocked())
            {
                printf("cocking, %f\n", puncher.get_torque());
                puncher.moveVoltage(12000);
            }
            else
            {
                // printf("is cocked\n");
                puncher.moveVoltage(0);
                currState = notRunning;
            }
            break;
        case yield:
            // Empty case to allow full control of puncher in macro.
            break;
        }
        // printf("puncher line: %d\n", lineP.get_value());
        pros::delay(10);
    }
}

} // namespace puncher