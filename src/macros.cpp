#include "main.h"
#include "vision/vision.hpp"

using namespace okapi;

ControllerButton anglerNearHighBtn = controller[ControllerDigital::down];
ControllerButton anglerNearMidBtn = controller[ControllerDigital::up];
ControllerButton anglerFarMidBtn = controller[ControllerDigital::left];
ControllerButton anglerFarHighBtn = controller[ControllerDigital::right];

ControllerButton singleShotBtn = controller[ControllerDigital::L1];
ControllerButton doubleShotNearBtn = controller[ControllerDigital::L2];
ControllerButton doubleShotFarBtn = controller[ControllerDigital::X];

ControllerButton liftBtn = controller[ControllerDigital::B];
ControllerButton shiftBtn = controller[ControllerDigital::R1];

int macroTarget1; // target encoder values for angler to shoot first flag in macro
int macroTarget2; // target encoder value for angler to shoot second flag in macro

namespace macro
{

macroStates currState = none;

bool secondMove = false;

void update()
{
    if (anglerNearHighBtn.changedToPressed())
    {
        // if (currState == '1')
        // {
        //     currState = none;
        // }
        // else
        // {
        currState = anglerNearHigh;
        // }
    }
    if (anglerNearMidBtn.changedToPressed())
    {
        // if (currState == '2')
        // {
        //     currState = none;
        // }
        // else
        // {
        currState = anglerNearMid;
        // }
    }
    if (anglerFarMidBtn.changedToPressed())
    {
        // if (currState == '3')
        // {
        //     currState = none;
        // }
        // else
        // {
        currState = anglerFarMid;
        // }
    }
    if (anglerFarHighBtn.changedToPressed())
    {
        // if (currState == '4')
        // {
        //     currState = none;
        // }
        // else
        // {
        currState = anglerFarHigh;
        // }
    }
    if (singleShotBtn.changedToPressed())
    {
        // if (currState == 's')
        // {
        //     currState = none;
        //     puncher::currState = puncher::puncherStates::notRunning;
        // }
        // else
        // {
        currState = singleShot;
        // }
    }
    if (!shiftBtn.changedToPressed() && !shiftBtn.isPressed() && doubleShotNearBtn.changedToPressed())
    {
        // if (currState == 'c')
        // {
        //     cancelled = true;
        // }
        // else
        // {
        // cancelled = false;
        secondMove = false;
        customShotCall(0, 400);
        // }
    }
    if (!shiftBtn.changedToPressed() && !shiftBtn.isPressed() && doubleShotFarBtn.changedToPressed())
    {
        // if (currState == 'c')
        // {
        //     cancelled = true;
        // }
        // else
        // {
            // cancelled = false;
            secondMove = true;
            customShotCall(60, 380);
        // }
    }
    if (!shiftBtn.changedToPressed() && shiftBtn.isPressed() && doubleShotNearBtn.changedToPressed())
    {
        // if (currState == 'c')
        // {
        //     cancelled = true;
        // }
        // else
        // {
            // cancelled = false;
            secondMove = false;
            customShotCall(0, 0);
        // }
    }
    if (!shiftBtn.changedToPressed() && shiftBtn.isPressed() && doubleShotFarBtn.changedToPressed())
    {
        // if (currState == 'c')
        // {
        //     cancelled = true;
        // }
        // else
        // {
            // cancelled = false;
            secondMove = false;
            customShotCall(100, 420);
        // }
    }
    if (!shiftBtn.changedToPressed() && shiftBtn.isPressed() && liftBtn.changedToPressed())
    {
        // if (currState == 'p')
        // {
        //     cancelled = true;
        // }
        // else
        // {
            currState = scorePole;
        // }
    }
    // printf("Macro State: %c\n", currState);
} // namespace macro

void act(void *)
{
    while (true)
    {
        switch (currState)
        {
        case none: // macro is not activated
            break;
        case singleShot: // shoots a single flag with a single given target encoder value for that flag; for autonomous
            // drive::currState = drive::autoAim;

            drive::chassisController.setBrakeMode(AbstractMotor::brakeMode::hold);

            vision::vis.set_led(COLOR_LIGHT_BLUE);

            drive::chassisController.waitUntilSettled();

            puncher::currState = puncher::cocking;

            // while (!drive::chassisController.isSettled())
            // {
            //     pros::delay(2);
            // }
            // drive::chassisController.waitUntilSettled();

            // switches out of cocking when sensor value achieved

            // differential::currState = differential::intakeIn;
            // while (!puncher::puncherIsLoaded)
            // {
            //   pros::delay(2);
            // } // waits for puncher to load
            // differential::currState = differential::notRunning;

            // waitUntilSettled(angler::angler, 2, 5, 10_ms); // waits until angler to stop
            printf("Ang. PreShot: %i\n", (int)angler::angler.getPosition());

            puncher::currState = puncher::shooting;

            while (puncher::currState != puncher::cocking)
            {
                pros::delay(2);
            }

            angler::target = 0;
            angler::currState = angler::toTarget;

            drive::chassisController.setBrakeMode(AbstractMotor::brakeMode::coast);

            vision::vis.set_led(COLOR_ANTIQUE_WHITE);

            drive::currState = drive::notRunning;
            macro::currState = none;
            break;
        case customShotDouble: // shoots two flags with the specified target encoder values for those flags; for autonomous
            drive::currState = drive::yield;

            drive::chassisController.setBrakeMode(AbstractMotor::brakeMode::hold);

            vision::vis.set_led(COLOR_LIGHT_BLUE);

            puncher::currState = puncher::cocking;
            // switches out of cocking when sensor value achieved

            angler::target = macroTarget1;
            angler::currState = angler::toTarget;
            // will switch out of toTarget automatically when target reached

            // should automatically stop when ball loads into puncher

            // differential::currState = differential::intakeIn;
            while (!puncher::puncherIsLoaded)
            {
                pros::delay(2);
            } // waits for puncher to load
            differential::currState = differential::notRunning;

            waitUntilSettled(angler::angler, 5, 5, 20_ms); // waits until angler to stop

            drive::chassisController.waitUntilSettled();

            printf("Ang. PreShot 1: %i\n", (int)angler::angler.getPosition());
            puncher::currState = puncher::shooting;

            while (puncher::currState != puncher::cocking)
            {
                pros::delay(2);
            }

            if (secondMove){
            // printf("moved\n");
            drive::chassisController.setMaxVelocity(150);
            drive::chassisController.moveDistanceAsync(1_in);
            }

            vision::vis.set_led(COLOR_RED);

            angler::target = macroTarget2;
            angler::currState = angler::toTarget;

            differential::currState = differential::ballDecel;
            while (!puncher::puncherIsLoaded)
            {
                pros::delay(2);
            } // waits for puncher to load
            differential::currState = differential::notRunning;

            waitUntilSettled(angler::angler, 5, 5, 20_ms); // waits until angler to stop

            printf("Ang. PreShot 2: %i\n", (int)angler::angler.getPosition());
            puncher::currState = puncher::shooting;

            drive::chassisController.waitUntilSettled();
            drive::chassisController.setMaxVelocity(200);

            while (puncher::currState != puncher::cocking)
            {
                pros::delay(2);
            }
            angler::target = 0;
            angler::currState = angler::toTarget;

            vision::vis.set_led(COLOR_ANTIQUE_WHITE);
            drive::chassisController.setBrakeMode(AbstractMotor::brakeMode::coast);
            drive::currState = drive::notRunning;
            macro::currState = none;
            break;
        case doubleShotNoWait:
            drive::chassisController.setBrakeMode(AbstractMotor::brakeMode::hold);
            drive::currState = drive::yield;

            vision::vis.set_led(COLOR_LIGHT_BLUE);

            puncher::currState = puncher::cocking;
            // switches out of cocking when sensor value achieved

            angler::target = macroTarget1;
            angler::currState = angler::toTarget;
            // will switch out of toTarget automatically when target reached

            // should automatically stop when ball loads into puncher

            // while (!puncher::puncherIsLoaded)
            // {
            //   pros::delay(2);
            // } // waits for puncher to load

            if (!puncher::puncherIsLoaded)
            {
                differential::currState = differential::ballDecel;
                pros::delay(400);
            }

            if (!puncher::puncherIsLoaded)
            {
                drive::chassisController.setBrakeMode(AbstractMotor::brakeMode::coast);
                macro::currState = none;
                break; // if puncher not loaded do not shoot again
            }

            differential::currState = differential::notRunning;

            waitUntilSettled(angler::angler, 5, 5, 20_ms); // waits until angler to stop

            puncher::currState = puncher::shooting;

            // --------------------------------------------------------------- FIRST SHOT

            while (puncher::currState != puncher::cocking)
            {
                pros::delay(2);
            }

            angler::target = macroTarget2;
            angler::currState = angler::toTarget;

            differential::currState = differential::ballDecel;
            pros::delay(1000); // doesn't wait for ball to be loaded, because it may or may not be there

            if (!puncher::puncherIsLoaded)
            {
                drive::chassisController.setBrakeMode(AbstractMotor::brakeMode::coast);
                macro::currState = none;
                vision::vis.set_led(COLOR_ANTIQUE_WHITE);

                angler::target = 0;
                angler::currState = angler::toTarget;
                break; // if puncher not loaded do not shoot again
            }
            vision::vis.set_led(COLOR_RED);

            differential::currState = differential::notRunning;

            waitUntilSettled(angler::angler, 5, 5, 20_ms); // waits until angler to stop

            puncher::currState = puncher::shooting;

            while (puncher::currState != puncher::cocking)
            {
                pros::delay(2);
            }
            angler::target = 0;
            angler::currState = angler::toTarget;

            drive::chassisController.setBrakeMode(AbstractMotor::brakeMode::coast);

            vision::vis.set_led(COLOR_ANTIQUE_WHITE);

            drive::currState = drive::yield;

            macro::currState = none;
            break;
        case anglerNearHigh: // changes the angler to target the high flag from the close tile
            angler::target = 0;
            angler::currState = angler::toTarget;
            macro::currState = none;
            break;
        case anglerNearMid: // changes the angler to target the high flag from the middle tile
            angler::target = 400;
            angler::currState = angler::toTarget;
            macro::currState = none;
            break;
        case anglerFarMid: // changes the angler to target the middle flag from the far tile
            angler::target = 380;
            angler::currState = angler::toTarget;
            macro::currState = none;
            break;
        case anglerFarHigh: // Changes the angler to target the high flag from the far title
            angler::target = 60;
            angler::currState = angler::toTarget;
            macro::currState = none;
            break;
        case scorePole:
            // vision::vis.set_led(COLOR_LIGHT_BLUE);
            drive::currState = drive::yield;
            drive::chassisController.setMaxVelocity(50);
            drive::chassisController.moveDistanceAsync(-4_in);
            // drive::chassisController.waitUntilSettled();
            drive::chassisController.setMaxVelocity(200);

            differential::liftTarget = 3300; //2400
            // vision::vis.set_led(COLOR_RED);
            differential::currState = differential::liftPID;
            while (abs(3300 - differential::liftVal) > 100){
                pros::delay(2);
            }
            differential::currState = differential::notRunning;
            currState = none;
            break;
        }
        pros::delay(10);
    }
}

void resetSubsystems()
{
    drive::currState = drive::notRunning;
    puncher::currState = puncher::cocking;
    differential::currState = differential::notRunning;
}

} // namespace macro

// Double shot
void customShotCall(int target1, int target2, bool noWait)
{
    macroTarget1 = target1;
    macroTarget2 = target2;

    if (noWait)
    {
        macro::currState = macro::macroStates::doubleShotNoWait;
    }
    else
    {
        macro::currState = macro::macroStates::customShotDouble;
    }
}

// Single shot
void customShotCall(int target1)
{
    macroTarget1 = target1;

    macro::currState = macro::macroStates::singleShot;
}