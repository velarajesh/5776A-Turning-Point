#include "main.h"

using namespace okapi;

ControllerButton liftUpBtn = controller[ControllerDigital::A];
ControllerButton liftDownBtn = controller[ControllerDigital::Y];
ControllerButton intakeInBtn = controller[ControllerDigital::R1];
ControllerButton intakeOutBtn = controller[ControllerDigital::R2];

ControllerButton liftHoldBtn = controller[ControllerDigital::B];

namespace differential
{
differentialStates currState;

Motor diffLeft(DIFF_L, true, AbstractMotor::gearset::green);
Motor diffRight(DIFF_R, false, AbstractMotor::gearset::green);

Timer ballBrakeTimer;
pros::ADILineSensor line(SPORT_INTAKE);

Potentiometer liftPot(SPORT_LIFT);
AverageFilter<5> liftPotFilter;
AverageFilter<5> intakeSensorFilter;

bool intakeHasBall;
bool intakeHadBall;

int liftVal;
int liftTarget;

const double kP = 0.0008;
const double kI = 0.00;
const double kD = 0;
const double kayF = 0.15;

IterativePosPIDController liftController = IterativeControllerFactory::posPID(kP, kI, kD, 0, std::make_unique<AverageFilter<3>>());

bool hasBall()
{
    if (line.get_value() < 2200)
    {
        return true;
    }
    return false;
}

bool intakeRising()
{
    if (intakeHasBall && !intakeHadBall)
    {
        return true;
    }
    return false;
}

bool intakeFalling()
{
    if (intakeHadBall && !intakeHasBall)
    {
        return true;
    }
    return false;
}

void update()
{
    liftVal = liftPotFilter.filter(liftPot.get());
    intakeHasBall = hasBall();

    // Automatic state checkers
    if ((currState == intakeIn || currState == ballDecel) && puncher::puncherIsLoaded && intakeRising())
    {
        currState = ballBrake;
        ballBrakeTimer.getDt();
    }
    if (currState == intakeIn && !puncher::puncherIsLoaded && intakeHasBall)
    {
        currState = ballDecel;
    }
    if (currState == ballDecel && puncher::puncherIsLoaded)
    {
        currState = intakeIn;
    }
    if (currState == ballBrake /* && ballBrakeTimer.readDt() >= 5_ms*/ && (/*(intakeRising() ||*/ ballBrakeTimer.readDt() >= 25_ms))
    {
        currState = notRunning;
    }
    // if ((liftVal >= 3900 || liftVal <= 11) && (currState == liftUp || currState == liftDown))
    // {
    //     currState = notRunning;
    // }

    // User input handlers
    if (liftUpBtn.changedToReleased())
    {
        currState = liftHold;
    }
    if (liftUpBtn.isPressed() && !liftDownBtn.isPressed())
    {
        currState = liftUp;
    }
    if (liftDownBtn.isPressed() && !liftUpBtn.isPressed())
    {
        currState = liftDown;
    }
    if ((currState == notRunning || currState == yield) && intakeInBtn.changedToPressed() && !intakeOutBtn.isPressed())
    {
        currState = intakeIn;
    }
    if ((currState == intakeIn || currState == ballDecel) && intakeInBtn.changedToPressed() && !intakeOutBtn.isPressed())
    {
        currState = notRunning;
    }
    if (intakeOutBtn.isPressed() && !intakeInBtn.isPressed())
    {
        currState = intakeOut;
    }
    if (liftHoldBtn.changedToPressed() && !intakeInBtn.isPressed())
    {
        // Engage lift PID if B button is pressed.
        liftTarget = 1050;
        currState = liftPID;
    }

    if (intakeRising())
    {
        printf("rising\n");
    }

    if (intakeFalling())
    {
        printf("falling\n");
    }

    // printf("%d\n", line.get_value());

    intakeHadBall = intakeHasBall;
}

void act(void *)
{
    double power;

    while (true)
    {
        liftController.setTarget(liftVal);

        switch (currState)
        {
        case notRunning:
            liftController.flipDisable(true);
            diffLeft.setBrakeMode(AbstractMotor::brakeMode::coast);
            diffRight.setBrakeMode(AbstractMotor::brakeMode::coast);
            diffLeft.moveVoltage(0);
            diffRight.moveVoltage(0);
            break;
        case liftUp:
            liftController.flipDisable(true);
            if (liftVal <= 3700)
            {
                diffLeft.moveVoltage(12000);
                diffRight.moveVoltage(12000);
            }
            liftTarget = liftVal;
            currState = liftHold;
            break;
        case liftDown:
            liftController.flipDisable(true);
            if (liftVal >= 40)
            {
                diffLeft.moveVoltage(-12000);
                diffRight.moveVoltage(-12000);
            }
            // liftTarget = liftVal;
            currState = notRunning;
            break;
        case intakeIn:
            liftController.flipDisable(true);
            diffLeft.moveVoltage(-12000);
            diffRight.moveVoltage(12000);
            break;
        case intakeOut:
            liftController.flipDisable(true);
            // Move motors at 83.3% power.
            diffLeft.moveVoltage(10000);
            diffRight.moveVoltage(-10000);
            currState = notRunning;
            break;
        case liftHold:
            liftController.flipDisable(true);
            // Engage PID for lift.
            // diffLeft.setBrakeMode(AbstractMotor::brakeMode::hold);
            // diffRight.setBrakeMode(AbstractMotor::brakeMode::hold);
            // diffLeft.moveVelocity(0);
            // diffRight.moveVelocity(0);
            liftController.flipDisable(false);
            power = -12000 * (kayF * cos((liftToDegrees(liftVal) * PI / 180)));
            diffLeft.moveVoltage(power);
            diffRight.moveVoltage(power);
            printf("power: %f | %f | \n", power, liftController.getError());
            break;
        case ballBrake:
            liftController.flipDisable(true);
            // Outtake without yielding control to notRunning state.
            diffLeft.moveVoltage(12000);
            diffRight.moveVoltage(-12000);
            break;
        case ballDecel:
            liftController.flipDisable(true);
            // Run intake at 83.3% of normal speed to prevent ball
            // from flying out.
            diffLeft.moveVoltage(-6500);
            diffRight.moveVoltage(6500);
            break;
        case intakeOutNY:
            liftController.flipDisable(true);
            // Keep outtaking to flip caps. For use in auton.
            diffLeft.moveVoltage(10000);
            diffRight.moveVoltage(-10000);
            break;
        case liftPID:
            // if (liftVal <= liftTarget)
            // {
            //     diffLeft.moveVoltage(-10000);
            //     diffRight.moveVoltage(-10000);
            // }
            // if (liftVal >= liftTarget)
            // {
            //     diffLeft.moveVoltage(10000);
            //     diffRight.moveVoltage(10000);
            // }
            liftController.flipDisable(false);
            power = -12000 * (liftController.step(liftTarget) + (kayF * cos((liftToDegrees(liftVal) * PI / 180))));
            diffLeft.moveVoltage(power);
            diffRight.moveVoltage(power);
            // printf("power: %f | %f | \n", power, liftController.getError());
            break;
        case yield:
            liftController.flipDisable(true);
            break;
        }
        pros::delay(10);
    }
}

double liftToDegrees(int potValue)
{
    double angle = (4200 - potValue) / 18.889;
    return angle;
}

} // namespace differential

void runIntake(int speed)
{
    differential::diffLeft.moveVelocity(-1 * speed);
    differential::diffRight.moveVelocity(1 * speed);
}
