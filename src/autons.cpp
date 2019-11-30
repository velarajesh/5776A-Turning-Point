 #include "main.h"

autonRoutines autonRoutine = notSelected;

void executeProgSkills()
{}
/*-------------------------------------------------------------------*/

void executeRedNear1() {
    drive::odometry.setPose({84_in, -14.25_in, 90_deg});
    

    differential::currState = differential::intakeIn;

    drive::ramBoi.moveTo({{84_in, -53.25_in, 90_deg}});

    pros::delay(0);

    drive::ramBoi.moveTo({{84_in, -17_in, 90_deg}});

    drive::turn(0_deg);

    customShotCall(0, 400, true);
    // Performs double shot

    while (macro::currState != macro::none)
    {
        pros::delay(10);
    }
    // Wait for the two shots to complete

    differential::currState = differential::intakeIn;

    drive::ramBoi.setCorrectionMode(RRLib::RamseteProfileController::CorrectionMode::Heading);

    drive::ramBoi.moveTo({{132_in, -15_in, 0_deg}});

    drive::ramBoi.moveTo({{120_in, -15_in, 0_deg}});

    drive::ramBoi.setCorrectionMode(RRLib::RamseteProfileController::CorrectionMode::Heading);

     if (puncher::puncherIsLoaded)
    {
        drive::turn(74_deg);
        // Turn right

        customShotCall(80, 450, true);
        // Performs double shot at center column

        while (macro::currState != macro::none)
        {
            pros::delay(10);
        }
        // Wait for the macro to complete
    }

    drive::turn(118_deg, 200);

    differential::currState = differential::intakeOutNY;

    // drive::ramBoi.setCorrectionMode(RRLib::RamseteProfileController::CorrectionMode::Heading);

    drive::ramBoi.moveTo({{108_in, -37.25_in, 90_deg}});

    // drive::ramBoi.setCorrectionMode(RRLib::RamseteProfileController::CorrectionMode::Heading);

    drive::turn(210_deg, 200);

    differential::liftTarget = 900;
    differential::currState = differential::liftPID;

    drive::chassisController.moveDistance(-30_in);

    differential::currState = differential::notRunning;

    drive::ramBoi.setCorrectionMode(RRLib::RamseteProfileController::CorrectionMode::Heading);
    drive::ramBoi.moveTo({{85_in, -58_in, 180_deg}});
    drive::ramBoi.setCorrectionMode(RRLib::RamseteProfileController::CorrectionMode::Heading);

    drive::chassisController.moveDistance(35_in);
}

/*-------------------------------------------------------------------*/

void executeRedNear2() {}

/*-------------------------------------------------------------------*/

void executeRedFar1() {
    drive::odometry.setPose({36_in, -14.25_in, 90_deg});


    drive::turn(270_deg, 100);

    drive::ramBoi.moveTo({{12_in, -53_in, 270_deg}});
    
    drive::chassisController.setMaxVelocity(50);
    drive::chassisController.moveDistanceAsync(-5_in);
    angler::target = 300;
    angler::currState = angler::toTarget;
    drive::chassisController.waitUntilSettled();
    drive::chassisController.setMaxVelocity(200);

    differential::liftTarget = 900;
    differential::currState = differential::liftPID;

    pros::delay(200);

    drive::ramBoi.moveTo({{25.5_in, -9_in, 270_deg}});

    macro::currState = macro::scorePole;
    while (macro::currState != macro::none)
    {
        pros::delay(10);
    }
    // Wait for the macro to complete

    pros::delay(1000);

    differential::currState = differential::notRunning;

    drive::chassisController.setMaxVelocity(50);
    drive::chassisController.moveDistance(-5_in);
    drive::chassisController.setMaxVelocity(200);

    differential::liftTarget = 0;
    differential::currState = differential::liftPID;

    drive::turn(90_deg, 200);

    differential::currState = differential::intakeIn;

    drive::ramBoi.moveTo({{34_in, -53.25_in, 90_deg}});

    angler::target = 0;
    angler::currState = angler::toTarget;

    drive::ramBoi.moveTo({{24_in, -48_in, 90_deg}});

    drive::turn(10_deg);

    customShotCall(100, 420, true);
    while (macro::currState != macro::none)
    {
        pros::delay(10);
    }
    // Wait for the macro to complete

    // drive::turn(0_deg);

    differential::currState = differential::intakeIn;

    drive::chassisController.moveDistance(40_in);
}

/*-------------------------------------------------------------------*/

void executeRedFar2() {
    drive::odometry.setPose({36_in, -14.25_in, 90_deg});


    drive::turn(270_deg, 100);

    drive::ramBoi.moveTo({{12_in, -53_in, 270_deg}});
    
    drive::chassisController.setMaxVelocity(50);
    drive::chassisController.moveDistanceAsync(-5_in);
    angler::target = 300;
    angler::currState = angler::toTarget;
    drive::chassisController.waitUntilSettled();
    drive::chassisController.setMaxVelocity(200);

    differential::liftTarget = 900;
    differential::currState = differential::liftPID;

    pros::delay(200);

    drive::ramBoi.moveTo({{25.5_in, -9_in, 270_deg}});

    macro::currState = macro::scorePole;
    while (macro::currState != macro::none)
    {
        pros::delay(10);
    }
    // Wait for the macro to complete

    pros::delay(1000);

    differential::currState = differential::notRunning;

    drive::chassisController.setMaxVelocity(50);
    drive::chassisController.moveDistance(-5_in);
    drive::chassisController.setMaxVelocity(200);

    differential::liftTarget = 0;
    differential::currState = differential::liftPID;

    drive::turn(90_deg, 200);

    differential::currState = differential::intakeIn;

    drive::ramBoi.moveTo({{34_in, -53.25_in, 90_deg}});

    angler::target = 0;
    angler::currState = angler::toTarget;

    drive::ramBoi.moveTo({{24_in, -48_in, 90_deg}});

    drive::turn(34.5_deg);

    customShotCall(30, 420, true);
    while (macro::currState != macro::none)
    {
        pros::delay(10);
    }
    // Wait for the macro to complete

    drive::turn(10_deg);

    differential::currState = differential::intakeIn;

    drive::chassisController.moveDistance(40_in);
}

/*-------------------------------------------------------------------*/

void executeBlueNear1() {
    drive::odometry.setPose({84_in, -129.75_in, 270_deg});
    

    differential::currState = differential::intakeIn;

    drive::ramBoi.moveTo({{84_in, -90.75_in, 270_deg}});

    pros::delay(0);

    drive::ramBoi.moveTo({{84_in, -127_in, 270_deg}});

    drive::turn(0_deg);

    customShotCall(0, 400, true);
    // Performs double shot

    while (macro::currState != macro::none)
    {
        pros::delay(10);
    }
    // Wait for the two shots to complete

    differential::currState = differential::intakeIn;

    drive::ramBoi.setCorrectionMode(RRLib::RamseteProfileController::CorrectionMode::Heading);

    drive::ramBoi.moveTo({{132_in, -129_in, 0_deg}});

    drive::ramBoi.moveTo({{120_in, -129_in, 0_deg}});

    drive::ramBoi.setCorrectionMode(RRLib::RamseteProfileController::CorrectionMode::Heading);

     if (puncher::puncherIsLoaded)
    {
        drive::turn(286_deg);
        // Turn right

        customShotCall(80, 450, true);
        // Performs double shot at center column

        while (macro::currState != macro::none)
        {
            pros::delay(10);
        }
        // Wait for the macro to complete
    }

    drive::turn(242_deg, 200);

    differential::currState = differential::intakeOutNY;

    // drive::ramBoi.setCorrectionMode(RRLib::RamseteProfileController::CorrectionMode::Heading);

    drive::ramBoi.moveTo({{108_in, -106.75_in, 270_deg}});

    // drive::ramBoi.setCorrectionMode(RRLib::RamseteProfileController::CorrectionMode::Heading);

    drive::turn(150_deg, 200);

    differential::liftTarget = 900;
    differential::currState = differential::liftPID;

    drive::chassisController.moveDistance(-30_in);

    differential::currState = differential::notRunning;

    drive::ramBoi.setCorrectionMode(RRLib::RamseteProfileController::CorrectionMode::Heading);
    drive::ramBoi.moveTo({{85_in, -86_in, 180_deg}});
    drive::ramBoi.setCorrectionMode(RRLib::RamseteProfileController::CorrectionMode::Heading);

    drive::chassisController.moveDistance(35_in);
}

/*-------------------------------------------------------------------*/

void executeBlueNear2() {}

/*-------------------------------------------------------------------*/

void executeBlueFar1() {
    drive::odometry.setPose({36_in, -129.75_in, 270_deg});


    drive::turn(90_deg, 100);

    drive::ramBoi.moveTo({{12_in, -91_in, 90_deg}});
    
    drive::chassisController.setMaxVelocity(50);
    drive::chassisController.moveDistanceAsync(-5_in);
    angler::target = 300;
    angler::currState = angler::toTarget;
    drive::chassisController.waitUntilSettled();
    drive::chassisController.setMaxVelocity(200);

    differential::liftTarget = 900;
    differential::currState = differential::liftPID;

    pros::delay(200);

    drive::ramBoi.moveTo({{25.5_in, -135_in, 90_deg}});
    macro::currState = macro::scorePole;
    while (macro::currState != macro::none)
    {
        pros::delay(10);
    }
    // Wait for the macro to complete

    pros::delay(1000);

    differential::currState = differential::notRunning;

    drive::chassisController.setMaxVelocity(50);
    drive::chassisController.moveDistance(-5_in);
    drive::chassisController.setMaxVelocity(200);

    differential::liftTarget = 0;
    differential::currState = differential::liftPID;

    drive::turn(270_deg, 200);

    differential::currState = differential::intakeIn;

    drive::ramBoi.moveTo({{34_in, -90.75_in, 270_deg}});

    angler::target = 0;
    angler::currState = angler::toTarget;

    drive::ramBoi.moveTo({{24_in, -96_in, 270_deg}});

    drive::turn(350_deg);

    customShotCall(100, 420, true);
    while (macro::currState != macro::none)
    {
        pros::delay(10);
    }
    // Wait for the macro to complete

    // drive::turn(0_deg);

    differential::currState = differential::intakeIn;

    drive::chassisController.moveDistance(40_in);
}

/*-------------------------------------------------------------------*/

void executeBlueFar2() {
    drive::odometry.setPose({36_in, -129.75_in, 270_deg});


    drive::turn(90_deg, 100);

    drive::ramBoi.moveTo({{12_in, -91_in, 90_deg}});
    
    drive::chassisController.setMaxVelocity(50);
    drive::chassisController.moveDistanceAsync(-5_in);
    angler::target = 300;
    angler::currState = angler::toTarget;
    drive::chassisController.waitUntilSettled();
    drive::chassisController.setMaxVelocity(200);

    differential::liftTarget = 900;
    differential::currState = differential::liftPID;

    pros::delay(200);

    drive::ramBoi.moveTo({{25.5_in, -135_in, 90_deg}});
    macro::currState = macro::scorePole;
    while (macro::currState != macro::none)
    {
        pros::delay(10);
    }
    // Wait for the macro to complete

    pros::delay(1000);

    differential::currState = differential::notRunning;

    drive::chassisController.setMaxVelocity(50);
    drive::chassisController.moveDistance(-5_in);
    drive::chassisController.setMaxVelocity(200);

    differential::liftTarget = 0;
    differential::currState = differential::liftPID;

    drive::turn(270_deg, 200);

    differential::currState = differential::intakeIn;

    drive::ramBoi.moveTo({{34_in, -90.75_in, 270_deg}});

    angler::target = 0;
    angler::currState = angler::toTarget;

    drive::ramBoi.moveTo({{24_in, -96_in, 270_deg}});

    drive::turn(325.5_deg);

    customShotCall(100, 420, true);
    while (macro::currState != macro::none)
    {
        pros::delay(10);
    }
    // Wait for the macro to complete

    drive::turn(350_deg);

    differential::currState = differential::intakeIn;

    drive::chassisController.moveDistance(40_in);
}