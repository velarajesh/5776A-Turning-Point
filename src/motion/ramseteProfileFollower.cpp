#include "motion/ramseteProfileFollower.hpp"

#include "okapi/api/util/mathUtil.hpp"

namespace RRLib {
using namespace okapi;

RamseteOutput calculateRamsete(RamseteGains gains, Pose currentPose, Pose desiredPose, QSpeed desiredVelocity, QAngularSpeed desiredOmega) {
    auto positionError = PositionVector::subtract(desiredPose.position, currentPose.position);

    auto theta = currentPose.heading.convert(radian);
    auto thetaDiff = desiredPose.heading.convert(radian) - theta;

    double e1 = std::cos(theta) * positionError.getX().convert(meter) + std::sin(theta) * positionError.getY().convert(meter);
    double e2 = -std::sin(theta) * positionError.getX().convert(meter) + std::cos(theta) * positionError.getY().convert(meter);
    double e3 = std::atan2(std::sin(thetaDiff), std::cos(thetaDiff));

    double k1 = 2 * gains.zeta * std::sqrt((std::pow(desiredOmega.convert(radps), 2) + gains.b * std::pow(desiredVelocity.convert(mps), 2)));
    double k2 = gains.b;
    double k3 = k1;

    double sinc = e3 == 0 ? 1 : std::sin(e3) / e3;

    double u1 = -k1 * e1;
    double u2 = -k2 * desiredVelocity.convert(mps) * sinc * e2 - k3 * e3;

    auto velocity = desiredVelocity * std::cos(e3) - u1 * mps;
    auto omega = desiredOmega - u2 * radps;

    return { velocity, omega };
}

RamseteGains::RamseteGains(double zeta, double b, double positionKD)
    : zeta(zeta)
    , b(b)
    , positionKD(positionKD) {}

RamseteProfileController::RamseteProfileController(
    RamseteGains gains,
    double headingkP,
    std::shared_ptr<ChassisModel> model,
    std::shared_ptr<PoseEstimator> poseEstimator,
    KinematicConstraints constraints,
    ChassisScales scales,
    AbstractMotor::GearsetRatioPair ratioPair)
    : gains(gains)
    , headingkP(headingkP)
    , model(model)
    , poseEstimator(poseEstimator)
    , constraints(constraints)
    , scales(scales)
    , ratioPair(ratioPair) {
    task = new pros::Task(trampoline, this);
}

QAngularSpeed RamseteProfileController::calculateDesiredOmega() {
    auto profile = generatedProfile.value();
    auto segment = profile.path[currentStep];

    auto pointA = (currentStep < profile.length - 1) ? profile.path[currentStep + 1] : segment;
    auto pointB = (currentStep < profile.length - 1) ? segment : profile.path[currentStep - 1];

    auto headingDifference = constrainAngle(radian * (pointA.heading - pointB.heading));

    return radps * (headingDifference.convert(radian) / segment.dt);
}

void RamseteProfileController::setCorrectionMode(CorrectionMode mode) {
    correctionMode = mode;
}

RamseteProfileController::CorrectionMode RamseteProfileController::getCorrectionMode() {
    return correctionMode;
}

std::tuple<QAngularSpeed, QAngularSpeed> RamseteProfileController::calculateWheelVelocitiesRamsete() {
    double width = scales.wheelbaseWidth.convert(meter);
    int reversing = generatedProfile.value().needsReversing ? -1 : 1;
    auto segment = generatedProfile.value().path[currentStep];

    auto poseError = getError();
    auto pose = poseEstimator->getPose();
    auto positionError = poseError.position;

    auto desiredVelocity = segment.velocity * mps * reversing;
    auto desiredOmega = calculateDesiredOmega();

    auto theta = -pose.heading.convert(radian);

    double e1 = std::cos(theta) * positionError.getX().convert(meter) + std::sin(theta) * positionError.getY().convert(meter);
    double e2 = -std::sin(theta) * positionError.getX().convert(meter) + std::cos(theta) * positionError.getY().convert(meter);
    double e3 = poseError.heading.convert(radian);

    double k1 = 2 * gains.zeta * std::sqrt((std::pow(desiredOmega.convert(radps), 2) + gains.b * std::pow(desiredVelocity.convert(mps), 2)));
    double k2 = gains.b;
    double k3 = k1;

    double sinc = e3 == 0 ? 1 : std::sin(e3) / e3;

    double u1 = -k1 * e1;
    double u2 = -k2 * desiredVelocity.convert(mps) * sinc * e2 - k3 * e3;

    auto velocity = desiredVelocity * std::cos(e3) - u1 * mps;
    auto omega = desiredOmega - u2 * radps;

    auto turnDifference = mps * (omega.convert(radps) * width / 2.0);

    return std::make_tuple<QAngularSpeed, QAngularSpeed>(
        linearToRotational(velocity - turnDifference), linearToRotational(velocity + turnDifference));
}

std::tuple<QAngularSpeed, QAngularSpeed> RamseteProfileController::calculateWheelVelocitiesBasic() {
    double width = scales.wheelbaseWidth.convert(meter);
    int reversing = generatedProfile.value().needsReversing ? -1 : 1;
    auto segment = generatedProfile.value().path[currentStep];

    auto poseError = getError();
    auto pose = poseEstimator->getPose();
    auto positionError = poseError.position;

    auto velocity = segment.velocity * mps * reversing;
    auto omega = calculateDesiredOmega();

    auto turnDifference = mps * (omega.convert(radps) * width / 2.0);

    return std::make_tuple<QAngularSpeed, QAngularSpeed>(
        linearToRotational(velocity - turnDifference), linearToRotational(velocity + turnDifference));
}

std::tuple<QAngularSpeed, QAngularSpeed> RamseteProfileController::calculateWheelVelocitiesHeadingCorrect() {
    double width = scales.wheelbaseWidth.convert(meter);
    int reversing = generatedProfile.value().needsReversing ? -1 : 1;
    auto segment = generatedProfile.value().path[currentStep];

    auto poseError = getError();
    auto pose = poseEstimator->getPose();
    auto positionError = poseError.position;

    auto velocity = segment.velocity * mps * reversing;
    auto omega = calculateDesiredOmega() + radps * (headingkP * poseError.heading.convert(radian));

    auto turnDifference = mps * (omega.convert(radps) * width / 2.0);
    pros::lcd::print(3, "Angular error %.2f", poseError.heading.convert(degree));
    pros::lcd::print(4, "Turn diff %.2f", turnDifference.convert(mps));

    return std::make_tuple<QAngularSpeed, QAngularSpeed>(
        linearToRotational(velocity - turnDifference), linearToRotational(velocity + turnDifference));
}

std::tuple<QAngularSpeed, QAngularSpeed> RamseteProfileController::calculateWheelVelocities() {
    switch (correctionMode) {
    case CorrectionMode::None:
        return calculateWheelVelocitiesBasic();

    case CorrectionMode::Ramsete:
        return calculateWheelVelocitiesRamsete();

    case CorrectionMode::Heading:
        return calculateWheelVelocitiesHeadingCorrect();
    }

    return std::make_tuple<QAngularSpeed, QAngularSpeed>(0 * radps, 0 * radps);
}

QAngularSpeed RamseteProfileController::linearToRotational(QSpeed linear) const {
    return (linear * (360_deg / (scales.wheelDiameter * 1_pi))) * ratioPair.ratio;
}

QAngle RamseteProfileController::constrainAngle(QAngle in, QAngle min, QAngle max) const {
    double out = in.convert(radian);

    while (out > max.convert(radian)) {
        out -= 2_pi;
    }

    while (out < min.convert(radian)) {
        out += 2_pi;
    }

    return out * radian;
}

void RamseteProfileController::onNewTarget(std::function<void(Pose)> callback) {
    newTargetCallbacks.push_back(callback);
}

void RamseteProfileController::trampoline(void* instance) {
    static_cast<RamseteProfileController*>(instance)->taskLoop();
}

void RamseteProfileController::taskLoop() {
    while (task->notify_take(true, TIMEOUT_MAX)) {
        generateProfile();

        while (deferringMove.load()) {
            pros::delay(1);
        }

        executePath();
        isExecuting = false;
    }
}

bool RamseteProfileController::calculateIfReversed() {
    auto path = currentPath.value();

    auto poseA = path[0];
    auto poseB = path[1];

    auto posDifference = PositionVector::subtract(poseB.position, poseA.position);
    posDifference.rotateSelf(-poseA.heading);

    return posDifference.getX().convert(meter) < 0;
}

void RamseteProfileController::generateProfile() {
    std::vector<Waypoint> pfWaypoints;
    auto path = currentPath.value();
    bool needsReversed = calculateIfReversed();

    auto finalPose = path[path.size() - 1];
    for (auto& cb : newTargetCallbacks) {
        cb(finalPose);
    }

    for (auto& waypoint : path) {
        auto heading = needsReversed ? waypoint.heading.convert(radian) + 1_pi : waypoint.heading.convert(radian);
        pfWaypoints.push_back(
            Waypoint{ waypoint.position.getX().convert(meter), waypoint.position.getY().convert(meter), heading });
    }

    auto fitMode = FIT_HERMITE_CUBIC;

    if (currentFitMode == FitMode::Quintic) {
        fitMode = FIT_HERMITE_QUINTIC;
    }

    TrajectoryCandidate candidate;

    int length = pathfinder_prepare(
        pfWaypoints.data(),
        static_cast<int>(pfWaypoints.size()),
        fitMode,
        samples,
        timestep.load().convert(second),
        constraints.maxVel,
        constraints.maxAccel,
        constraints.maxJerk,
        &candidate);

    if (candidate.length < 0) {
        throw std::runtime_error("Oof I think that's an impossible path...");
    }

    auto trajectory = new Segment[candidate.length];

    if (trajectory == nullptr) {
        // eek.
        if (candidate.laptr) {
            delete candidate.laptr;
        }

        if (candidate.saptr) {
            delete candidate.saptr;
        }

        throw std::runtime_error("Zoinkers! Looks like we couldn't allocate memory for the path. Either it has issues or you're out of memory!");
    }

    pathfinder_generate(&candidate, trajectory);

    // TODO CHECK REVERSING BOIS AND DO LOTS OF MATH AND HEADACHES EARLIER
    generatedProfile.emplace(GeneratedProfile{ trajectory, candidate.length, needsReversed });
}

void RamseteProfileController::executePath() {
    auto profile = generatedProfile.value();

    for (currentStep = 0; currentStep < profile.length && !isDisabled() && !pros::competition::is_disabled(); currentStep++) {
        // TODO: how about actually implmeneting ramsete
        // to done?
        updateError();
        auto velocities = calculateWheelVelocities();

        model->left(std::get<0>(velocities).convert(rpm) / toUnderlyingType(ratioPair.internalGearset));
        model->right(std::get<1>(velocities).convert(rpm) / toUnderlyingType(ratioPair.internalGearset));

        pros::delay(timestep.load().convert(millisecond));
    }

    model->stop();

    reset();
}

void RamseteProfileController::setTarget(WaypointSet path) {
    if (!isExecuting.load()) {
        currentPath = path;
        isExecuting = true;
        task->notify();
    }
}

void RamseteProfileController::controllerSet(WaypointSet path) {
    setTarget(path);
}

WaypointSet RamseteProfileController::getTarget() {
    return currentPath.value();
}

void RamseteProfileController::updateError() {
    auto profile = generatedProfile.value();
    // todo <= or <???
    if (currentStep > -1 && currentStep <= profile.length) {
        poseEstimator->update();
        auto currPose = poseEstimator->getPose();
        Pose poseVelocity;

        if (lastMeasuredPose.has_value()) {
            poseVelocity = Pose(
                PositionVector::subtract(currPose.position, lastMeasuredPose.value().position),
                currPose.heading - lastMeasuredPose.value().heading);
        }

        poseVelocity.position.multiplySelf(gains.positionKD);

        auto predictedPose = Pose(PositionVector::add(currPose.position, poseVelocity.position), constrainAngle(currPose.heading + poseVelocity.heading));

        auto pathStep = profile.path[currentStep];

        if (generatedProfile.value().needsReversing) {
            pathStep.heading = constrainAngle(radian * (pathStep.heading - 1_pi)).convert(radian);
        }

        PositionVector desiredPosition(pathStep.x * meter, pathStep.y * meter);

        auto posError = PositionVector::subtract(desiredPosition, predictedPose.position);
        auto headingError = constrainAngle(radian * (pathStep.heading + predictedPose.heading.convert(radian)));

        lastMeasuredPose = currPose;

        lastError = Pose(posError, headingError);
    } else {
        lastError.reset();
    }
}

Pose RamseteProfileController::getError() const {
    return lastError.value_or(Pose());
}

void RamseteProfileController::waitUntilSettled() {
    while (!isSettled()) {
        pros::delay(10);
    }
}

bool RamseteProfileController::isSettled() {
    return isDisabled() || !isExecuting.load();
}

void RamseteProfileController::reset() {
    isExecuting = false;
    currentStep = -1;

    if (generatedProfile.has_value() && generatedProfile.value().path != nullptr) {
        delete generatedProfile.value().path;
    }

    currentPath.reset();
    generatedProfile.reset();
    lastMeasuredPose.reset();
}

bool RamseteProfileController::isDisabled() const {
    return disabled.load();
}

void RamseteProfileController::flipDisable() {
    disabled = !disabled.load();
}

void RamseteProfileController::flipDisable(bool isDisabled) {
    disabled = isDisabled;
}

void RamseteProfileController::setTimestep(QTime newTimestep) {
    timestep = newTimestep;
}

QTime RamseteProfileController::getTimestep() {
    return timestep;
}

void RamseteProfileController::setSamples(int sampleCount) {
    samples = sampleCount;
}

int RamseteProfileController::getSamples() {
    return samples;
}

void RamseteProfileController::setFitMode(FitMode mode) {
    currentFitMode = mode;
}

void RamseteProfileController::moveAsync(WaypointSet path) {
    flipDisable(false);
    setTarget(path);
}

void RamseteProfileController::moveAsync(WaypointSet path, QTime timestep) {
    setTimestep(timestep);
    moveAsync(path);
}

void RamseteProfileController::moveToAsync(WaypointSet path) {
    auto wps = path;
    auto currPose = poseEstimator->getPose();
    currPose.heading = currPose.heading * -1;
    wps.insert(wps.begin(), currPose);
    moveAsync(wps);
}

void RamseteProfileController::moveToAsync(Pose pose) {
    WaypointSet points;
    points.push_back(pose);
    moveToAsync(points);
}

void RamseteProfileController::move(WaypointSet path) {
    flipDisable(false);
    setTarget(path);
    waitUntilSettled();
    reset();
}

void RamseteProfileController::move(WaypointSet path, QTime timestep) {
    setTimestep(timestep);
    move(path);
}

void RamseteProfileController::moveTo(WaypointSet path) {
    auto wps = path;
    auto currPose = poseEstimator->getPose();
    currPose.heading = currPose.heading * -1;
    wps.insert(wps.begin(), currPose);
    move(wps);
}

void RamseteProfileController::moveTo(Pose pose) {
    WaypointSet points;
    points.push_back(pose);
    moveTo(points);
}

void RamseteProfileController::deferNext() {
    deferringMove = true;
}

bool RamseteProfileController::isDeferred() {
    return deferringMove.load();
}

void RamseteProfileController::go() {
    deferringMove = false;
}

void RamseteProfileController::goAndWait() {
    go();
    waitUntilSettled();
}
}