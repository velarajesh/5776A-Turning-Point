#include "motion/odometry.hpp"

namespace RRLib {
using namespace okapi;
using namespace okapi::literals;

TwoWheelOdometry::TwoWheelOdometry(std::shared_ptr<ReadOnlyChassisModel> model, ChassisScales scales)
    : currentPose(0_m, 0_m, 0_rad)
    , model(model)
    , scales(scales) {}

void TwoWheelOdometry::update() {
    auto newSensorValues = model->getSensorVals();

    double deltaLeft = (newSensorValues[0] - lastLeft) / scales.straight;
    double deltaRight = (newSensorValues[1] - lastRight) / scales.straight;

    lastLeft = newSensorValues[0];
    lastRight = newSensorValues[1];

    double deltaAvg = (deltaLeft + deltaRight) / 2.0;
    double deltaTheta = (deltaLeft - deltaRight) / scales.wheelbaseWidth.convert(meter);

    double r1 = deltaTheta == 0 ? 0 : deltaAvg / deltaTheta;

    PositionVector positionUpdate(
        meter * (deltaTheta == 0 ? deltaAvg : (r1 * std::sin(deltaTheta))),
        meter * (deltaTheta == 0 ? 0 : (r1 - r1 * std::cos(deltaTheta))));

    // todo not be dumb
    positionUpdate.rotateSelf(currentPose.heading);
    positionUpdate.setSelf(PositionVector(positionUpdate.getX(), -positionUpdate.getY())); // this is gross fix asap
    currentPose.position.addSelf(positionUpdate);
    currentPose.turn(radian * deltaTheta);
}

Pose TwoWheelOdometry::getPose() {
    return currentPose;
}

void TwoWheelOdometry::setPose(Pose p) {
    currentPose = p;
}

void TwoWheelOdometry::printPose() {
    printf("X: %.2f | Y: %.2f | A: %.2f\n", currentPose.position.getX().convert(inch), currentPose.position.getY().convert(inch), currentPose.heading.convert(degree));
}
}