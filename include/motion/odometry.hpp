#pragma once

#include "motion/structs.hpp"
#include "okapi/api/chassis/controller/chassisScales.hpp"
#include "okapi/api/chassis/model/chassisModel.hpp"
#include "okapi/api/device/rotarysensor/continuousRotarySensor.hpp"

namespace RRLib {
class PoseEstimator {
public:
    virtual Pose getPose() = 0;
    virtual void setPose(Pose pose) = 0;
    virtual void update() = 0;
    virtual void printPose() = 0;
};

class TwoWheelOdometry : public PoseEstimator {
    Pose currentPose;
    std::shared_ptr<okapi::ReadOnlyChassisModel> model;
    okapi::ChassisScales scales;

    double lastLeft = 0.0;
    double lastRight = 0.0;

public:
    TwoWheelOdometry(
        std::shared_ptr<okapi::ReadOnlyChassisModel> model,
        okapi::ChassisScales scales);

    virtual Pose getPose() override;
    virtual void setPose(Pose pose) override;

    virtual void update() override;

    virtual void printPose() override;
};
}