#pragma once

#include "motion/odometry.hpp"
#include "motion/structs.hpp"
#include "okapi/api/chassis/model/chassisModel.hpp"
#include "okapi/api/control/async/asyncPositionController.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/units/QSpeed.hpp"
#include "okapi/api/units/QTime.hpp"
extern "C" {
#include "okapi/pathfinder/include/pathfinder.h"
}

#include <atomic>
#include <optional>
#include <tuple>

namespace RRLib {
struct RamseteGains {
    double zeta; // between 0.0 and 1.0, dampens
    double b; // greater than 0.0, increases correction
    double positionKD;

    RamseteGains(double zeta, double b, double positionKD = 0.0);
};

struct RamseteOutput {
    okapi::QSpeed velocity;
    okapi::QAngularSpeed omega;
};

RamseteOutput calculateRamsete(RamseteGains gains, Pose currentPose, Pose desiredPose, okapi::QSpeed desiredVelocity, okapi::QAngularSpeed desiredOmega);

// todo reversing automagically

using WaypointSet = std::vector<Pose>;

class RamseteProfileController : public okapi::AsyncPositionController<WaypointSet, Pose> {
public:
    enum class FitMode {
        Cubic,
        Quintic
    };

    enum class CorrectionMode {
        None,
        Ramsete,
        Heading
    };

private:
    RamseteGains gains;
    double headingkP;
    std::shared_ptr<okapi::ChassisModel> model;
    std::shared_ptr<PoseEstimator> poseEstimator;

    KinematicConstraints constraints;
    okapi::ChassisScales scales;
    okapi::AbstractMotor::GearsetRatioPair ratioPair;

    std::atomic<okapi::QTime> timestep = 10_ms;
    std::atomic_bool disabled = false;
    std::atomic_int currentStep = -1;
    pros::Task* task = nullptr;
    std::atomic_bool isExecuting = false;
    std::atomic_bool deferringMove = false;
    CorrectionMode correctionMode{ CorrectionMode::None };

    FitMode currentFitMode{ FitMode::Cubic };
    int samples{ PATHFINDER_SAMPLES_FAST };

    struct WheelVelocities {
        okapi::QAngularSpeed left;
        okapi::QAngularSpeed right;
    };

    struct GeneratedProfile {
        Segment* path;
        int length;
        bool needsReversing;
    };

    std::optional<GeneratedProfile> generatedProfile = std::nullopt;
    std::optional<WaypointSet> currentPath = std::nullopt;
    std::optional<Pose> lastMeasuredPose = std::nullopt;
    std::optional<Pose> lastError = std::nullopt;
    std::vector<std::function<void(Pose)>> newTargetCallbacks;

    okapi::QAngularSpeed linearToRotational(okapi::QSpeed linearSpeed) const;
    okapi::QAngularSpeed calculateDesiredOmega();
    std::tuple<okapi::QAngularSpeed, okapi::QAngularSpeed> calculateWheelVelocitiesRamsete();
    std::tuple<okapi::QAngularSpeed, okapi::QAngularSpeed> calculateWheelVelocitiesHeadingCorrect();
    std::tuple<okapi::QAngularSpeed, okapi::QAngularSpeed> calculateWheelVelocitiesBasic();
    std::tuple<okapi::QAngularSpeed, okapi::QAngularSpeed> calculateWheelVelocities();
    okapi::QAngle constrainAngle(okapi::QAngle in, okapi::QAngle min = -180_deg, okapi::QAngle max = 180_deg) const;
    bool calculateIfReversed();

    static void trampoline(void* instance);
    void updateError();
    void taskLoop();
    void generateProfile();
    void executePath();

public:
    RamseteProfileController(
        RamseteGains gains,
        double headingkP,
        std::shared_ptr<okapi::ChassisModel> model,
        std::shared_ptr<PoseEstimator> poseEstimator,
        KinematicConstraints constraints,
        okapi::ChassisScales scales,
        okapi::AbstractMotor::GearsetRatioPair ratioPair);

    void setTarget(WaypointSet path) override;
    void controllerSet(WaypointSet path) override;
    WaypointSet getTarget() override;

    // gets difference between latest target and current pose
    Pose getError() const override;

    void waitUntilSettled() override;
    bool isSettled() override;

    void reset() override;
    void deferNext();
    void go();
    void goAndWait();
    bool isDeferred();

    bool isDisabled() const override;
    void flipDisable() override;
    void flipDisable(bool isDisabled) override;
    void setCorrectionMode(CorrectionMode mode);
    CorrectionMode getCorrectionMode();
    void setTimestep(okapi::QTime timestep);
    okapi::QTime getTimestep();
    void setSamples(int samples);
    int getSamples();
    void setFitMode(FitMode mode);

    void onNewTarget(std::function<void(Pose)> cb);
    void move(WaypointSet path);
    void move(WaypointSet path, okapi::QTime timestep);
    void moveTo(WaypointSet path);
    void moveTo(Pose pose);

    void moveAsync(WaypointSet path);
    void moveAsync(WaypointSet path, okapi::QTime timestep);
    void moveToAsync(WaypointSet path);
    void moveToAsync(Pose pose);
    // todo more stuff
};
}
