#include "main.h"
// #include "config.hpp"

#include "motion/adaptivepurepursuit.hpp"
#include "subsystems/drive.hpp"

namespace pathfollowing
{
AdaptivePurePursuit::AdaptivePurePursuit(
	std::unique_ptr<okapi::IterativePosPIDController> straight,
	std::unique_ptr<okapi::IterativePosPIDController> turn,
	int lookahead, double lookaheadKf) : straightController(std::move(straight)),
										 turnController(std::move(turn)),
										 mainLookahead(lookahead),
										 lookahead(lookahead),
										 lookaheadKf(lookaheadKf),
										 path(nullptr),
										 direction(1),
										 angleTarget(0_deg),
										 distanceSettledUtil(SettledUtilFactory::create(1.5, /*0.2*/ 99999, 50_ms)),
										 angularSettledUtil(SettledUtilFactory::create(9999, /*0.05*/ 99999, /*50_ms*/ 50_ms)),
										 _isSettled(false),
										 isLooping(false),
										 taskHandle(nullptr)

{
	// i commented this out because I think if it's globally constructed it might gay?
	// it's okay though i added a call to this in initiailize
	// startThread();
}

void AdaptivePurePursuit::setPath(path::Path *path)
{
	this->path = path;
	int look = path->getLookahead();
	lookahead = (look < 0) ? mainLookahead : look;
}

void AdaptivePurePursuit::setLookahead(int lookahead)
{
	this->lookahead = lookahead;
}

QAngle AdaptivePurePursuit::calculateAngleError(QAngle pV, QAngle setpoint)
{
	if (setpoint.convert(radian) < PI && pV.convert(radian) > PI)
	{
		pV = radian * (pV.convert(radian) - 2_pi);
	}
	double error = setpoint.convert(radian) - pV.convert(radian);
	return radian * (error > PI ? error - 2_pi : error);
}

void AdaptivePurePursuit::ramseteLoop()
{
	using namespace okapi;

	if (pros::competition::is_disabled())
	{
		isLooping = false;
	}

	// auto model = robot.getModel();
	// auto pose = model.getCurrentPose();

	// auto currHeading = QAngle(pose.theta);

	auto pose = drive::odometry.getPose();

	path::Point robotPosition = {pose.position.getX(), pose.position.getY()};
	path::PointAndDistance closestPointAndDistance = path->getClosestPointAndDistance(robotPosition);

	int newLookahead = lookahead - (closestPointAndDistance.distance.convert(inch) * lookaheadKf);
	newLookahead = (newLookahead < 0) ? 1 : newLookahead;

	int requiredPosition = closestPointAndDistance.point.t;
	target = path->pointAt(requiredPosition + newLookahead);
	// path->lookingAt(requiredPosition + newLookahead);

	double distTolookaheadPoint =
		sqrt(pow(target.x.convert(inch) - robotPosition.x.convert(inch), 2) + pow(target.y.convert(inch) - robotPosition.y.convert(inch), 2));

	straightController->setTarget(0);

	QAngle bearing = std::atan2((this->target.x.convert(inch) - robotPosition.x.convert(inch)),
								(this->target.y.convert(inch) - robotPosition.y.convert(inch))) *
					 radian;

	while (bearing.convert(radian) < 0)
	{
		bearing = radian * (bearing.convert(radian) + 2_pi);
	}

	QAngle currBearingAngle = std::atan2(std::sin(pose.heading.convert(radian)), std::cos(pose.heading.convert(radian))) * radian;

	double angleError = calculateAngleError(currBearingAngle, bearing).convert(radian);

	// NOTE
	// WE NEED TO FIGURE OUT A WAY TO TUNE THIS TO BE GOOD
	double areYouThere = false;
	if (target.t >= path->getResolution() && distTolookaheadPoint < 3)
	{
		// printf("Doing the thing at the end\n");
		double headingAtPoint = getHeadingAtPoint(target.t);
		double bearingAtPoint = atan2(std::sin(headingAtPoint), std::cos(headingAtPoint));
		angleError = calculateAngleError(currBearingAngle, bearingAtPoint * radian).convert(radian);
	}
	// else
	// {
	// 	// printf("target t: %d, path res: %d\n", target.t, path->getResolution());
	// }

	// if (target.t == path->getResolution())
	// {
	direction = 1;
	if (angleError * 180.0 / PI > 90)
	{
		angleError -= PI;
		// angleError *= -1;
		direction *= -1;
	}
	else if (angleError * 180.0 / PI < -90)
	{
		angleError += PI;
		// angleError *= -1;
		direction *= -1;
	}
	// }

	turnController->setTarget(0);

	// ramsete math
	double zeta = 0.00;
	double beta = 0.00;

	double theta = currBearingAngle.convert(radian);

	double x_d = target.x.convert(inch);
	double y_d = target.y.convert(inch);

	double x_curr = robotPosition.x.convert(inch);
	double y_curr = robotPosition.y.convert(inch);

	double e1 = std::sin(theta) * (x_d - x_curr) + std::cos(theta) * (y_d - y_curr);
	double e2 = -std::cos(theta) * (x_d - x_curr) + std::sin(theta) * (y_d - y_curr);
	double e3 = angleError;

	double v_d = straightController->step(-distTolookaheadPoint);
	double w_d = turnController->step(-e3 * 180 / PI);

	double k1 = 2 * zeta * sqrt((pow(w_d, 2) + beta * pow(v_d, 2)));
	double sinc = e3 == 0 ? 1 : std::sin(e3) / e3;

	double v_out = v_d * std::cos(e3) + k1 * e1;

	double w_out = w_d + beta * sinc * e2 + k1 * e3;

	// robot.getModel().driveVector(v_out, w_out);
	drive::chassisController.driveVector(direction * v_out, w_out);
} // namespace pathfollowing

double AdaptivePurePursuit::getHeadingAtPoint(int T)
{
	auto point = path->pointAt(T);
	double targetPoseHeading;

	if (T - 1 >= 0)
	{
		//printf("Took target at end of path\n");
		auto preTarget = path->pointAt(T - 1);
		//printf("end: (%1.2f, %1.2f), preEnd: (%1.2f, %1.2f)\n", point.x.convert(inch), point.y.convert(inch), preTarget.x.convert(inch), preTarget.y.convert(inch));
		targetPoseHeading = atan2(point.x.convert(inch) - preTarget.x.convert(inch), point.y.convert(inch) - preTarget.y.convert(inch));
	}
	else
	{
		//printf("Took target at front of path\n");
		auto preTarget = path->pointAt(T + 1);
		targetPoseHeading = atan2(point.x.convert(inch) - preTarget.x.convert(inch), point.y.convert(inch) - preTarget.y.convert(inch));
	}

	//printf("target heading: %1.2f\n", targetPoseHeading * 180.0/1_pi);

	return targetPoseHeading;
}

path::Point AdaptivePurePursuit::getPointTarget()
{
	return target;
}

void AdaptivePurePursuit::checkIsSettled()
{
	// you should calculate distanceError and turnError inside of this function --> ok
	// oops sorry manas thought i was looking at a different function (ic)
	// update private bool isSettled logic here

	auto pose = drive::odometry.getPose();

	auto point = path->pointAt(path->getResolution());
	auto preFinalPoint = path->pointAt(path->getResolution() - 1);

	double finalAngleTarget = std::atan2(point.y.convert(okapi::inch) - preFinalPoint.y.convert(okapi::inch), point.x.convert(okapi::inch) - preFinalPoint.x.convert(okapi::inch));
	double angleError = atan2(sin(finalAngleTarget - pose.heading.convert(okapi::radian)), cos(finalAngleTarget - pose.heading.convert(okapi::radian)));

	double finalDistance = sqrt(pow((point.x.convert(okapi::inch) - pose.position.getX().convert(okapi::inch)), 2) + pow((point.y.convert(okapi::inch) - pose.position.getY().convert(okapi::inch)), 2));

	_isSettled = distanceSettledUtil.isSettled(finalDistance) && angularSettledUtil.isSettled(angleError * 180.0 / PI); // converted to degrees for ease of settle error, etc.
}

// bool AdaptivePurePursuit::isSettled()
// {
// 	path::Point endPoint = path->pointAt(path->getResolution());
// 	double distance = sqrt(pow(endPoint.x.convert(inch) - currentPose.position.getX().convert(inch), 2) + pow(endPoint.y.convert(inch) - currentPose.position.getY().convert(inch), 2));
// 	return (distance < 2);
// }

bool AdaptivePurePursuit::isSettled()
{
	return _isSettled;
}

void AdaptivePurePursuit::resetSettled()
{
	distanceSettledUtil.reset(); // that was me, checking API rn
	angularSettledUtil.reset();  // im just guessing this method exists not gonna lie

	_isSettled = false;
}

void AdaptivePurePursuit::waitUntilSettled(bool stopWhenSettled)
{
	while (isLooping && !_isSettled)
	{
		pros::delay(10);
	}

	resetSettled();

	// my code is so good it comments itself
	if (stopWhenSettled)
	{
		stop();
	}
}

void AdaptivePurePursuit::runPathAsync(path::Path *path)
{
	setPath(path);
	start();
}

void AdaptivePurePursuit::runPath(path::Path *path, bool curved)
{
	runPathAsync(path);
	waitUntilSettled();
}

void AdaptivePurePursuit::setKf(double kf)
{
	this->lookaheadKf = kf;
}

void AdaptivePurePursuit::setStraightGains(double kP, double kI, double kD)
{
	straightController->setGains(kP, kI, kD, 0.0);
}

void AdaptivePurePursuit::setTurnGains(double kP, double kI, double kD)
{
	turnController->setGains(kP, kI, kD, 0.0);
}

void AdaptivePurePursuit::runLoop()
{
	while (true)
	{
		if (isLooping && path != nullptr)
		{
			ramseteLoop();
			checkIsSettled(); // updates internal boolean `isSettled` which is then exposed through getter bool isSettled();

			// if (drive::appc.isSettled())
			// {
			// 	looperoni = false;
			// }
		}
		else
		{
			resetSettled();
		}
		pros::delay(10);
	}
}

void AdaptivePurePursuit::trampoline(void *instance)
{ // we're literally just calling the runLoop function
	static_cast<AdaptivePurePursuit *>(instance)->runLoop();
}

void AdaptivePurePursuit::startThread()
{
	taskHandle = new pros::Task(trampoline, static_cast<void *>(this));
}

void AdaptivePurePursuit::stopThread()
{
	taskHandle->remove();
	delete taskHandle;
}

void AdaptivePurePursuit::stop()
{
	isLooping = false;
	drive::chassisController.stop(); // reEEEEEEEE this'll do for now, far too lazy to restructure ngl
	resetSettled();
}

void AdaptivePurePursuit::start()
{
	isLooping = true;
}

} // namespace pathfollowing