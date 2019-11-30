#pragma once

#include "path/path.hpp"
#include "okapi/api.hpp"

namespace pathfollowing
{
class AdaptivePurePursuit
{
private:
  std::unique_ptr<okapi::IterativePosPIDController> straightController;
  std::unique_ptr<okapi::IterativePosPIDController> turnController;
  int mainLookahead;
  int lookahead;
  double lookaheadKf;
  path::Path *path;

  path::Point target;
  int direction;
  okapi::QAngle angleTarget;

  okapi::SettledUtil distanceSettledUtil;
  okapi::SettledUtil angularSettledUtil;

  // yes I do
  bool _isSettled;
  bool isLooping; // should I make this atomic? thonking

  pros::Task *taskHandle;

  void runLoop();
  static void trampoline(void *instancePtr);
  void checkIsSettled();
  void resetSettled();

  okapi::QAngle calculateAngleError(okapi::QAngle pV, okapi::QAngle setpoint);
  double getHeadingAtPoint(int T);

public:
  AdaptivePurePursuit(
      std::unique_ptr<okapi::IterativePosPIDController> straight,
      std::unique_ptr<okapi::IterativePosPIDController> turn,
      int lookahead, double lookaheadKf);

  void setPath(path::Path *path);

  void setLookahead(int lookahead);

  void loop();
  void ramseteLoop();
  // void pureRamseteLoop();

  path::Point getPointTarget();

  bool isSettled();
  void waitUntilSettled(bool stopWhenSettled = true);
  void runPath(path::Path *path, bool curved = false);
  void runPathAsync(path::Path *path); // @Lachlan are we making the settled utils inside the constructor of appc

  void setKf(double kf);

  void setStraightGains(double kP = -1, double kI = -1, double kD = -1);

  void setTurnGains(double kP = -1, double kI = -1, double kD = -1);

  // instead of this, you probably wanna use start and stop
  void startThread();
  void stopThread();

  void start();
  void stop();
};
} // namespace pathfollowing
