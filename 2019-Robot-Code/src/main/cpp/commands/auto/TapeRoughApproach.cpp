/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/auto/TapeRoughApproach.h"
#include "commands/auto/TurnToAngle.h"
#include "commands/auto/DriveDistance.h"
#include "Robot.h"

#include <iostream>
#include <sstream>

struct CameraData {
  double yaw;
  double distance;
};

CameraData parseTapeData(std::string cameraOutput) {
  std::istringstream stream(cameraOutput);

  // Ignore prefix "Tape Distance: "
  stream.ignore(15);

  double distance = 0.0;

  stream >> distance;

  // Ignore ", Yaw: "
  stream.ignore(9);

  double yaw = 0.0;

  stream >> yaw;

  return CameraData{ yaw, distance };
}

CameraData parseBallData(std::string cameraOutput) {
  std::istringstream stream(cameraOutput);

  // Ignore prefix "Ball Yaw: "
  stream.ignore(10);

  double yaw = 0.0;

  stream >> yaw;

  return CameraData{ yaw, 0.0 };
}

CameraData parseCameraOutput(const std::string& cameraOutput) {
  assert(cameraOutput.size() > 4);

  if (cameraOutput.substr(0, 4) == "Tape") {
    return parseTapeData(cameraOutput);
  }
  else if (cameraOutput.substr(0, 4) == "Ball") {
    return parseBallData(cameraOutput);
  }
  else {
    std::cout << "Invalid camera data format\n";
    assert(false);
    return {};
  }
}

Pose calcTapePosition(CameraData data) {
  double absAngle = data.yaw + Robot::getHeading();

  double x = data.distance * std::cos(absAngle * (2.0 * M_PI / 360.0));
  double y = data.distance * std::sin(absAngle * (2.0 * M_PI / 360.0));

  // Snap tape angle to nearest multiple of 45 degrees
  double angle = std::round(absAngle / 45.0) * 45.0;

  return Pose{ x, y, angle };
}

Pose calcTargetPosition(Pose tapePosition) {
  // TODO: Determine arm radius
  double radius = 2.0;

  double x = tapePosition.x - radius * std::cos(tapePosition.angle * (2.0 * M_PI / 360.0));
  double y = tapePosition.y - radius * std::sin(tapePosition.angle * (2.0 * M_PI / 360.0));

  return Pose{ x, y, tapePosition.angle };
}

TapeRoughApproach::Path TapeRoughApproach::getPath(Pose target) {
  Path path;
  path.angle1 = std::atan2(target.y, target.x) * 360.0 / (2.0 * M_PI);
  path.distance = 250.0 * std::sqrt(target.x * target.x + target.y * target.y);
  path.angle2 = target.angle;

  path.angle1 = constrainAngle(path.angle1);
  path.angle2 = constrainAngle(path.angle2);
  return path;
}

TapeRoughApproach::TapeRoughApproach() {
  CameraData data{ 10.0, 1.0 };
  Path path = getPath(calcTargetPosition(calcTapePosition(data)));

  AddSequential(new TurnToAngle(path.angle1));
  AddSequential(new DriveDistance(path.distance));
  AddSequential(new TurnToAngle(path.angle2));
}
