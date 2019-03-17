/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/auto/TapeRoughApproach.h"
#include "commands/auto/TurnToAngle.h"
#include "commands/auto/DriveDistance.h"
#include "commands/auto/SnapAngle.h"
#include "Robot.h"

#include <iostream>
#include <sstream>

CameraData parseTapeData(const std::string& cameraOutput) {
  std::istringstream stream(cameraOutput);

  // Ignore prefix "Tape X: "
  stream.ignore(8);

  double x;

  stream >> x;

  // Ignore ", Y: "
  stream.ignore(5);

  double y;

  stream >> y;

  return CameraData{ x, y };
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
  if (cameraOutput == "Tape None" || cameraOutput == "OK") {
    return CameraData{ 0.0, 0.0 };
  }
  //std::cout << cameraOutput << "\n";

  if (cameraOutput.substr(0, 4) == "Tape") {
    return parseTapeData(cameraOutput);
  }
  else if (cameraOutput.substr(0, 4) == "Ball") {
    return parseBallData(cameraOutput);
  }
  else {
    std::cout << "Invalid camera data format " << cameraOutput << "\n";
    std::cout.flush();
    //assert(false);
    return {};
  }
}

Pose calcTapePosition(CameraData data) {
  /*double absAngle = data.yaw + Robot::getHeading();

  double x = data.distance * std::cos(absAngle * (2.0 * M_PI / 360.0));
  double y = data.distance * std::sin(absAngle * (2.0 * M_PI / 360.0));

  // Snap tape angle to nearest multiple of 45 degrees
  double angle = calcAngle(absAngle);

  return Pose{ x, y, angle };*/
  return {};
}

Pose calcTargetPosition(Pose tapePosition) {
  // TODO: Determine arm radius
  double radius = 0.1;

  double x = tapePosition.x - radius * std::cos(tapePosition.angle * (2.0 * M_PI / 360.0));
  double y = tapePosition.y - radius * std::sin(tapePosition.angle * (2.0 * M_PI / 360.0));

  return Pose{ x, y, tapePosition.angle };
}

TapeRoughApproach::Path TapeRoughApproach::getPath(Pose target) {
  Path path;
  path.angle1 = std::atan2(target.y, target.x) * 360.0 / (2.0 * M_PI);
  path.distance = std::sqrt(target.x * target.x + target.y * target.y);
  path.angle2 = target.angle;

  path.angle1 = constrainAngle(path.angle1);
  path.angle2 = constrainAngle(path.angle2);
  return path;
}

TapeRoughApproach::TapeRoughApproach(double distance, double yaw) {
  Path path = { 0.0, 0.0, 0.0 }; //getPath(calcTargetPosition(calcTapePosition(parseCameraOutput(data))));

  //yaw -= 5;

  double angle = yaw + Robot::getHeading();

  double x = distance * std::cos(angle * 2 * M_PI / 360.0);
  double y = distance * std::sin(angle * 2 * M_PI / 360.0);

  x += 8.75 * std::cos((Robot::getHeading() - 90.0) * 2 * M_PI / 360.0);
  y += 8.75 * std::sin((Robot::getHeading() - 90.0) * 2 * M_PI / 360.0);

  path.angle2 = calcAngle(angle);

  std::array< double, LEVEL_COUNT > armLengths;
  armLengths[Level::CargoLevel1] = 32.0;
  armLengths[Level::CargoLevel2] = 34.0;
  armLengths[Level::CargoLevel3] = 18.0;
  armLengths[Level::HatchLevel1] = 17.0;
  armLengths[Level::HatchLevel2] = 26.0;
  armLengths[Level::HatchLevel3] = 17.0;
  armLengths[Level::CargoShip] = 33.0;

  double armLength = std::max(armLengths[Robot::m_arm->getLevel()] + 6.0, 12.0);

  x -= armLength * std::cos(path.angle2 * 2 * M_PI / 360.0);
  y -= armLength * std::sin(path.angle2 * 2 * M_PI / 360.0);

  distance = std::sqrt(x * x + y * y);

  // 9.0, 12.0

  path.angle1 = atan2(y, x) * 360.0 / (2.0 * M_PI);
  path.distance = distance / 39.6;

  std::cout << "path:\n" << path.angle1 << "\n" << path.distance << "\n" << path.angle2 << "\n";

  AddSequential(new TurnToAngle(path.angle1));
  AddSequential(new DriveDistance(path.distance));
  AddSequential(new TurnToAngle(path.angle2));
}
TapeRoughApproach::TapeRoughApproach(double dist, double yaw) {
  
}