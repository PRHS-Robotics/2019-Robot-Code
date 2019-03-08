/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/auto/TapeRoughApproach.h"
#include "commands/auto/TurnToAngle.h"
#include "commands/auto/DriveDistance.h"

#include <iostream>
#include <sstream>

struct Pose {
  double x;
  double y;
  double angle;
};

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
  
}

Pose calcTargetPosition(Pose tapePosition) {

}

TapeRoughApproach::TapeRoughApproach() {
  double x = 1.0, y = 2.0;
  Path path{ 0.0, 0.0, 0.0 }; // TODO

  path.angle1 = std::atan2(y, x) * 360.0 / (2 * M_PI);
  path.distance = 250.0 * std::sqrt(x * x + y * y);
  path.angle2 = 0.0;

  AddSequential(new TurnToAngle(path.angle1));
  AddSequential(new DriveDistance(path.distance));
  AddSequential(new TurnToAngle(path.angle2));
  // Add Commands here:
  // e.g. AddSequential(new Command1());
  //      AddSequential(new Command2());
  // these will run in order.

  // To run multiple commands at the same time,
  // use AddParallel()
  // e.g. AddParallel(new Command1());
  //      AddSequential(new Command2());
  // Command1 and Command2 will run in parallel.

  // A command group will require all of the subsystems that each member
  // would require.
  // e.g. if Command1 requires chassis, and Command2 requires arm,
  // a CommandGroup containing them would require both the chassis and the
  // arm.
}
