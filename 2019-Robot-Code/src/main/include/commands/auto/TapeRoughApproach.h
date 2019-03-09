/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/CommandGroup.h>


struct Pose {
  double x;
  double y;
  double angle;
};

class TapeRoughApproach : public frc::CommandGroup {
public:
  TapeRoughApproach(const std::string& data);

private:
  struct Path {
    double angle1;
    double distance;
    double angle2;
  };

  Path getPath(Pose target);
};