/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/auto/DriveDistance.h"
#include "Robot.h"

constexpr const double MAX_ACCEL = 0.02;
constexpr const double SLOW_THRESHOLD = 4000.0;
constexpr const double HIGH_SPEED = 0.3;
constexpr const double LOW_SPEED = 0.2;
constexpr const double ERROR_THRESHOLD = 10.0;

DriveDistance::DriveDistance(double dist) :
  targetDistance(dist),
  frc::Command("DriveDistance", *Robot::m_driveTrain)
{

}

double DriveDistance::averageEncoderValue() {
  auto positions = Robot::m_driveTrain->getEncoderPositions();
  double temp = 0.0;
  /*for (double pos : positions.first) {
    temp += pos;
  }
  for (double pos : positions.second) {
    temp += pos;
  }
  temp /= 6.0;*/
  
  temp += positions.first[2];
  temp += positions.second[2];
  temp /= 2.0;

  return temp;
}

// Called just before this Command runs the first time
void DriveDistance::Initialize() {
  m_encoderOffset = averageEncoderValue();
  m_angle = Robot::getHeading();
  m_speed = 0.0;
}

double DriveDistance::distanceError() {
  return (averageEncoderValue() - m_encoderOffset) - targetDistance;
}

double DriveDistance::targetSpeed() {
  if (IsFinished()) {
    return 0.0;
  }
  
  if (distanceError() < SLOW_THRESHOLD) {
    return LOW_SPEED;
  }
  else {
    return HIGH_SPEED;
  }
}

template < typename T >
static int signum(const T& value) {
	return (value > 0) - (value < 0);
}

// Called repeatedly when this Command is scheduled to run
void DriveDistance::Execute() {
  double angle_difference = minDifference(Robot::getHeading(), m_angle);
  double turn = 1.0 * (-1.0/80.0) * angle_difference;

  if (std::abs(m_speed - targetSpeed()) < MAX_ACCEL) {
    m_speed = targetSpeed();
  }
  else {
    m_speed += signum(targetSpeed() - m_speed) * MAX_ACCEL;
  }

  Robot::m_driveTrain->drive(m_speed + turn, m_speed - turn);
}

// Make this return true when this Command no longer needs to run execute()
bool DriveDistance::IsFinished() {
  return std::abs(distanceError()) < ERROR_THRESHOLD;
}

// Called once after isFinished returns true
void DriveDistance::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveDistance::Interrupted() {}
