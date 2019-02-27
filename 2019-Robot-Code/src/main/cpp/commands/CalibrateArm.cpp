/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/CalibrateArm.h"
#include "Robot.h"
#include <fstream>

CalibrateArm::CalibrateArm(Input *input) :
  m_input(input),
  frc::Command("CalibrateArm", *Robot::m_arm) {

}

// Called just before this Command runs the first time
void CalibrateArm::Initialize() {
  Robot::m_arm->setArmSetpoint(0.0);
}

// Called repeatedly when this Command is scheduled to run
void CalibrateArm::Execute() {
  bool goUp   = (m_input->getInput().pov2 == 0);
  bool goDown = (m_input->getInput().pov2 == 180);

  double change = 0.0;
  
  if (!m_debounce) {
    change = (goUp - goDown) * 0.02;
    m_debounce = true;
  }

  if (!(goUp || goDown)) {
    m_debounce = false;
  }
  
  Robot::m_arm->setArmSetpoint(Robot::m_arm->getArmSetpoint() + change);
}

// Make this return true when this Command no longer needs to run execute()
bool CalibrateArm::IsFinished() { return false; }

// Called once after isFinished returns true
void CalibrateArm::End() {
  Robot::m_arm->setArmSetpoint(0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CalibrateArm::Interrupted() {
  End();
}

void CalibrateArm::saveValues() {
  std::ofstream file("/home/lvuser/arm_settings.txt");

  assert(file.good());

  file << m_baseSensorValues.size() << '\n';

  for (auto value : m_baseSensorValues) {
    file << value << '\n';
  }
  for (auto value : m_wristSensorValues) {
    file << value << '\n';
  }
}

void CalibrateArm::readValues() {
  std::ifstream file("/home/lvuser/arm_settings.txt");

  assert(file.good());

  int size;
  file >> size;
  assert(size == m_baseSensorValues.size());

  for (auto& value : m_baseSensorValues) {
    file >> value;
  }
  for (auto& value : m_wristSensorValues) {
    file >> value;
  }
}