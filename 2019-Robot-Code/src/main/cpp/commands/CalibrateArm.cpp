/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/CalibrateArm.h"
#include "commands/ManualArm.h"
#include "Robot.h"
#include <fstream>
#include <iostream>

CalibrateArm::CalibrateArm(Input *input) :
  m_input(input),
  frc::Command("CalibrateArm", *Robot::m_arm) {
}

// Called just before this Command runs the first time
void CalibrateArm::Initialize() {
  readValues();
  Robot::m_arm->setArmSetpoint(Robot::m_arm->getSensorValues().first);
  Robot::m_arm->setWristSetpoint(Robot::m_arm->getSensorValues().second);
  Robot::m_arm->setEnabled(true);
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

  change += ((m_input->getInput().pov2 == 90) - (m_input->getInput().pov2 == 270)) * 0.01;

  if (buttonValue(m_input->getInput(), "ARM_CALIBRATE")) {
    int level = getLevel(Robot::m_input.get());
    if (level != -1) {
      std::cout << "saving to level " << level << ", setpoints: ";
      std::cout << Robot::m_arm->getArmSetpoint() << ", ";
      std::cout << Robot::m_arm->getWristSetpoint() << "\n";
      m_baseSensorValues[level]  = Robot::m_arm->getArmSetpoint();
      m_wristSensorValues[level] = Robot::m_arm->getWristSetpoint();

      m_baseSensorValues[Level::CargoHome] = m_baseSensorValues[Level::Home];
      m_wristSensorValues[Level::CargoHome] = m_wristSensorValues[Level::Home] + 0.06;
      std::cout << "saving\n";
      saveValues();
    }
    else {
      std::cout << "===============\nSELECT A LEVEL!\n===============\n";
    }
  }
  
  if (m_input->getInput().rtrig > 0.9) {
    Robot::m_arm->setWristSetpoint(Robot::m_arm->getWristSetpoint() + change);
  }
  else {
    Robot::m_arm->setArmSetpoint(Robot::m_arm->getArmSetpoint() + change);
  }

  static int i = 0;
  if (++i >= 100) {
    i = 0;
    std::cout << "setpoints are "
      << Robot::m_arm->getArmSetpoint()
      << ", "
      << Robot::m_arm->getWristSetpoint()
      << "\n";
  }
}

// Make this return true when this Command no longer needs to run execute()
bool CalibrateArm::IsFinished() { return false; }

// Called once after isFinished returns true
void CalibrateArm::End() {
  Robot::m_arm->setArmSetpoint(2.42);
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

  file.close();

  std::ifstream file2("home/lvuser/arm_settings.txt");

  int size;
  file2 >> size;
  for (int i = 0; i < size; ++i) {
    double temp;
    file2 >> temp;
    std::cout << temp << "\n";
  }
}

void CalibrateArm::readValues() {
  std::ifstream file("/home/lvuser/arm_settings.txt");

  assert(file.good());

  int size;
  file >> size;

  std::cout << "Reading values\n";
  for (int i = 0; i < size; ++i) {
    double value;
    file >> value;
    m_baseSensorValues[i] = value;
    std::cout << value << "\n";
  }
  std::cout << "Wrist values\n";
  for (int i = 0; i < size; ++i) {
    double value;
    file >> value;
    m_wristSensorValues[i] = value;
    std::cout << value << "\n";
  }
  std::cout << "Done reading\n";
}