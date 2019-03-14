/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/commands/Scheduler.h>
#include <cameraserver/CameraServer.h>

std::unique_ptr< DriveTrain > Robot::m_driveTrain{};
std::unique_ptr< Input > Robot::m_input{};
std::unique_ptr< frc::Compressor > Robot::m_compressor{};

std::unique_ptr< ManualControl > Robot::m_manualControl{};
std::unique_ptr< ApproachCargo > Robot::m_approachCargo{};
std::unique_ptr< SpeedTest > Robot::m_speedTest{};
std::unique_ptr< FollowPath > Robot::m_followPath{};
std::unique_ptr< Elevator > Robot::m_elevator{};
std::unique_ptr< ElevatorDriveTrain > Robot::m_elevatordrivetrain{};
std::unique_ptr< Manipulator > Robot::m_manipulator{};
std::unique_ptr< Arm > Robot::m_arm{};

std::unique_ptr< DriveUntil > Robot::m_driveUntil{};

std::unique_ptr< ManualManip > Robot::m_manualManip{};

std::unique_ptr< ManualArm > Robot::m_manualArm{};

std::unique_ptr< PigeonIMU > Robot::m_gyro{};

std::unique_ptr< CalibrateArm > Robot::m_calibrateArm{};

std::unique_ptr< frc::DigitalOutput > Robot::m_lights{};

std::unique_ptr< frc::SerialPort > Robot::m_cameraSerial{};

std::unique_ptr< frc::AnalogInput > Robot::m_ultrasonic{};
std::unique_ptr< frc::AnalogInput > Robot::m_ultrasonic2{};

std::unique_ptr< LevelDriveUntil > Robot::m_levelDriveUntil{};

std::unique_ptr< TurnToAngle > Robot::m_turnToAngle{};

std::unique_ptr< DriveDistance > Robot::m_driveDistance{};

std::unique_ptr< TapeRoughApproach > Robot::m_tapeRoughApproach{};

std::unique_ptr< SnapAngle > Robot::m_snapAngle{};

std::unique_ptr< HatchIntake > Robot::m_hatchIntake{};

// Converts an angle <0 or >=360 to be within the range [0, 360)
double constrainAngle(double angle) {
  return std::fmod((std::fmod(angle, 360) + 360), 360);
}

// Returns the minimum angle change to get from startAngle to endAngle
// e.g. minDifference(1, 359) == -2, instead of 358
double minDifference(double startAngle, double endAngle) {
  endAngle = constrainAngle(endAngle);
  startAngle = constrainAngle(startAngle);

  double temp = endAngle - startAngle;
  double diff = temp;
  if (temp > 180) {
    diff -= 360;
  }
  if (temp < -180) {
    diff += 360;
  }

  return diff;
}

// Returns the yaw value of the gyro (not constrained between 0 and 360 degrees)
double Robot::getYaw() {
  double ypr[3];
  m_gyro->GetYawPitchRoll(ypr);
  return ypr[0];
}

// Returns the yaw value of the gyro, converted to be between 0 and 360 degrees
double Robot::getHeading() {
  return constrainAngle(getYaw());
}

double Robot::ultrasonicDistance() {
  return std::min(ultrasonicDistance(0), ultrasonicDistance(0));
}

double Robot::ultrasonicDistance(int sensor) {
  if (sensor) {
    return m_ultrasonic2->GetAverageVoltage() / (5.0 / 512.0);
  }
  else {
    return m_ultrasonic->GetAverageVoltage() / (5.0 / 512.0);
  }
}

void Robot::RobotInit() {
  std::cout << "Hello, world\n";

  m_driveTrain = std::make_unique< DriveTrain >(3, 5, 7, 4, 6, 8);
  m_input = std::make_unique< Input >(0, 1, 2);

	m_manualControl = std::make_unique< ManualControl >(Robot::m_input.get());
	m_approachCargo = std::make_unique< ApproachCargo >(10);
	m_speedTest = std::make_unique< SpeedTest >(Robot::m_input.get());

  m_arm = std::make_unique< Arm >(1, 0, 2, 1, 1);

  m_lights = std::make_unique< frc::DigitalOutput >(9);
  m_lights->EnablePWM(0.0);

  m_manualArm = std::make_unique< ManualArm >(Robot::m_input.get());

  m_manipulator = std::make_unique< Manipulator >(3, 0, 1, 0);

  m_manualManip = std::make_unique< ManualManip >(Robot::m_input.get());

  m_calibrateArm = std::make_unique< CalibrateArm >(Robot::m_input.get());

  m_compressor = std::make_unique< frc::Compressor >(0);

  m_gyro = std::make_unique< PigeonIMU >(10);

  m_cameraSerial = std::make_unique< frc::SerialPort >(115200, frc::SerialPort::kUSB1);

  m_ultrasonic = std::make_unique< frc::AnalogInput >(3);
  m_ultrasonic->SetAverageBits(8);

  m_ultrasonic2 = std::make_unique< frc::AnalogInput >(5);
  m_ultrasonic2->SetAverageBits(8);

  m_driveUntil = std::make_unique< DriveUntil >(40.0);

  m_levelDriveUntil = std::make_unique< LevelDriveUntil >();

  m_turnToAngle = std::make_unique< TurnToAngle >(0.0);

  m_driveDistance = std::make_unique< DriveDistance >(1.0);

  m_snapAngle = std::make_unique< SnapAngle >();

  m_hatchIntake = std::make_unique< HatchIntake >();

  m_tapeRoughApproach = std::make_unique< TapeRoughApproach >("Tape None");

  m_testModeChooser.SetDefaultOption("Competition Mode", 0);
  m_testModeChooser.AddOption("Test Mode", 1);

  m_armChooser.SetDefaultOption("DisabledArm", 0);
  m_armChooser.AddOption("EnabledArm", 1);
  m_armChooser.AddOption("CalibrateArm", 2);

  m_manipulatorChooser.SetDefaultOption("DisabledManip", 0);
  m_manipulatorChooser.AddOption("EnabledManip", 1);

  m_driveTrainChooser.SetDefaultOption("DisabledDrive", 0);
  m_driveTrainChooser.AddOption("EnabledDrive", 1);

  m_pneumaticChooser.SetDefaultOption("DisabledCompressor", 0);
  m_pneumaticChooser.AddOption("EnabledCompressor", 1);
  std::cout << "e\n";

  frc::SmartDashboard::PutData("Mode Chooser", &m_testModeChooser);
  frc::SmartDashboard::PutData("Arm Chooser", &m_armChooser);
  frc::SmartDashboard::PutData("Manip Chooser", &m_manipulatorChooser);
  frc::SmartDashboard::PutData("Drive Chooser", &m_driveTrainChooser);
  frc::SmartDashboard::PutData("Pneumatic Chooser", &m_pneumaticChooser);
  
  frc::SmartDashboard::PutData("Drive Until", m_driveUntil.get());
  frc::SmartDashboard::PutData("Level Drive Until", m_levelDriveUntil.get());
  frc::SmartDashboard::PutData("Turn To Angle", m_turnToAngle.get());
  frc::SmartDashboard::PutData("Drive Distance", m_driveDistance.get());
  frc::SmartDashboard::PutData("Tape Rough Approach", m_tapeRoughApproach.get());
  frc::SmartDashboard::PutData("Hatch Intake", m_hatchIntake.get());

  m_input->getButton("ARM_CALIBRATE")->WhenPressed(m_calibrateArm.get());
  m_input->getButton("ALIGN")->WhenPressed(m_snapAngle.get());
  m_input->getButton("DEBUG_BUTTON")->WhenPressed(m_hatchIntake.get());

  m_driveTrain->SetDefaultCommand(m_manualControl.get());

  frc::CameraServer::GetInstance()->StartAutomaticCapture(1).SetVideoMode(cs::VideoMode::PixelFormat::kYUYV, 320, 240, 30);
}

void Robot::DisabledInit() {
  m_calibrateArm->Start();
  
  m_lights->UpdateDutyCycle(0.0);
}

void Robot::DisabledPeriodic() {
  frc::Scheduler::GetInstance()->Run();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  auto armSensors = m_arm->getSensorValues();

  frc::SmartDashboard::PutNumber("Arm Base Analog", armSensors.first);
  frc::SmartDashboard::PutNumber("Arm Wrist Analog", armSensors.second);


  frc::SmartDashboard::PutNumber("Ultrasonic", ultrasonicDistance());
  frc::SmartDashboard::PutNumber("Ultrasonic1", ultrasonicDistance(0));
  frc::SmartDashboard::PutNumber("Ultrasonic2", ultrasonicDistance(1));

  //frc::SmartDashboard::PutNumber("Gyro Heading", getYaw());

  auto positions = m_driveTrain->getEncoderPositions();

  for (int i : positions.first) {
    std::cout << i << ", ";
  }
  for (int i : positions.second) {
    std::cout << i << ", ";
  }
  std::cout << "\n";

  frc::SmartDashboard::PutNumber("Encoder Position", m_driveTrain->getEncoderPosition(false, 0));
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */

void Robot::matchInit() {
  frc::Scheduler::GetInstance()->RemoveAll();

  m_arm->setEnabled(false);

  m_arm->reloadValues();

  m_lights->UpdateDutyCycle(1.0);

  m_gyro->SetYaw(90.0, 10.0);

  std::cout << "Starting teleop\n";
  if (m_testModeChooser.GetSelected()) {
    std::cout << "Running in test mode\n";
    switch (m_armChooser.GetSelected()) {
    case 1:
      std::cout << "Starting arm control system\n";
      m_arm->setEnabled(true);
      m_manualArm->Start();
      break;
    case 2:
      std::cout << "Starting arm calibration system\n";
      m_calibrateArm->Start();
      break;
    }
    if (m_manipulatorChooser.GetSelected()) {
      std::cout << "Starting manipulator control system\n";
      m_manualManip->Start();
    }
    if (m_driveTrainChooser.GetSelected()) {
      std::cout << "Starting drive train control system\n";
      m_manualControl->Start();
    }
    if (m_pneumaticChooser.GetSelected()) {
      std::cout << "Starting pneumatic system\n";
      m_compressor->Start();
    }
    else {
      m_compressor->Stop();
    }
    std::cout << "Finished starting commands\n";
  }
  else {
    std::cout << "Running in competition mode\n";

    m_arm->setEnabled(true);
    m_manualControl->Start();
    m_manualArm->Start();
    m_manualManip->Start();
    m_compressor->Start();
  }

  if (m_cameraSerial) {
    // setcam autoexp 1\nsetcam absexp 10\n
    std::cout << "Writing\n";
    const char buf[] = "setcam autowb 0\nsetcam autogain 0\nsetcam gain 16\nsetcam redbal 110\nsetcam bluebal 170\nsetcam autoexp 1\nsetcam absexp 15\nsetpar serout All\n";
    m_cameraSerial->Write(buf, sizeof(buf));
  }
}

void Robot::matchPeriodic() {
  //*m_snapAngle = SnapAngle{};

  if (m_cameraSerial) {
    int bytes = m_cameraSerial->GetBytesReceived();
    frc::SmartDashboard::PutNumber("Serial Bytes", bytes);
    if (bytes != 0) {
      char buffer[100] = { 0 };
      int readCount = m_cameraSerial->Read(buffer, sizeof(buffer) - 1);
      //std::cout << "bytes: " << bytes << "\n";
      //std::cout << "Read " << readCount << " bytes, buffer contents:\n";
      //std::cout << buffer << "\n";

      char *result = std::remove_if(buffer, buffer + sizeof(buffer), [](char c) {
        return c == '\r' || c == '\n';
      });

      if (result != buffer + sizeof(buffer)) {
        *result = 0;
      }

      if (buffer != std::string("Tape None") && buffer != std::string("OK")) {

        CameraData data = parseCameraOutput(buffer);

        frc::SmartDashboard::PutNumber("Tape X", data.x);
        frc::SmartDashboard::PutNumber("Tape Y", data.y);

        double distance = 0.0469096 * data.y * data.y - 18.0531 * data.y + 1762.49;
        double yaw = (data.x - 159.5) / 320.0 * 65.0;

        *m_tapeRoughApproach = TapeRoughApproach(distance, yaw);

        frc::SmartDashboard::PutNumber("Tape Distance", distance);
        frc::SmartDashboard::PutNumber("Tape Yaw", yaw);
      }
    }
  }

  frc::Scheduler::GetInstance()->Run();
}

void Robot::AutonomousInit() {
  matchInit();
}

void Robot::AutonomousPeriodic() {
  matchPeriodic();
}

void Robot::TeleopInit() {
  matchInit();
}

void Robot::TeleopPeriodic() {
  matchPeriodic();
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
