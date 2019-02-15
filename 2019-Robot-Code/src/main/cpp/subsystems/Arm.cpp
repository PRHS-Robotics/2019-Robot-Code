#include "subsystems/Arm.h"
#include "Robot.h"

#include <iostream>
#include <iomanip>
#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

const int LEVEL_COUNT = 8;
const double BASE_SENSOR_VALUES[LEVEL_COUNT] = { 2.8, 2.933, 3.375, 3.817, 4.310, 2.974, 3.441, 3.947 };
const double WRIST_SENSOR_VALUES[LEVEL_COUNT] = { 3.93, 3.508, 3.443, 3.473, 3.309, 3.872, 3.4787, 3.680 };
const double DIFFERENCE = 0.0;

Arm::Arm(int baseMotor, int baseSensor, int wristMotor, int wristSensor) :
    m_baseSensor(baseSensor),
    m_wristSensor(wristSensor),
    m_baseMotor(baseMotor),
    m_wristMotor(wristMotor),
    m_basePID(2, 0.01, 0.0, m_baseSensor, m_baseMotor),
    m_wristPID(4, 0.0, 0.0, m_wristSensor, m_wristMotor),
    m_level(0),
    Subsystem("ARM")
{
    m_baseSensor.SetAverageBits(8);
    m_wristSensor.SetAverageBits(8);

    m_basePID.SetOutputRange(-0.1, 0.4);
    m_wristPID.SetOutputRange(/*-0.3, 0.7*/-0.2,0.2);

    m_basePID.SetContinuous(false);
    m_wristPID.SetContinuous(false);

    frc::Scheduler::GetInstance()->RegisterSubsystem(this);
}

void Arm::setEnabled(bool enabled) {
    if (enabled) {
        m_basePID.Enable();
        m_wristPID.Enable();
    }
    else {
        m_basePID.Disable();
        m_wristPID.Disable();
    }
}

void Arm::setLevel(int level) {
    if (level < 0 || level >= LEVEL_COUNT) {
        return;
    }

    m_level = level;
}

int Arm::getLevel() {
    return m_level;
}

std::pair< double, double > Arm::getSensorValues() {
    return {
        m_baseSensor.GetAverageVoltage(),
        m_wristSensor.GetAverageVoltage()
    };
}

void Arm::Periodic() {
    // Allows for gradual change of arm and wrist target position
    double currentArm = m_basePID.GetSetpoint();
    double targetArm = BASE_SENSOR_VALUES[m_level];
    currentArm += 0.05 * (targetArm - currentArm);
    if (std::abs(targetArm - currentArm) < 0.1) {
        currentArm = targetArm;
    }
    m_basePID.SetSetpoint(currentArm);

    double currentWrist = m_wristPID.GetSetpoint();
    double targetWrist = WRIST_SENSOR_VALUES[m_level];
    currentWrist += 0.05 * (targetWrist - currentWrist);
    if (std::abs(targetWrist - targetWrist) < 0.1) {
        currentWrist = targetWrist;
    }
    m_wristPID.SetSetpoint(WRIST_SENSOR_VALUES[m_level]);

    if (m_baseSensor.GetAverageVoltage() < 2.7 || m_baseSensor.GetAverageVoltage() > 4.5) {
        m_basePID.Disable();
        std::cerr << "Base PID Control disabled, sensor out of range!\n";
    }
    else {
        frc::SmartDashboard::PutNumber("Arm Base Error", m_basePID.GetError());
        frc::SmartDashboard::PutNumber("Arm Base Motor Output", m_basePID.Get());
    }

    if (m_wristSensor.GetAverageVoltage() > 4.0 || m_wristSensor.GetAverageVoltage() < 3.5) {
        m_wristPID.Disable();
        std::cerr << "Wrist PID Control disabled, sensor out of range!\n";
    }
    else {
        frc::SmartDashboard::PutNumber("Arm Wrist Error", m_wristPID.GetError());
        frc::SmartDashboard::PutNumber("Arm Wrist Motor Output", m_wristPID.Get());
    }

}

void Arm::InitDefaultCommand() {
    SetDefaultCommand(Robot::m_manualArm.get());
}