#include "subsystems/Arm.h"
#include "Robot.h"

#include <iostream>
#include <iomanip>
#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

const int LEVEL_COUNT = 9;
/*const double BASE_SENSOR_VALUES[LEVEL_COUNT] = { 2.8, 2.933, 3.375, 3.817, 4.310, 2.974, 3.441, 3.947 };
const double WRIST_SENSOR_VALUES[LEVEL_COUNT] = { 3.93, 3.508, 3.443, 3.473, 3.309, 3.872, 3.4787, 3.680 };*/
const double BASE_SENSOR_VALUES[LEVEL_COUNT] =  { 2.8, /*3.1*/3.2,   3.152, 3.445, 3.473, 3.899, 3.958, 4.342, 4.499 };
const double WRIST_SENSOR_VALUES[LEVEL_COUNT] = { 3.89,       4.398, 3.979, 4.411, 4.077, 4.507, 4.176, 4.549, 4.431 };
const double DIFFERENCE = 0.0;

const double lower_output_limit = -0.4;
const double upper_output_limit =  0.2;

Arm::Arm(int baseMotor, int baseSensor, int wristMotor, int wristSensor, int stopSwitchPort) :
    m_baseSensor(baseSensor),
    m_wristSensor(wristSensor),
    m_baseMotor(baseMotor),
    m_wristMotor(wristMotor),
    m_basePID(4, 0.01, 0.0, m_baseSensor, m_baseMotor),
    m_wristPID(5, 0.01, 0.0, m_wristSensor, m_wristMotor),
    m_level(0),
    m_stopSwitch(stopSwitchPort),
    Subsystem("ARM")
{
    m_wristMotor.SetInverted(true);

    m_baseSensor.SetAverageBits(8);
    m_wristSensor.SetAverageBits(8);

    m_basePID.SetOutputRange(-0.1, 0.4);
    m_wristPID.SetOutputRange(lower_output_limit, upper_output_limit);

    m_basePID.SetContinuous(false);
    m_wristPID.SetContinuous(false);

    m_basePID.SetSetpoint(BASE_SENSOR_VALUES[0]);
    m_wristPID.SetSetpoint(WRIST_SENSOR_VALUES[0]);

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
    if (!m_stopSwitch.Get()) {
        m_wristPID.SetOutputRange(0.0, upper_output_limit);
    }
    else {
        m_wristPID.SetOutputRange(lower_output_limit, upper_output_limit);
    }


    // Allows for gradual change of arm and wrist target position
    double currentArm = m_basePID.GetSetpoint();
    double targetArm = BASE_SENSOR_VALUES[m_level] + 0.05 * (m_level > 2);
    currentArm += 0.01 * (targetArm - currentArm);
    if (std::abs(targetArm - currentArm) < 0.1) {
        currentArm = targetArm;
    }
    m_basePID.SetSetpoint(currentArm);

    double currentWrist = m_wristPID.GetSetpoint();
    double targetWrist = WRIST_SENSOR_VALUES[m_level] - 0.0 * (m_level > 2);
    currentWrist += 0.05 * (targetWrist - currentWrist);
    if (std::abs(targetWrist - currentWrist) < 0.1) {
        currentWrist = targetWrist;
    }
    m_wristPID.SetSetpoint(currentWrist);

    /*if (m_baseSensor.GetAverageVoltage() < 2.7 || m_baseSensor.GetAverageVoltage() > 4.5) {
        m_basePID.Disable();
        std::cerr << "Base PID Control disabled, sensor out of range!\n";
    }
    else {*/
        frc::SmartDashboard::PutNumber("Arm Base Error", m_basePID.GetError());
        frc::SmartDashboard::PutNumber("Arm Base Motor Output", m_basePID.Get());
    //}

    /*if (m_wristSensor.GetAverageVoltage() > 4.0 || m_wristSensor.GetAverageVoltage() < 3.5) {
        m_wristPID.Disable();
        std::cerr << "Wrist PID Control disabled, sensor out of range!\n";
    }
    else {*/
        frc::SmartDashboard::PutNumber("Arm Wrist Error", m_wristPID.GetError());
        frc::SmartDashboard::PutNumber("Arm Wrist Motor Output", m_wristPID.Get());
    //}

}

void Arm::InitDefaultCommand() {
    SetDefaultCommand(Robot::m_manualArm.get());
}