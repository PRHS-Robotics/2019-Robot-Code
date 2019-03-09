#pragma once
#include <frc/AnalogInput.h>
#include <frc/PWMVictorSPX.h>
#include <frc/commands/Subsystem.h>
#include <ctre/Phoenix.h>
#include <frc/PIDController.h>
#include <frc/DigitalInput.h>

#include "commands/ManualArm.h"


// makes the arm do the arm things
class Arm : public frc::Subsystem {
public:
    Arm(int baseMotor, int baseSensorPort, int wristMotor, int wristSensor, int stopSwitchPort);

    void setMode(bool calibration);

    void setArmSetpoint(double setpoint);
    void setWristSetpoint(double setpoint);

    void setLevel(Level level);

    void reloadValues();

    double getArmSetpoint() const;
    double getWristSetpoint() const;

    Level getLevel();

    std::pair< double, double > getSensorValues();

    void Periodic() override;

    void InitDefaultCommand() override;

    void setEnabled(bool enabled);

private:
    Level m_level = Level::Home;

    bool m_calibration = false;

    bool m_wristRetract = false;

    double m_armSetpoint = 0.0, m_wristSetpoint = 0.0;
    
    frc::AnalogInput   m_baseSensor;
    WPI_VictorSPX m_baseMotor;
    frc::PIDController m_basePID;

    frc::AnalogInput   m_wristSensor;
    WPI_VictorSPX m_wristMotor;
    frc::PIDController m_wristPID;

    frc::DigitalInput m_stopSwitch;
};