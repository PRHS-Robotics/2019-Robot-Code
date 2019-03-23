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

    bool stopSwitchPressed() const;

    bool setpointReached();

    double getTargetWrist();

    double getTargetArm();

private:
    Level m_prevLevel = Level::Home;

    Level m_level = Level::Home;

    Level m_finalLevel = Level::Home;

    enum State { ArmLevelMove, WristMove, ArmHomeMove };

    State m_state = State::ArmHomeMove;

    bool m_calibration = false;

    bool m_wristRetract = false;

    double m_baseOffset = 0.0;
    double m_wristOffset = 0.0;

    bool m_debounce;

    double updateWrist();
    double updateArm();

    bool doHalfSpeed();
    
    frc::AnalogInput   m_baseSensor;
    WPI_VictorSPX m_baseMotor;
    frc::PIDController m_basePID;

    frc::AnalogInput   m_wristSensor;
    WPI_VictorSPX m_wristMotor;
    frc::PIDController m_wristPID;

    frc::DigitalInput m_stopSwitch;
};