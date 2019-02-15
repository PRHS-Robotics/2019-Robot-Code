#pragma once
#include <frc/AnalogInput.h>
#include <frc/PWMVictorSPX.h>
#include <frc/commands/Subsystem.h>
#include <ctre/Phoenix.h>
#include <frc/PIDController.h>

// makes the arm do the arm things
class Arm : public frc::Subsystem {
public:
    Arm(int baseMotor, int baseSensorPort, int wristMotor, int wristSensor);

    void setLevel(int level);

    int getLevel();

    std::pair< double, double > getSensorValues();

    void Periodic() override;

    void InitDefaultCommand() override;

    void setEnabled(bool enabled);

private:
    int m_level;
    // AnalogChannel * potentiometer;
    // Victor* victor;
    // end ATOUGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATION
    // hope this is what you needed
    frc::AnalogInput   m_baseSensor;
    WPI_VictorSPX m_baseMotor;
    frc::PIDController m_basePID;

    frc::AnalogInput   m_wristSensor;
    WPI_VictorSPX m_wristMotor;
    frc::PIDController m_wristPID;
};