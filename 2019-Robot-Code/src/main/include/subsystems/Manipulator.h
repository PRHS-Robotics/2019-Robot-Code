#pragma once

#include <frc/commands/Subsystem.h>
#include <frc/Solenoid.h>
#include <frc/DigitalInput.h>
#include "rev/SparkMax.h"

class Manipulator : public frc::Subsystem {
public:
    Manipulator(int motorPort, int extendSolenoidPort, int retractSolenoidPort, int switchPort);

    void setCargoDir(double dir);

    void setExtended(bool extended);

    bool hasCargo() const;

    void Periodic() override;

private:
    double m_cargoDir; 

    rev::SparkMax m_cargoMotor;
    frc::Solenoid m_extendSolenoid;
    frc::Solenoid m_retractSolenoid;
    frc::DigitalInput m_stopSwitch;
};