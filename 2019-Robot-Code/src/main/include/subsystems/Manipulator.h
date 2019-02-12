#pragma once

#include <frc/commands/Subsystem.h>
#include <frc/Solenoid.h>
#include <frc/DigitalInput.h>
#include "rev/SparkMax.h"

class Manipulator : public frc::Subsystem {
public:
    Manipulator(int motorPort, int extendSolenoidPort, int retractSolenoidPort, int switchPort);

    void setCargoDir(int dir);

    void setExtended(bool extended);

    void update();

private:
    int m_cargoDir; 

    rev::SparkMax m_cargoMotor;
    frc::Solenoid m_extendSolenoid;
    frc::Solenoid m_retractSolenoid;
    frc::DigitalInput m_stopSwitch;
};