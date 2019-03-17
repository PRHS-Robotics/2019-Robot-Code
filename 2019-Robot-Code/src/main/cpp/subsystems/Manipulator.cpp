#include "subsystems/Manipulator.h"
#include <frc/commands/Scheduler.h>
#include "Robot.h"

Manipulator::Manipulator(int motorPort, int extendSolenoidPort, int retractSolenoidPort, int switchPort) :
    m_cargoMotor(motorPort),
    m_extendSolenoid(extendSolenoidPort),
    m_retractSolenoid(retractSolenoidPort),
    m_stopSwitch(switchPort),
    Subsystem("Manipulator")
{
    frc::Scheduler::GetInstance()->RegisterSubsystem(this);
}

bool Manipulator::hasCargo() const {
    return !m_stopSwitch.Get();
}

void Manipulator::setCargoDir(double dir) {
    m_cargoDir = dir;
}

void Manipulator::setExtended(bool extended) {
    m_extendSolenoid.Set(extended);
    m_retractSolenoid.Set(!extended);
}

void Manipulator::Periodic() {
    if (Robot::m_arm->getLevel() == Level::CargoHome || Robot::m_arm->getLevel() == Level::Home) {
        m_cargoMotor.Set(0.0);
        setExtended(false);
        return;
    }

    double temp = m_cargoDir;
    if (temp < 0.0 && Robot::m_arm->getLevel() != Level::CargoShip) {
        temp *= 0.7 * 0.75;
    }
    else if (temp > 0.0) {
        temp *= 1.0;
    }


    if (!m_stopSwitch.Get()) {
        m_cargoMotor.Set(std::min(temp, 0.0));
    }
    else {
        m_cargoMotor.Set(temp);
    }
}