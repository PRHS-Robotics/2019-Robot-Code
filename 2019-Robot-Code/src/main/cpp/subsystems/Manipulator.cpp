#include "subsystems/Manipulator.h"
#include <frc/commands/Scheduler.h>

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
    if (!m_stopSwitch.Get()) {
        m_cargoMotor.Set(std::min(m_cargoDir, 0.0));
    }
    else {
        m_cargoMotor.Set(m_cargoDir);
    }
}