#include "subsystems/Manipulator.h"

Manipulator::Manipulator(int motorPort, int extendSolenoidPort, int retractSolenoidPort, int switchPort) :
    m_cargoMotor(motorPort),
    m_extendSolenoid(extendSolenoidPort),
    m_retractSolenoid(retractSolenoidPort),
    m_stopSwitch(switchPort),
    Subsystem("Manipulator")
{
    
}

void Manipulator::setCargoDir(int dir) {
    m_cargoDir = dir;
}

void Manipulator::setExtended(bool extended) {
    m_extendSolenoid.Set(extended);
    m_retractSolenoid.Set(!extended);
}

void Manipulator::update() {
    m_cargoMotor.Set(m_cargoDir * 0.2);
}