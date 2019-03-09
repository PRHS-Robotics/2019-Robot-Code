#include "subsystems/SonarMax.h"

SonarMax::SonarMax(int analogPort) :
    m_analogInput(analogPort)
{
    
}

double SonarMax::getDistance() {
    return (m_analogInput.GetVoltage() / (5.0 / 512.0)) + 2.0;
}