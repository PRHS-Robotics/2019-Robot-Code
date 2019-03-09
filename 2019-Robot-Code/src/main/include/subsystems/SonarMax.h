#pragma once

#include <frc/AnalogInput.h>

class SonarMax {
public:
	SonarMax(int analogPort);

	double getDistance();

private:
	frc::AnalogInput m_analogInput;
};