/*
 * Input.cpp
 *
 *  Created on: Nov 10, 2018
 *      Author: super-tails
 */

#include "subsystems/Input.h"

double applyDeadzone(double value, double deadzoneRange) {
	double sign = (value > 0.0) - (value < 0.0);
	value = std::abs(value);
	if (value <= deadzoneRange) {
		return 0;
	}
	else {
		return sign * ((value - deadzoneRange) / (1.0 - deadzoneRange));
	}
}

Input::Input(int primaryPort, int secondaryPort) :
	primary(primaryPort),
	secondary(secondaryPort)
{

}

InputState Input::getInput() {
	double x = applyDeadzone(primary.GetX(), deadzone);
	double y = applyDeadzone(primary.GetY(), deadzone);
	double z = applyDeadzone(primary.GetZ(), deadzone);
	if (std::abs(z) <= 0.2 * (std::abs(x) + std::abs(y))) {
		z = 0;
	}
	return InputState{ x, y, z };
}
