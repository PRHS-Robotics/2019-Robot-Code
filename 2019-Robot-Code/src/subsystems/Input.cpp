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

InputState Input::getRawInput() {
	return InputState{ primary.GetX(), primary.GetY(), primary.GetZ() };
}

InputState Input::getInput() {
	InputState rawState = getRawInput();
	double x = applyDeadzone(rawState.x, deadzone);
	double y = applyDeadzone(rawState.y, deadzone);

	// Increase twist-axis deadzone when the stick is far from the center to prevent accidental turning
	double r = applyDeadzone(rawState.r, 2.0 * deadzone * (1 + 0.5 *(std::abs(x) + std::abs(y))));
	return InputState{ x, y, r };
}
