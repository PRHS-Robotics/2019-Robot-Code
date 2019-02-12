/*
 * Input.cpp
 *
 *  Created on: Nov 10, 2018
 *      Author: super-tails
 */

#include "subsystems/Input.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include <frc/buttons/JoystickButton.h>

// Checks SmartDashboard entry, returns true if button map is valid and usable
static bool isValidMap(std::string buttonId) {
	// SmartDashboard does not support integral-only values
	double temp = frc::SmartDashboard::GetNumber(defaultButtonMap.at(buttonId).first, 0.0);

	if (std::floor(temp) != temp || temp < 1.0 || temp >= MAX_BUTTONS + 1) {
		// Fractional, non-positive, and out-of-range values not allowed
		return false;
	}

	return true;
}

std::size_t buttonIndex(const std::string& buttonId) {
	if (isValidMap(buttonId)) {
		double raw = frc::SmartDashboard::GetNumber(defaultButtonMap.at(buttonId).first, 0.0);
		return static_cast< std::size_t >(raw);
	}
	else {
		return 0;
	}
}

bool buttonValue(InputState input, const std::string& buttonId) {
	std::size_t index = buttonIndex(buttonId);
	if (index == 0) {
		return false;
	}
	else {
		return input.buttons[index];
	}
}

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
	// Ensure all button mappings are valid
	for (const auto& button : defaultButtonMap) {
		frc::SmartDashboard::SetPersistent(button.second.first);
		if (!isValidMap(button.first)) {
			frc::SmartDashboard::PutNumber(button.second.first, button.second.second);
		}
	}

	for (int i = 0; i < MAX_BUTTONS; ++i) {
		// Arrays start at 0, button numbering starts at 1
		buttons.push_back(std::make_unique< frc::JoystickButton >(&primary, i + 1));
	}
}

InputState Input::getRawInput() {
	InputState temp = {
		primary.GetX(),
		primary.GetY(),
		primary.GetZ(),
		primary.GetThrottle(),
		0
	};
	for (std::size_t i = 0; i < MAX_BUTTONS; ++i) {
		temp.buttons[i + 1] = primary.GetRawButton(i + 1);
	}
	return temp;
}

InputState Input::getInput() {
	InputState rawState = getRawInput();

	// Increase twist-axis deadzone when the stick is far from the center to prevent accidental turning
	rawState.r = applyDeadzone(rawState.r, deadzone * (std::abs(rawState.x) + std::abs(rawState.y) + 1.0));

	rawState.x = applyDeadzone(rawState.x, deadzone);
	rawState.y = applyDeadzone(rawState.y, deadzone);

	rawState.t = applyDeadzone(rawState.t, deadzone);

	return rawState;
}

frc::Button *Input::getButton(const std::string& name) {
	std::size_t index = buttonIndex(name);
	if (index == 0) {
		return nullptr;
	}
	else {
		// Arrays start at 0, button numbering starts at 1
		return buttons[index - 1].get();
	}
}