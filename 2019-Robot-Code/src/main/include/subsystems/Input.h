/*
 * Input.h
 *
 *  Created on: Nov 10, 2018
 *      Author: super-tails
 */

#ifndef SRC_SUBSYSTEMS_INPUT_H_
#define SRC_SUBSYSTEMS_INPUT_H_

#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/buttons/Button.h>
#include <unordered_map>
#include <string>
#include <bitset>
#include <tuple>
#include <vector>

constexpr const std::size_t MAX_PRIMARY_BUTTONS = 11;
constexpr const std::size_t MAX_SECONDARY_BUTTONS = 10;
constexpr const std::size_t MAX_BUTTONS = MAX_PRIMARY_BUTTONS + MAX_SECONDARY_BUTTONS;

// Stores the current state of the joystick & xbox controller axes, buttons, etc.
struct InputState {
	double x, y, r, t, lx, ly, ltrig, rtrig, rx, ry;
	int pov1, pov2;
	std::bitset< MAX_BUTTONS > buttons;
};

std::size_t buttonIndex(const std::string& buttonId);

// Returns true if button mapped to code input 'buttonId' (e.g. 'SHIFT_FAST') is pressed
// Returns false if button map is invalid
bool buttonValue(InputState input, const std::string& buttonId);

double applyDeadzone(double value, double deadzoneRange);

class Input {
public:

	Input(int primaryPort, int secondaryPort);

	InputState getRawInput();

	InputState getInput();

	frc::Button *getButton(const std::string& name);

private:

	std::array< std::unique_ptr< frc::Button >, MAX_BUTTONS > buttons;

	frc::Joystick primary;
	frc::XboxController secondary;

	constexpr const static double deadzone = 0.15;
};



#endif /* SRC_SUBSYSTEMS_INPUT_H_ */
