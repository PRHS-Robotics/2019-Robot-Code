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

static const std::unordered_map< std::string, std::pair< std::string, int > > defaultButtonMap = {
		{ "SHIFT_FAST", { "High Speed", 3 }, },
		{ "SHIFT_SLOW", { "Low Speed", 5 }, },
		{ "TRIGGER", 	{ "Trigger", 1 } },
		{ "DEBUG_BUTTON", { "DO NOT TOUCH", 2 } },
		{ "SEARCH_AND_DESTROY", { "Search and Destroy", 7 } },
		{ "DEBUG_BUTTON_2", { "DO NOT TOUCH 2", 8 } },
		{ "MANUAL_OVERRIDE", { "Manual Override", 9 } }
};

constexpr const std::size_t MAX_BUTTONS = 11;

struct InputState {
	double x, y, r, t;
	std::bitset< MAX_BUTTONS + 1 > buttons; // FRC button numbering starts at 1
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

	std::vector< std::unique_ptr< frc::Button > > buttons;

	frc::Joystick primary;
	frc::XboxController secondary;

	constexpr const static double deadzone = 0.15;
};



#endif /* SRC_SUBSYSTEMS_INPUT_H_ */
