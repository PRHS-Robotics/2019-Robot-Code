/*
 * Input.h
 *
 *  Created on: Nov 10, 2018
 *      Author: super-tails
 */

#ifndef SRC_SUBSYSTEMS_INPUT_H_
#define SRC_SUBSYSTEMS_INPUT_H_

#include <Joystick.h>
#include <XboxController.h>
#include <unordered_map>
#include <string>
#include <bitset>
#include <tuple>

static const std::unordered_map< std::string, std::pair< std::string, int > > defaultButtonMap = {
		{ "SHIFT_FAST", { "High Speed", 3 }, },
		{ "SHIFT_SLOW", { "Low Speed", 5 }, },
		{ "TRIGGER", 	{ "Trigger", 1 } }
};

constexpr const std::size_t MAX_BUTTONS = 11;

struct InputState {
	double x, y, r;
	std::bitset< MAX_BUTTONS + 1 > buttons; // FRC button numbering starts at 1
};

// Returns true if button mapped to code input 'buttonId' (e.g. 'SHIFT_FAST') is pressed
// Returns false if button map is invalid
bool buttonValue(InputState input, std::string buttonId);

double applyDeadzone(double value, double deadzoneRange);

class Input {
public:

	Input(int primaryPort, int secondaryPort);

	InputState getRawInput();

	InputState getInput();


private:
	frc::Joystick primary;
	frc::XboxController secondary;

	constexpr const static double deadzone = 0.10;
};



#endif /* SRC_SUBSYSTEMS_INPUT_H_ */
