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

struct InputState {
	double x, y, r;
};

double applyDeadzone(double value, double deadzoneRange);

class Input {
public:

	Input(int primaryPort, int secondaryPort);

	InputState getRawInput();

	InputState getInput();


private:
	frc::Joystick primary;
	frc::XboxController secondary;

	constexpr const static double deadzone = 0.1;
};



#endif /* SRC_SUBSYSTEMS_INPUT_H_ */
