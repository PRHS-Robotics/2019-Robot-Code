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

class Input {
public:

	Input(int primaryPort, int secondaryPort);

	InputState getInput();

private:
	frc::Joystick primary;
	frc::XboxController secondary;
};



#endif /* SRC_SUBSYSTEMS_INPUT_H_ */
