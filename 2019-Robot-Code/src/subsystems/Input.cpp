/*
 * Input.cpp
 *
 *  Created on: Nov 10, 2018
 *      Author: super-tails
 */

#include "subsystems/Input.h"

Input::Input(int primaryPort, int secondaryPort) :
	primary(primaryPort),
	secondary(secondaryPort)
{

}

InputState Input::getInput() {
	return InputState{ primary.GetX(), primary.GetY(), primary.GetZ() };
}
