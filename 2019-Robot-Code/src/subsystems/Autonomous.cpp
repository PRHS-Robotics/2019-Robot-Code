/*
 * Autonomous.cpp
 *
 *  Created on: Jan 3, 2019
 *      Author: super-tails
 */

#include "Autonomous.h"
#include "DriveTrain.h"
#include "Timer.h"

void Autonomous::execute(DriveTrain& driveTrain, Action action) {
	m_timer.Reset();
	m_timer.Start();
	while (!m_timer.HasPeriodPassed(action.duration)) {
		driveTrain.drive(action.leftSpeed, action.rightSpeed);
	}

	m_timer.Stop();
}

void Autonomous::run(DriveTrain& driveTrain) {
	for (Action& action : m_actions) {
		execute(driveTrain, action);
	}

	driveTrain.drive(0.0, 0.0);
}
