/*
 * DriveTrain.h
 *
 *  Created on: Nov 10, 2018
 *      Author: super-tails
 */

#ifndef SRC_SUBSYSTEMS_DRIVETRAIN_H_
#define SRC_SUBSYSTEMS_DRIVETRAIN_H_

#include "subsystems/Input.h"

#include <memory>

#include <frc/Solenoid.h>
#include <frc/commands/Subsystem.h>

#include <ctre/Phoenix.h>

class DriveTrain : public frc::Subsystem {
public:

	DriveTrain(int frontLeft, int midLeft, int backLeft, int frontRight, int midRight, int backRight);

	void drive(InputState state);

	void drive(double leftSpeed, double rightSpeed, bool percentOutput = true);

	std::pair< std::array< int, 3 >, std::array< int, 3 > > getEncoderPositions();

	void resetSensors();

	void InitDefaultCommand() override;

private:

	//void calibratePhase(double leftSpeed, double rightSpeed);

	bool leftSidePhase() const;
	void setLeftSidePhase(bool phase);

	bool rightSidePhase() const;
	void setRightSidePhase(bool phase);

	std::array< std::unique_ptr< WPI_TalonSRX >, 3 > m_lMotors;
	std::array< std::unique_ptr< WPI_TalonSRX >, 3 > m_rMotors;
};



#endif /* SRC_SUBSYSTEMS_DRIVETRAIN_H_ */
