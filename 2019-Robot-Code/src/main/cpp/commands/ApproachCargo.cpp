#include "commands/ApproachCargo.h"
#include "subsystems/DriveTrain.h"
#include "subsystems/ArduinoInterface.h"
#include "Robot.h"

#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include <iostream>
#include <algorithm>

ApproachCargo::ApproachCargo() :
    frc::Command("ApproachCargo", *Robot::m_driveTrain.get())
{

}

void ApproachCargo::Initialize() {
	Robot::m_arduino->readData(false);

	auto ntinstance = nt::NetworkTableInstance::GetDefault();
	auto table = ntinstance.GetTable("ChickenVision");

	nt::NetworkTableEntry useTape = table->GetEntry("Tape");
	useTape.SetBoolean(false);
}

void ApproachCargo::Execute() {
	auto result = getTargetYaw();

	double turningSpeed = result.first / 50.0;
	double forwardSpeed = std::max((1.0 - std::abs(result.first / 10.0), 0.0) * 0.2, 0.0);

	Robot::m_driveTrain->drive(forwardSpeed + turningSpeed, forwardSpeed - turningSpeed);
}

bool ApproachCargo::IsFinished() {
    // TODO: Determine when close enough to cargo
    return false;
}

void ApproachCargo::End() {
    Robot::m_driveTrain->drive(0.0, 0.0);
	Robot::m_arduino->readData(false);
}

void ApproachCargo::Interrupted() {
    End();
}

std::pair< double, bool > ApproachCargo::getTargetYaw() {
	auto ntinstance = nt::NetworkTableInstance::GetDefault();
	auto table = ntinstance.GetTable("ChickenVision");

	nt::NetworkTableEntry detected = table->GetEntry("cargoDetected");
	nt::NetworkTableEntry yaw = table->GetEntry("cargoYaw");

	frc::SmartDashboard::PutNumber("Cargo Contours", table->GetEntry("cargoContours").GetDouble(0.0));

	if (!detected.GetBoolean(false)) {
		return { 0.0, false };
	}

	return { yaw.GetDouble(0.0), true };
}