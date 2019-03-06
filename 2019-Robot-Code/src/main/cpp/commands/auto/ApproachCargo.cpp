#include "commands/auto/ApproachCargo.h"
#include "subsystems/DriveTrain.h"
#include "Robot.h"

#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>

ApproachCargo::ApproachCargo(int yawSamples) :
    m_yawSamples(yawSamples),
    m_yawAverager(m_yawSamples),
    frc::Command("ApproachCargo", *static_cast< frc::Subsystem* >(Robot::m_driveTrain.get()))
{

}

void ApproachCargo::Initialize() {
    m_lastDetected = false;
}

void ApproachCargo::Execute() {
	auto ntinstance = nt::NetworkTableInstance::GetDefault();
	auto table = ntinstance.GetTable("ChickenVision");

	nt::NetworkTableEntry detectedEntry = table->GetEntry("cargoDetected");
	nt::NetworkTableEntry yawEntry = table->GetEntry("cargoYaw");

	bool detected = detectedEntry.GetBoolean(false);
	int yaw = detectedEntry.GetDouble(0.0);

    if (detected) {
		std::cout << "Yaw: " << yaw << "\n";

		double yawValue = yaw;

		const int SAMPLES = 2;
		static MovingAverage yawAverager(SAMPLES);

		if (!m_lastDetected) {
			yawAverager.Clear();
		}

		double speed = 0.0;

		double yawTarget = yawAverager.Process(yawValue);

		if (yawTarget > -10 && yawTarget < 10) {
			speed = (1.0 - std::abs(yawTarget / 10.0)) * 0.2;
		}

		frc::SmartDashboard::PutNumber("Average Yaw", yawTarget);

		// TODO: Add gradual ramp-up
		Robot::m_driveTrain->drive(yawTarget / 30.0 + speed, -yawTarget / 30.0 + speed);
	}

	m_lastDetected = detected;
}

bool ApproachCargo::IsFinished() {
    // TODO: Determine when close enough to cargo
    return false;
}

void ApproachCargo::End() {
    Robot::m_driveTrain->drive(0.0, 0.0);
}

void ApproachCargo::Interrupted() {
    End();
}