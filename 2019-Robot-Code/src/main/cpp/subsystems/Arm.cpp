#include "subsystems/Arm.h"
#include "Robot.h"

#include <iostream>
#include <iomanip>
#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

const int LEVEL_COUNT = 10;
/*const double BASE_SENSOR_VALUES[LEVEL_COUNT] = { 2.8, 2.933, 3.375, 3.817, 4.310, 2.974, 3.441, 3.947 };
const double WRIST_SENSOR_VALUES[LEVEL_COUNT] = { 3.93, 3.508, 3.443, 3.473, 3.309, 3.872, 3.4787, 3.680 };*/
const double BASE_SENSOR_VALUES[LEVEL_COUNT] =  { 2.461, 2.771, /*3.152*/2.713, 3.071, 3.187, 3.518, 3.667, 3.968, 3.327, 2.461 };
const double WRIST_SENSOR_VALUES[LEVEL_COUNT] = { 2.563, 3.029, /*3.979*/2.665, 3.061, 2.765, 3.146, 2.859, 3.176, 3.156, 2.669 };
const double DIFFERENCE = 0.0;

const double arm_lower_output_limit = -0.05;
const double arm_upper_output_limit =  0.40;
const double arm_max_change = 0.005;

const double wrist_lower_output_limit = /*-0.20*/-0.20;
const double wrist_upper_output_limit = /* 0.15*/ 0.15;
const double wrist_max_change = 0.005;

static const double calculateArmFGain(double setpoint) {
    static const double arm_center_voltage = /*TODO*/3.209;
    static const double factor = /*TODO*/0.145;

    double angle = (setpoint - arm_center_voltage) / 0.014666666 * (2.0 * M_PI / 360.0);
    return factor * std::cos(angle);
}

Arm::Arm(int baseMotor, int baseSensor, int wristMotor, int wristSensor, int stopSwitchPort) :
    m_baseSensor(baseSensor),
    m_wristSensor(wristSensor),
    m_baseMotor(baseMotor),
    m_wristMotor(wristMotor),
    m_basePID(8.0, 0.0, 0.0, m_baseSensor, m_baseMotor),
    m_wristPID(5.0, 0.0, 0.0, m_wristSensor, m_wristMotor),
    m_level(0),
    m_stopSwitch(stopSwitchPort),
    Subsystem("ARM")
{
    m_baseSensor.SetAverageBits(8);
    m_wristSensor.SetAverageBits(8);

    m_basePID.SetOutputRange(arm_lower_output_limit, arm_upper_output_limit);
    m_wristPID.SetOutputRange(wrist_lower_output_limit, wrist_upper_output_limit);

    m_basePID.SetContinuous(false);
    m_wristPID.SetContinuous(false);

    m_basePID.SetF(calculateArmFGain(BASE_SENSOR_VALUES[0]));

    m_basePID.SetSetpoint(BASE_SENSOR_VALUES[0]);
    m_wristPID.SetSetpoint(WRIST_SENSOR_VALUES[0]);

    frc::Scheduler::GetInstance()->RegisterSubsystem(this);
}

void Arm::setEnabled(bool enabled) {
    if (enabled) {
        m_basePID.Enable();
        m_wristPID.Enable();
    }
    else {
        m_basePID.Disable();
        m_wristPID.Disable();
    }
}

void Arm::setLevel(int level) {
    if (level < 0 || level >= LEVEL_COUNT) {
        return;
    }

    m_level = level;
}

int Arm::getLevel() {
    return m_level;
}

std::pair< double, double > Arm::getSensorValues() {
    return {
        m_baseSensor.GetAverageVoltage(),
        m_wristSensor.GetAverageVoltage()
    };
}

template < typename T >
static int signum(const T& value) {
	return (value > 0) - (value < 0);
}

void Arm::Periodic() {
    if (!m_stopSwitch.Get()) {
        m_wristPID.SetOutputRange(0.0, wrist_upper_output_limit);
    }
    else {
        m_wristPID.SetOutputRange(wrist_lower_output_limit, wrist_upper_output_limit);
    }

    double currentWrist = m_wristPID.GetSetpoint();
    double targetWrist = WRIST_SENSOR_VALUES[m_level];
    if (std::abs(targetWrist - currentWrist) < wrist_max_change) {
        currentWrist = targetWrist;
    }
    else {
        currentWrist += wrist_max_change * signum(targetWrist - currentWrist);
    }
    m_wristPID.SetSetpoint(currentWrist);

    // Allows for gradual change of arm and wrist target position
    double currentArm = m_basePID.GetSetpoint();
    double targetArm = BASE_SENSOR_VALUES[m_level];
    if (WRIST_SENSOR_VALUES[m_level] <= WRIST_SENSOR_VALUES[9] && currentWrist != targetWrist) {
        targetArm = currentArm;
    }
    if (std::abs(targetArm - currentArm) < arm_max_change) {
        currentArm = targetArm;
    }
    else {
        currentArm += arm_max_change * signum(targetArm - currentArm);
    }
    m_basePID.SetSetpoint(currentArm);
    m_basePID.SetF(calculateArmFGain(currentArm));

    frc::SmartDashboard::PutNumber("Arm Feedforward", calculateArmFGain(currentArm));


    /*if (m_baseSensor.GetAverageVoltage() < 2.7 || m_baseSensor.GetAverageVoltage() > 4.5) {
        m_basePID.Disable();
        std::cerr << "Base PID Control disabled, sensor out of range!\n";
    }
    else {*/
        frc::SmartDashboard::PutNumber("Arm Base Error", m_basePID.GetError());
        frc::SmartDashboard::PutNumber("Arm Base Motor Output", m_basePID.Get());
    //}

    /*if (m_wristSensor.GetAverageVoltage() > 4.0 || m_wristSensor.GetAverageVoltage() < 3.5) {
        m_wristPID.Disable();
        std::cerr << "Wrist PID Control disabled, sensor out of range!\n";
    }
    else {*/
        frc::SmartDashboard::PutNumber("Arm Wrist Error", m_wristPID.GetError());
        frc::SmartDashboard::PutNumber("Arm Wrist Motor Output", m_wristPID.Get());
    //}

}

void Arm::InitDefaultCommand() {
    SetDefaultCommand(Robot::m_manualArm.get());
}