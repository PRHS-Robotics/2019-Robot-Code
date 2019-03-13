#include "subsystems/Arm.h"
#include "Robot.h"

#include <array>
#include <iostream>
#include <iomanip>
#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <fstream>

/*const double BASE_SENSOR_VALUES[LEVEL_COUNT] = { 2.8, 2.933, 3.375, 3.817, 4.310, 2.974, 3.441, 3.947 };
const double WRIST_SENSOR_VALUES[LEVEL_COUNT] = { 3.93, 3.508, 3.443, 3.473, 3.309, 3.872, 3.4787, 3.680 };*/
std::array< double, LEVEL_COUNT > BASE_SENSOR_VALUES =  { 2.461, 2.771, /*2.713*/2.733, 3.071, 3.217, 3.518, 3.697, 3.968, 3.327, 2.461 };
std::array< double, LEVEL_COUNT > WRIST_SENSOR_VALUES = { 2.563, 3.029, /*2.665*/2.665, 3.061, 2.765, 3.146, 2.859, 3.176, 3.156, 2.669 };
const double DIFFERENCE = 0.0;

const double arm_lower_output_limit = /*-0.05*/-0.60;
const double arm_upper_output_limit = /* 0.40*/ 0.80;
const double arm_max_change = 0.015;

const double wrist_lower_output_limit = /*-0.20*/-0.40;
const double wrist_upper_output_limit = /* 0.15*/ 0.50;
const double wrist_max_change = 0.010;

const double armMinValue = 2.4;
const double armMaxValue = 4.2;

const double wristMinValue = 2.53;
const double wristMaxValue = 3.2;

static const double calculateArmFGain(double setpoint) {
    static const double arm_center_voltage = /*TODO*/3.209;
    static const double factor = /*TODO*/0.145;

    double angle = (setpoint - arm_center_voltage) / 0.014666666 * (2.0 * M_PI / 360.0);
    return factor * std::cos(angle) / setpoint;
}

void Arm::reloadValues() {
    std::ifstream file("/home/lvuser/arm_settings.txt");

    assert(file.good());

    int size;
    file >> size;

    std::cout << "Reading values\n";
    for (int i = 0; i < size; ++i) {
        double value;
        file >> value;
        BASE_SENSOR_VALUES[i] = value;
        std::cout << value << "\n";
    }
    std::cout << "Wrist values\n";
    for (int i = 0; i < size; ++i) {
        double value;
        file >> value;
        WRIST_SENSOR_VALUES[i] = value;
        std::cout << value << "\n";
    }
    std::cout << "Done reading\n";
}

Arm::Arm(int baseMotor, int baseSensor, int wristMotor, int wristSensor, int stopSwitchPort) :
    m_baseSensor(baseSensor),
    m_wristSensor(wristSensor),
    m_baseMotor(baseMotor),
    m_wristMotor(wristMotor),
    m_basePID(8.0, 0.0, 0.0, m_baseSensor, m_baseMotor),
    m_wristPID(5.0, 0.0, 0.0, m_wristSensor, m_wristMotor),
    m_level(Level::Home),
    m_stopSwitch(stopSwitchPort),
    Subsystem("ARM")
{
    m_baseSensor.SetAverageBits(8);
    m_wristSensor.SetAverageBits(8);

    m_baseMotor.ConfigVoltageCompSaturation(10.0, 10);
    m_wristMotor.ConfigVoltageCompSaturation(10.0, 10);

    m_baseMotor.EnableVoltageCompensation(true);
    m_wristMotor.EnableVoltageCompensation(true);

    m_baseMotor.SetInverted(false);
    m_wristMotor.SetInverted(true); // SET TO FALSE ON COMP ROBOT

    m_basePID.SetOutputRange(arm_lower_output_limit, arm_upper_output_limit);
    m_wristPID.SetOutputRange(wrist_lower_output_limit, wrist_upper_output_limit);

    m_basePID.SetContinuous(false);
    m_wristPID.SetContinuous(false);

    m_basePID.SetF(calculateArmFGain(BASE_SENSOR_VALUES[0]));

    m_basePID.SetSetpoint(BASE_SENSOR_VALUES[0]);
    m_wristPID.SetSetpoint(WRIST_SENSOR_VALUES[0]);

    reloadValues();

    frc::Scheduler::GetInstance()->RegisterSubsystem(this);
}

void Arm::setArmSetpoint(double setpoint) {
    m_calibration = true;

    m_armSetpoint = std::max(armMinValue, std::min(armMaxValue, setpoint));
}

void Arm::setWristSetpoint(double setpoint) {
    m_calibration = true;

    m_wristSetpoint = std::max(wristMinValue, std::min(wristMaxValue, setpoint));
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

void Arm::setLevel(Level level) {
    m_calibration = false;
    m_wristRetract = false;

    if (level < Level::Home || level >= Level::LEVEL_COUNT) {
        return;
    }

    m_level = level;

    setMode(false);
}

Level Arm::getLevel() {
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

bool Arm::setpointReached() {
    double currentWrist = m_wristPID.GetSetpoint();
    double targetWrist = WRIST_SENSOR_VALUES[static_cast< int >(m_level)];
    if (currentWrist != targetWrist) {
        return false;
    }
    
    if (std::abs(m_wristPID.GetError()) > 0.1) {
        return false;
    }

    double currentArm = m_basePID.GetSetpoint();
    double targetArm = BASE_SENSOR_VALUES[static_cast< int >(m_level)];

    if (currentArm != targetArm) {
        return false;
    }

    if (std::abs(m_basePID.GetError()) > 0.1) {
        return false;
    }

    return true;
}

void Arm::setMode(bool calibration) {
    m_calibration = calibration;
}

double Arm::getArmSetpoint() const {
    return m_armSetpoint;
}

double Arm::getWristSetpoint() const {
    return m_wristSetpoint;
}

void Arm::Periodic() {
    if (!m_stopSwitch.Get()) {
        m_wristPID.SetOutputRange(0.0, wrist_upper_output_limit);
    }
    else {
        m_wristPID.SetOutputRange(wrist_lower_output_limit, wrist_upper_output_limit);
    }

    if (m_calibration) {
        double armLimitedSetpoint = std::max(armMinValue, std::min(armMaxValue, m_armSetpoint));
        m_basePID.SetSetpoint(armLimitedSetpoint);
        m_basePID.SetF(calculateArmFGain(armLimitedSetpoint));

        double wristLimitedSetpoint = std::max(wristMinValue, std::min(wristMaxValue, m_wristSetpoint));
        m_wristPID.SetSetpoint(wristLimitedSetpoint);
    } else {
        double currentWrist = m_wristPID.GetSetpoint();
        double targetWrist = WRIST_SENSOR_VALUES[static_cast< int >(m_level)];
        if (std::abs(targetWrist - currentWrist) < wrist_max_change) {
            currentWrist = targetWrist;
        }
        else {
            currentWrist += wrist_max_change * signum(targetWrist - currentWrist);
        }

        m_wristRetract |= Robot::m_input->getInput().pov2 == 0;

        if (m_wristRetract) {
            currentWrist -= 0.03;
        }

        double wristLimitedSetpoint = std::max(wristMinValue, std::min(wristMaxValue, currentWrist));
        m_wristPID.SetSetpoint(wristLimitedSetpoint);

        // Allows for gradual change of arm and wrist target position
        double currentArm = m_basePID.GetSetpoint();
        double targetArm = BASE_SENSOR_VALUES[static_cast< int >(m_level)];
        if (WRIST_SENSOR_VALUES[static_cast< int >(m_level)] <= WRIST_SENSOR_VALUES[9] && currentWrist != targetWrist) {
            targetArm = currentArm;
        }
        if (std::abs(targetArm - currentArm) < arm_max_change) {
            currentArm = targetArm;
        }
        else {
            currentArm += arm_max_change * signum(targetArm - currentArm);
        }

        double armLimitedSetpoint = std::max(armMinValue, std::min(armMaxValue, currentArm));
        m_basePID.SetSetpoint(armLimitedSetpoint);
        m_basePID.SetF(calculateArmFGain(armLimitedSetpoint));
        frc::SmartDashboard::PutNumber("Arm Feedforward", calculateArmFGain(currentArm));
        frc::SmartDashboard::PutNumber("Arm Base Goal", targetArm);
    }



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