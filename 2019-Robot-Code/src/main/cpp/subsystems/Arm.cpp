#include "subsystems/Arm.h"
#include "Robot.h"

#include <iostream>
#include <iomanip>

const int LEVEL_COUNT = 9;
//const double BASE_SENSOR_VALUES[LEVEL_COUNT] = { 2.9211, 3.26782, 3.372802, 3.6211225, 3.732909, 3.85864, 4.22363, 4.409129 };
const double BASE_SENSOR_VALUES[LEVEL_COUNT] = { 2.976, 2.933, 3.29, 3.42, 3.817, 4.264, 2.974, 3.441, 3.947 };
const double WRIST_SENSOR_VALUES[LEVEL_COUNT] = { /*FIXTHISONE*/3.14159265358979323846, 3.508, 3.906, 3.432, 3.344, 3.308, 3.872, 3.4787, 3.680 };
const double DIFFERENCE = 0.0;

Arm::Arm(int baseMotor, int baseSensor, int wristMotor, int wristSensor) :
    m_baseSensor(baseSensor),
    m_wristSensor(wristSensor),
    m_baseMotor(baseMotor),
    m_wristMotor(wristMotor),
    m_basePID(0.7, 0.0001, 0.0, m_baseSensor, m_baseMotor),
    m_wristPID(0.1, 0.0, 0.0, m_baseSensor, m_baseMotor),
    m_level(0),
    Subsystem("ARM")
{
    m_baseSensor.SetAverageBits(8);
    m_wristSensor.SetAverageBits(8);

    m_basePID.SetOutputRange(-0.2, 0.2);
    m_wristPID.SetOutputRange(-0.01, 0.01);

    m_basePID.SetContinuous(false);
    m_wristPID.SetContinuous(false);

    m_basePID.Enable();
    m_wristPID.Enable();

    update();
}

void Arm::setLevel(int level) {
    if (level < 0 || level >= LEVEL_COUNT) {
        return;
    }

    m_level = level;

    update();
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

void Arm::update() {
    static double current = BASE_SENSOR_VALUES[m_level] /*- 0.6921*/ + 0.3;
    double target = BASE_SENSOR_VALUES[m_level] /*- 0.6921*/ + 0.3;
    current += 0.05 * (target - current);
    m_basePID.SetSetpoint(current);
    //m_wristPID.SetSetpoint(DIFFERENCE - BASE_SENSOR_VALUES[m_level]);
    m_wristPID.SetSetpoint(WRIST_SENSOR_VALUES[m_level]);

    std::cout << "Base PID: " << std::setw(10)
              << m_basePID.Get() << "\n";

    std::cout << "Wrist PID: " << std::setw(10)
              << m_wristPID.Get() << "\n";
}

void Arm::InitDefaultCommand() {
    SetDefaultCommand(Robot::m_manualArm.get());
}