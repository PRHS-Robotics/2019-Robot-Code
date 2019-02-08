#include "subsystems/Arm.h"
#include "Robot.h"

const int   LEVEL_COUNT = 8;
const double BASE_SENSOR_VALUES[LEVEL_COUNT] =  { 2.9211, 3.26782, 3.372802, 3.6211225, 3.732909, 3.85864, 4.22363, 4.409129 };
const double DIFFERENCE = 0.0;

Arm::Arm(int baseMotor, int baseSensor, int wristMotor, int wristSensor) :
    m_baseSensor(baseSensor),
    m_wristSensor(wristSensor),
    m_baseMotor(baseMotor),
    m_wristMotor(wristMotor),
    m_basePID(0.1, 0.0, 0.0, m_baseSensor, m_baseMotor),
    m_wristPID(0.1, 0.0, 0.0, m_baseSensor, m_baseMotor),
    m_level(0),
    Subsystem("ARM")
{
    m_baseSensor.SetAverageBits(8);
    m_wristSensor.SetAverageBits(8);

    m_basePID.SetOutputRange(-0.01, 0.01);

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
    m_basePID.SetSetpoint(BASE_SENSOR_VALUES[m_level]);
    m_wristPID.SetSetpoint(DIFFERENCE - BASE_SENSOR_VALUES[m_level]);
}

void Arm::InitDefaultCommand() {
    SetDefaultCommand(Robot::m_manualArm.get());
}