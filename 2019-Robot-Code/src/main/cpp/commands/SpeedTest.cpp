#include "commands/SpeedTest.h"
#include "Robot.h"
#include "subsystems/DriveTrain.h"

SpeedTest::SpeedTest(Input *input) :
    m_input(input),
    Command("SpeedTest", *static_cast< frc::Subsystem* >(Robot::m_driveTrain.get()))
{

}

void SpeedTest::Initialize() {

}

void SpeedTest::Execute() {
    Robot::m_driveTrain->drive(m_input->getInput().t, m_input->getInput().t, buttonValue(m_input->getInput(), "TRIGGER"));
}

bool SpeedTest::IsFinished() {
    return false;
}

void SpeedTest::End() {
    Robot::m_driveTrain->drive(0.0, 0.0);
}

void SpeedTest::Interrupted() {
    End();
}