#include "commands/ManualManip.h"
#include "Robot.h"

ManualManip::ManualManip(Input *input) :
    m_input(input),
    Command("ManualManip", *Robot::m_manipulator)
{
    
}

void ManualManip::Execute() {
    /*bool intakeCargo = buttonValue(m_input->getInput(), "INTAKE_CARGO");
    bool outtakeCargo = buttonValue(m_input->getInput(), "OUTTAKE_CARGO");

    if (intakeCargo) {
        Robot::m_manipulator->setCargoDir(-1.0);
    }
    if (outtakeCargo) {
        Robot::m_manipulator->setCargoDir(1.0);
    }*/

    Robot::m_manipulator->setCargoDir(m_input->getInput().ly * 0.5);

    Robot::m_manipulator->setExtended(m_input->getInput().pov2 == 0);

    //Robot::m_manipulator->setExtended(buttonValue(m_input->getInput(), "OUTTAKE_PANEL"));
}

bool ManualManip::IsFinished() {
    return false;
}

void ManualManip::End() {
    Robot::m_manipulator->setCargoDir(0.0);
    Robot::m_manipulator->setExtended(false);
}

void ManualManip::Interrupted() {
    End();
}