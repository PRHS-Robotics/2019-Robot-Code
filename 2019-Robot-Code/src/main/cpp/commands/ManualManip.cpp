#include "commands/ManualManip.h"
#include "Robot.h"

ManualManip::ManualManip(Input *input) :
    m_input(input),
    Command("ManualManip", Robot::m_manipulator)
{
    
}

void ManualManip::Execute() {
    bool intakeCargo = buttonValue(input->getInput(), "INTAKE_CARGO");
    bool outtakeCargo = buttonValue(input->getInput(), "OUTTAKE_CARGO");

    if (intakeCargo) {
        Robot::m_manipulator->setCargoDir(-1.0);
    }
    if (outtakeCargo) {
        Robot::m_manipulator->setCargoDir(1.0);
    }

    Robot::m_manipulator->setExtended(buttonValue(input->getInput(), "OUTTAKE_PANEL"));

    Robot::m_manipulator->update();
}