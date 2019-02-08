#include "commands/ManualArm.h"
#include "subsystems/Input.h"
#include "Robot.h"

ManualArm::ManualArm(Input *input) :
    m_input(input),
    Command("MANUAL_ARM", *Robot::m_arm)
{
    Robot::m_arm->setLevel(0);
}

void ManualArm::Execute() {
    if (buttonValue(m_input->getInput(), "ARM_UP")) {
        if (debounce) {
            int currentLevel = Robot::m_arm->getLevel();
            Robot::m_arm->setLevel(currentLevel + 1);
        }
        debounce = true;
    }
    else if (buttonValue(m_input->getInput(), "ARM_DOWN")) {
        if (debounce) {
            int currentLevel = Robot::m_arm->getLevel();
            Robot::m_arm->setLevel(currentLevel - 1);
        }
        debounce = true;
    }
    else {
        debounce = false;
    }
    Robot::m_arm->update();
}

bool ManualArm::IsFinished() {
    return false;
}

void ManualArm::End() {
    Robot::m_arm->setLevel(0);
}

void ManualArm::Interrupted() {
    End();
}