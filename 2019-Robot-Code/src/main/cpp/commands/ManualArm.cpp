#include "commands/ManualArm.h"
#include "subsystems/Input.h"
#include "Robot.h"
#include <iostream>

ManualArm::ManualArm(Input *input) :
    m_input(input),
    Command("MANUAL_ARM", *Robot::m_arm)
{

}

void ManualArm::Initialize() {
    Robot::m_arm->setLevel(0);
    Robot::m_arm->setEnabled(true);
}

void ManualArm::Execute() {
    /*if (buttonValue(m_input->getInput(), "ARM_UP")) {
        if (!m_debounce) {
            int currentLevel = Robot::m_arm->getLevel();
            Robot::m_arm->setLevel(currentLevel + 1);
        }
        m_debounce = true;
    }
    else if (buttonValue(m_input->getInput(), "ARM_DOWN")) {
        if (!m_debounce) {
            int currentLevel = Robot::m_arm->getLevel();
            Robot::m_arm->setLevel(currentLevel - 1);
        }
        m_debounce = true;
    }
    else {
        m_debounce = false;
    }*/
    if (m_input->getInput().ltrig > 0.9) {
        std::cout << "hatch mode false\n";
        m_hatchMode = false;
    }
    if (m_input->getInput().rtrig > 0.9) {
        std::cout << "hatch mode true\n";
        m_hatchMode = true;
    }

    if (buttonValue(m_input->getInput(), "ARM_LEVEL_1")) {
        Robot::m_arm->setLevel(2 + 3 * m_hatchMode);
    }
    if (buttonValue(m_input->getInput(), "ARM_LEVEL_2")) {
        Robot::m_arm->setLevel(3 + 3 * m_hatchMode);
    }
    if (buttonValue(m_input->getInput(), "ARM_LEVEL_3")) {
        Robot::m_arm->setLevel(4 + 3 * m_hatchMode);
    }

    if (buttonValue(m_input->getInput(), "ARM_RETRACT")) {
        Robot::m_arm->setLevel(0);
    }
}

bool ManualArm::IsFinished() {
    return false;
}

void ManualArm::End() {
    Robot::m_arm->setLevel(0);

    // DO NOT CALL setEnabled(false); THE ARM WILL FALL!
}

void ManualArm::Interrupted() {
    End();
}