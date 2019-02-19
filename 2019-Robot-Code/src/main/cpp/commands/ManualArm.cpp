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

    bool cargoMode = buttonValue(m_input->getInput(), "ARM_CARGO_LEVELS");
    bool hatchMode = buttonValue(m_input->getInput(), "ARM_HATCH_LEVELS");

    if (cargoMode || hatchMode) {
        if (buttonValue(m_input->getInput(), "ARM_LEVEL_1")) {
            Robot::m_arm->setLevel(2 + cargoMode);
        }
        if (buttonValue(m_input->getInput(), "ARM_LEVEL_2")) {
            Robot::m_arm->setLevel(4 + cargoMode);
        }
        if (buttonValue(m_input->getInput(), "ARM_LEVEL_3")) {
            Robot::m_arm->setLevel(6 + cargoMode);
        }
    }


    if (m_input->getInput().pov2 == 90) {
        Robot::m_arm->setLevel(8);
    }

    if (buttonValue(m_input->getInput(), "ARM_RETRACT")) {
        Robot::m_arm->setLevel(Robot::m_manipulator->hasCargo() ? 9 : 0);
    }

    if (Robot::m_manipulator->hasCargo()) {
        if (Robot::m_arm->getLevel() == 1) {
            Robot::m_arm->setLevel(9);
        }
    }
    else if (buttonValue(m_input->getInput(), "ARM_CARGO_INTAKE")) {
        Robot::m_arm->setLevel(1);
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