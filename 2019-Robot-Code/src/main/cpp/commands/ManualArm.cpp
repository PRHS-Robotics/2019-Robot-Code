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
    Robot::m_arm->setLevel(Level::Home);
}

Level getLevel(Input *input) {
    bool cargoMode = buttonValue(input->getInput(), "ARM_CARGO_LEVELS");
    bool hatchMode = buttonValue(input->getInput(), "ARM_HATCH_LEVELS");

    if (buttonValue(input->getInput(), "ARM_HATCH_FLOOR")) {
        return HatchFloorIntake;
    }

    if (cargoMode || hatchMode) {
        if (buttonValue(input->getInput(), "ARM_LEVEL_1")) {
            return (cargoMode ? Level::CargoLevel1 : Level::HatchLevel1);
        }
        if (buttonValue(input->getInput(), "ARM_LEVEL_2")) {
            return (cargoMode ? Level::CargoLevel2 : Level::HatchLevel2);
        }
        if (buttonValue(input->getInput(), "ARM_LEVEL_3")) {
            return (cargoMode ? Level::CargoLevel3 : Level::HatchLevel3);
        }
    }

    if (buttonValue(input->getInput(), "ARM_CARGOSHIP")) {
        return Level::CargoShip;
    }

    if (buttonValue(input->getInput(), "ARM_RETRACT")) {
        return Robot::m_manipulator->hasCargo() ? Level::CargoHome : Level::Home;
    }

    if (Robot::m_manipulator->hasCargo()) {
        if (Robot::m_arm->getLevel() == Level::CargoFloorIntake) {
            return Level::CargoShip;
        }
    }
    else if (buttonValue(input->getInput(), "ARM_CARGO_INTAKE")) {
        return Level::CargoFloorIntake;
    }

    return LEVEL_COUNT;
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

    Level level = getLevel(m_input);
    if (level != LEVEL_COUNT) {
        Robot::m_arm->setLevel(level);
    }


}

bool ManualArm::IsFinished() {
    return false;
}

void ManualArm::End() {
    Robot::m_arm->setLevel(Level::Home);

    // DO NOT CALL setEnabled(false); THE ARM WILL FALL!
}

void ManualArm::Interrupted() {
    End();
}