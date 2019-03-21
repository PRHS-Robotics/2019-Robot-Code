#pragma once

#include "subsystems/Input.h"

#include "frc/commands/Command.h"

enum Level{ Home, CargoHome, HatchFloorIntake, CargoFloorIntake, HatchLevel1, CargoLevel1, HatchLevel2, CargoLevel2, HatchLevel3, CargoLevel3, CargoShip, LEVEL_COUNT };

class ManualArm : public frc::Command {
public:
    ManualArm(Input *input);

    void Initialize() override;

    void Execute() override;

    bool IsFinished() override;

    void End() override;

    void Interrupted() override;

private:
    Input *m_input = nullptr;

    bool m_debounce = false;
};


Level getLevel(Input *input);