#pragma once

#include "subsystems/Input.h"

#include "frc/commands/Command.h"

class ManualArm : public frc::Command {
public:
    ManualArm(Input *input);

    void Execute() override;

    bool IsFinished() override;

    void End() override;

    void Interrupted() override;

private:
    Input *m_input = nullptr;

    bool debounce = false;
};