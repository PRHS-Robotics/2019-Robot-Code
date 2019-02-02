#pragma once

#include "subsystems/Input.h"

#include <frc/commands/Command.h>

class ManualControl : public frc::Command {
public:

    ManualControl(Input *input);

    void Execute() override;

    bool IsFinished() override;

    void End() override;

    void Interrupted() override;

private:
    Input *m_input;
};