#pragma once

#include "subsystems/Input.h"

#include <frc/commands/Command.h>

class SpeedTest : public frc::Command {
public:
    SpeedTest(Input *input);

    void Initialize() override;

    void Execute() override;

    bool IsFinished() override;

    void End() override;

    void Interrupted() override;

private:
    Input *m_input;
};