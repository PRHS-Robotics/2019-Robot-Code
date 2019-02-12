#pragma once

#include "subsystems/Input.h"
#include "frc/commands/Command.h"

class ManualManip : frc::Command {
public:
    ManualManip(Input *input);

    void Execute() override;

    bool IsFinished() override;

    void End() override;

    void Interrupted() override;   

private:
    Input *m_input;
};