#pragma once

#include <frc/commands/Command.h>
#include <ctre/Phoenix.h>
#include <tuple>

class ApproachTape : public frc::Command {
public:
    ApproachTape();

    void Initialize() override;

    void Execute() override;

    bool IsFinished() override;

    void End() override;

    void Interrupted() override;

private:
    std::pair< double, bool > getTargetYaw();

};