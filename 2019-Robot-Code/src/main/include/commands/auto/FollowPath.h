#pragma once

#include <pathfinder.h>

#include <vector>

#include <frc/commands/Command.h>

class FollowPath : public frc::Command {
public:
    FollowPath(std::vector< Segment > leftData, std::vector< Segment > rightData);

    void Initialize() override;

    void Execute() override;

    bool IsFinished() override;

    void End() override;

    void Interrupted() override;

private:
    EncoderFollower m_lFollower;
    EncoderFollower m_rFollower;

    EncoderConfig m_config;

    std::vector< Segment > m_leftData;
    std::vector< Segment > m_rightData;
};