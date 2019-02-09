#include "commands/FollowPath.h"
#include "subsystems/DriveTrain.h"
#include "Robot.h"

FollowPath::FollowPath(std::vector< Segment > leftData, std::vector< Segment > rightData) :
    m_lFollower(EncoderFollower{ 0, 0, 0, 0, 0 }),
    m_rFollower(EncoderFollower{ 0, 0, 0, 0, 0 }),
    m_config(EncoderConfig{ 0, 96, 0.6383528 /* ish */, 1.0, 0.0, 0.0, 1.0 / 15.0, 0.0 }),
    m_leftData(leftData),
    m_rightData(rightData),
    Command("FollowPath", *Robot::m_driveTrain.get())
{

}

void FollowPath::Initialize() {

    Robot::m_driveTrain->resetSensors();

    // TODO: Measure wheel circumference
    // TODO: Adjust gains
    // TODO: Adjust max speed
    // m_config

}

void FollowPath::Execute() {
    double l = pathfinder_follow_encoder(
        m_config,
        &m_lFollower,
        m_leftData.data(),
        m_leftData.size(),
        Robot::m_driveTrain->getEncoderPositions().first[1]
    );

    double r = pathfinder_follow_encoder(
        m_config,
        &m_rFollower,
        m_rightData.data(),
        m_rightData.size(),
        Robot::m_driveTrain->getEncoderPositions().second[1]
    );

    // TODO: Apparently you need a gyro????

    Robot::m_driveTrain->drive(l, r);
}

bool FollowPath::IsFinished() {
    return false;
}

void FollowPath::End() {
    Robot::m_driveTrain->drive(0.0, 0.0);
}

void FollowPath::Interrupted() {
    End();
}

double pathfinder_follow_encoder(EncoderConfig c, EncoderFollower *follower, Segment *trajectory, int trajectory_length, int encoder_tick) {
    return 0.0;
}