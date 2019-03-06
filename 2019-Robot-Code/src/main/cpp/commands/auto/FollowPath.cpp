#include "commands/auto/FollowPath.h"
#include "subsystems/DriveTrain.h"
#include "Robot.h"
#include <iostream>

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

    m_lFollower = EncoderFollower{ 0, 0, 0, 0, 0 };
    m_rFollower = EncoderFollower{ 0, 0, 0, 0, 0 };

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

    //std::cout << "Follower: " << desired_heading << ", Gyro: " << gyro_heading << "\n";

    double angle_difference = minDifference(Robot::getHeading(), r2d(m_lFollower.heading));
    
    double turn = 1.0 * (-1.0/80.0) * angle_difference;

    Robot::m_driveTrain->drive(l + turn, r - turn);
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