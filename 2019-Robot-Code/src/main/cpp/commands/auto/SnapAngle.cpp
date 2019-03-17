#include "commands/auto/SnapAngle.h"
#include "Robot.h"
#include <iostream>

#include <array>
#include <algorithm>

double calcAngle(double angle) {
    const std::array< double, 8 > values = { 0, 61.25, 90, 118.75, 180, 241.25, 270, 298.75 };

    auto comp = [&angle](double lhs, double rhs) {
        return std::abs(minDifference(lhs, angle)) < std::abs(minDifference(rhs, angle));
    };

    return *std::min_element(values.begin(), values.end(), comp);
}

SnapAngle::SnapAngle() :
    TurnToAngle(calcAngle(Robot::getHeading()))
{
    /*std::cout << "=================\n";
    std::cout << "Target Angle: " << calcAngle(Robot::getHeading()) << "\n";
    std::cout << "=================\n";*/
}