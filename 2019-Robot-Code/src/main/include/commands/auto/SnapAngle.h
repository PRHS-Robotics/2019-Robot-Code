#pragma once

#include "commands/auto/TurnToAngle.h"

double calcAngle(double angle);

class SnapAngle : public TurnToAngle {
public:
    SnapAngle();
};