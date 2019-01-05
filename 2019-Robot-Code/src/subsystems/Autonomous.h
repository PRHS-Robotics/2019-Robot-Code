/*
 * Autonomous.h
 *
 *  Created on: Jan 3, 2019
 *      Author: super-tails
 */

#ifndef SRC_SUBSYSTEMS_AUTONOMOUS_H_
#define SRC_SUBSYSTEMS_AUTONOMOUS_H_

#include <cstddef>
#include <Timer.h>
#include <vector>

constexpr const std::size_t EDGES_PER_INCH_FAST = 10;
constexpr const std::size_t EDGES_PER_INCH_SLOW = 5;

/*
 * measured in native sensor units
 * +y
 *  ^
 *  |
 *  0 --> +x
 *
 */

class DriveTrain;

struct Action {
	double leftSpeed;
	double rightSpeed;
	double duration;
};

class Autonomous {
public:

	void run(DriveTrain& driveTrain);

	void execute(DriveTrain& driveTrain, Action action);

	std::vector< Action > m_actions;

private:

	Timer m_timer;
};



#endif /* SRC_SUBSYSTEMS_AUTONOMOUS_H_ */
