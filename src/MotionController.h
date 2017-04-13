/*
 * MotionController.h
 *
 *  Created on: Apr 8, 2017
 *      Author: mikec
 */

#ifndef SRC_MOTIONCONTROLLER_H_
#define SRC_MOTIONCONTROLLER_H_

#include <pathfinder.h>
#include "RobotModel.h"

class MotionController {
public:
	MotionController(RobotModel *robot);

	void InitBeforeOutput();
	void UpdateOutputs();
	void GeneratePath();
	double GetLeftOutput();
	double GetRightOutput();

	int POINT_LENGTH = 2;
	double wheelbase_width = 0.644525; //distance from center of left wheels to center of right wheels in meters (25.375 in)

private:

	RobotModel *robot;

	Waypoint p1;
	Waypoint p2;
	Waypoint p3;
	Waypoint p4;
	Waypoint p5;
	Waypoint p6;

	int length;
	double leftOutput, rightOutput;

	Segment *leftTrajectory, *rightTrajectory;

	Segment *trajectory;
	EncoderFollower *leftFollower, *rightFollower;

	EncoderConfig leftConfig, rightConfig;

	TrajectoryCandidate candidate;
};

#endif /* SRC_MOTIONCONTROLLER_H_ */
