/*
 * MotionController.cpp
 *
 *  Created on: Apr 8, 2017
 *      Author: mikec
 */

#include "MotionController.h"
#include "pathfinder.h"
#include "Params.h"
#include "WPILib.h"

MotionController::MotionController(RobotModel *robot) {
	this->robot = robot;

	Waypoint *points = (Waypoint*) malloc(sizeof(Waypoint) * POINT_LENGTH);

	p1 = { 0, 0, d2r(-90) }; // Waypoint @ x=-4, y=-1, exit angle=45 degrees
	p2 = { 0, 1, d2r(-90) }; // Waypoint @ x=-1, y= 2, exit angle= 0 radians

	//Waypoint p3 = { 2, 4, 0 }; // Waypoint @ x= 2, y= 4, exit angle= 0 radians
	points[0] = p1;
	points[1] = p2;
	//points[2] = p3;

	// Prepare the Trajectory for Generation.
	//
	// Arguments:
	// Fit Function:        FIT_HERMITE_CUBIC or FIT_HERMITE_QUINTIC
	// Sample Count:        PATHFINDER_SAMPLES_HIGH (100 000)
	//                      PATHFINDER_SAMPLES_LOW  (10 000)
	//                      PATHFINDER_SAMPLES_FAST (1 000)
	// Time Step:           0.001 Seconds
	// Max Velocity:        15 m/s
	// Max Acceleration:    10 m/s/s
	// Max Jerk:            60 m/s/s/s
	SmartDashboard::PutNumber("MOTION_STATE", 0);
	pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC,
	PATHFINDER_SAMPLES_LOW, 0.02, ROBOT_VELOCITY_MAX, 10.0, 60.0, &candidate);

	length = candidate.length;
	SmartDashboard::PutNumber("candidate.length", length);
	SmartDashboard::PutNumber("MOTION_STATE", 1);

	// Array of Segments (the trajectory points) to store the trajectory in
	trajectory = (Segment *) malloc(length * sizeof(Segment));

	SmartDashboard::PutNumber("MOTION_STATE", 2);
	// Generate the trajectory
	pathfinder_generate(&candidate, trajectory);
	SmartDashboard::PutNumber("MOTION_STATE", 3);

	leftTrajectory = (Segment*) malloc(sizeof(Segment) * length);
	rightTrajectory = (Segment*) malloc(sizeof(Segment) * length);

	// Generate the Left and Right trajectories of the wheelbase using the
	// originally generated trajectory
	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory,
			wheelbase_width);
	SmartDashboard::PutNumber("MOTION_STATE", 4);

	leftFollower = (EncoderFollower*) malloc(sizeof(EncoderFollower));
	leftFollower->last_error = 0;
	leftFollower->segment = 0;
	leftFollower->finished = 0;     // Just in case!

	rightFollower = (EncoderFollower*) malloc(sizeof(EncoderFollower));
	rightFollower->last_error = 0;
	rightFollower->segment = 0;
	rightFollower->finished = 0;     // Just in case!

	leftOutput = 0.0;
	rightOutput = 0.0;

}

void MotionController::InitBeforeOutput() {
	robot->leftDriveEncoder->Reset();
	robot->rightDriveEncoder->Reset();

	leftConfig = {robot->leftDriveEncoder->Get(), ENCODER_PULSES_PER_REV, WHEELS_CIRCUMFERENCE_METERS, // Position, Ticks per Rev, Wheel Circumference
		1.0, 0.0, 0.0, 1.0 / ROBOT_VELOCITY_MAX, 0.0};  // Kp, Ki, Kd and Kv, Ka
	rightConfig = {robot->rightDriveEncoder->Get(), ENCODER_PULSES_PER_REV, WHEELS_CIRCUMFERENCE_METERS, // Position, Ticks per Rev, Wheel Circumference
		1.0, 0.0, 0.0, 1.0 / ROBOT_VELOCITY_MAX, 0.0};  // Kp, Ki, Kd and Kv, Ka
}

void MotionController::UpdateOutputs() {

	// Arg 1: The EncoderConfig
	// Arg 2: The EncoderFollower for this side
	// Arg 3: The Trajectory generated from `pathfinder_modify_tank`
	// Arg 4: The Length of the Trajectory (length used in Segment seg[length];)
	// Arg 5: The current value of your encoder
	double leftEncoderValue = robot->leftDriveEncoder->Get();
    double rightEncoderValue = robot->rightDriveEncoder->Get();
	leftOutput = pathfinder_follow_encoder(leftConfig, leftFollower,
			leftTrajectory, length, leftEncoderValue);
	rightOutput = pathfinder_follow_encoder(rightConfig, rightFollower,
			rightTrajectory, length, rightEncoderValue);
	SmartDashboard::PutNumber("MOTIONCONTROLLER_LEFT_OUT", leftOutput);
	SmartDashboard::PutNumber("MOTIONCONTROLLER_RIGHT_OUT", rightOutput);
}

double MotionController::GetLeftOutput() {
	return leftOutput;
}

double MotionController::GetRightOutput() {
	return rightOutput;
}