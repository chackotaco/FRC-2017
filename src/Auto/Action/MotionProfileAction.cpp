/*
 * MotionProfileAction.cpp
 *
 *  Created on: Apr 10, 2017
 *      Author: mikec
 */

#include <Auto/Action/MotionProfileAction.h>
#include "WPILib.h"

MotionProfileAction::MotionProfileAction(RobotModel *robot, MotionController *motion, double timeout) {
	this->robot = robot;
	this->motion = motion;
	this->timeout = timeout;
}

bool MotionProfileAction::IsFinished() {
    return Timer::GetFPGATimestamp() >= start_time + timeout;

}

void MotionProfileAction::Update() {
	motion->UpdateOutputs();
	double leftOutput = motion->GetLeftOutput();
	double rightOutput = motion->GetRightOutput();

	robot->SetWheelSpeed(RobotModel::LeftWheels, leftOutput);
	robot->SetWheelSpeed(RobotModel::RightWheels, rightOutput);
    SmartDashboard::PutNumber("runningmotionACTION", Timer::GetFPGATimestamp());
}
void MotionProfileAction::Done() {
	robot->SetWheelSpeed(RobotModel::LeftWheels, 0.0);
	robot->SetWheelSpeed(RobotModel::RightWheels, 0.0);

}
void MotionProfileAction::Start() {
    start_time = Timer::GetFPGATimestamp();
    motion->InitBeforeOutput();
    SmartDashboard::PutString("INITmotionACTION", "yep");
}

