/*
 * MotionProfileAction.h
 *
 *  Created on: Apr 10, 2017
 *      Author: mikec
 */

#ifndef SRC_AUTO_ACTION_MOTIONPROFILEACTION_H_
#define SRC_AUTO_ACTION_MOTIONPROFILEACTION_H_
#include "Action.h"
#include "../../RobotModel.h"
#include "../../MotionController.h"

class MotionProfileAction : public Action{
public:
	MotionProfileAction(RobotModel *robot, MotionController *motion, double timeout);
    bool IsFinished();
    void Update();
    void Done();
    void Start();
private:
    RobotModel *robot;
    MotionController *motion;
    double timeout;
};

#endif /* SRC_AUTO_ACTION_MOTIONPROFILEACTION_H_ */
