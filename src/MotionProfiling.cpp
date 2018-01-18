/*
 * MotionProfiling.cpp
 *
 *  Created on: Jan 13, 2018
 *      Author: Matt Wildman
 */

#include <MotionProfiling.h>

MotionProfiling::MotionProfiling(WPI_TalonSRX *rightFront, WPI_TalonSRX *rightBack, WPI_TalonSRX *leftFront, WPI_TalonSRX *leftBack)
{
	rightFrontTalon = rightFront;
	rightBackTalon = rightBack;
	leftFrontTalon = leftFront;
	leftBackTalon = leftBack;

	//////////////////Not positive this works for motion profile trajectories//////////////////
	rightBackTalon->Follow(*rightFrontTalon);
	leftBackTalon->Follow(*leftFrontTalon);

	//For peace of mind
	rightFrontTalon->ClearMotionProfileTrajectories();
	leftFrontTalon->ClearMotionProfileTrajectories();

	//Set to the recommended value of double the speed of execution of motion profile
	rightFrontTalon->ChangeMotionControlFramePeriod(5);
	leftFrontTalon->ChangeMotionControlFramePeriod(5);

	//Will call StartPeriodic every 5ms
	m_notifier = new Notifier(MotionProfiling::PeriodicFeed, this);
	m_notifier->StartPeriodic(0.005);

	stage = 0;
}

/*
 * An init, to be called once before running motion profile
 */
void MotionProfiling::Init()
{
	feedTopBuffer();

	leftFrontTalon->Set(ControlMode::MotionProfile,0);
	rightFrontTalon->Set(ControlMode::MotionProfile,0);

	stage = 0;
}

void MotionProfiling::Loop()
{
	switch(stage)
	{
	case 0:
		leftFrontTalon->GetMotionProfileStatus(leftStatus);
		rightFrontTalon->GetMotionProfileStatus(rightStatus);
		if(leftStatus.btmBufferCnt > 5 && rightStatus.btmBufferCnt > 5)
			stage = 1;
		break;
	case 1:
		leftFrontTalon->Set(1);
		rightFrontTalon->Set(1);
		break;
	}
}

/*
 * Every time this is called the talons will process their buffer. Currently
 * called every 5ms which is twice as fast as the talons process the points
 */
void MotionProfiling::PeriodicFeed(MotionProfiling *arg)
{
	arg->rightFrontTalon->ProcessMotionProfileBuffer();
	arg->leftFrontTalon->ProcessMotionProfileBuffer();
}

/*
 * Feed our motion profile points from Profile.h into the Talon
 *
 * Assumes that the entire motion profile is contained within two
 * two dimensional arrays. 1 is the left array and 2 is the
 * right array. They also must be the same size
 */
void MotionProfiling::feedTopBuffer()
{
	TrajectoryPoint point;

	leftFrontTalon->ClearMotionProfileTrajectories();
	rightFrontTalon->ClearMotionProfileTrajectories();

	for(int i=0;i<ProfileSize;++i){

		point.position = mMotionProfile1[i][0];
		point.velocity = mMotionProfile1[i][1];
		point.timeDur = TrajectoryDuration_10ms;
		point.profileSlotSelect0 = 1;

		point.zeroPos = false;
		if (i == 0)
			point.zeroPos = true;

		point.isLastPoint = false;
		if( (i + 1) == ProfileSize )
			point.isLastPoint = true;

		leftFrontTalon->PushMotionProfileTrajectory(point);
	}

	for(int i=0;i<ProfileSize;++i){

		point.position = mMotionProfile2[i][0];
		point.velocity = mMotionProfile2[i][1];
		point.timeDur = TrajectoryDuration_10ms;
		point.profileSlotSelect0 = 1;

		point.zeroPos = false;
		if (i == 0)
			point.zeroPos = true;

		point.isLastPoint = false;
		if( (i + 1) == ProfileSize )
			point.isLastPoint = true;

		rightFrontTalon->PushMotionProfileTrajectory(point);
	}

}

MotionProfiling::~MotionProfiling()
{

}

