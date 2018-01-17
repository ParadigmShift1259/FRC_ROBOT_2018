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

	rightBackTalon->Follow(*rightFrontTalon);
	leftBackTalon->Follow(*leftFrontTalon);

	rightFrontTalon->ClearMotionProfileTrajectories();
	leftFrontTalon->ClearMotionProfileTrajectories();

	rightFrontTalon->ChangeMotionControlFramePeriod(5);
	leftFrontTalon->ChangeMotionControlFramePeriod(5);

	m_notifier = new Notifier(MotionProfiling::PeriodicFeed, this);
	m_notifier->StartPeriodic(0.005);
}

void MotionProfiling::PeriodicFeed(MotionProfiling *arg)
{
	arg->rightFrontTalon->ProcessMotionProfileBuffer();
	arg->leftFrontTalon->ProcessMotionProfileBuffer();
}

void MotionProfiling::feedTopBuffer()
{
	TrajectoryPoint point;

	for(int i=0;i<ProfileSize;++i){

		point.position = mMotionProfile[i][0];
		point.velocity = mMotionProfile[i][1];
		point.timeDur = TrajectoryDuration::TrajectoryDuration_10ms;
		point.profileSlotSelect = 1;

		point.zeroPos = false;
		if (i == 0)
			point.zeroPos = true; /* set this to true on the first point */

		point.isLastPoint = false;
		if( (i + 1) == ProfileSize )
			point.isLastPoint = true; /*
										 * set this to true on the last point
										 */


		/////// THIS WILL NEED TO BE CHANGED FOR TURNS
		leftFrontTalon->PushMotionProfileTrajectory(point);
		rightFrontTalon->PushMotionProfileTrajectory(point);
	}
}

void MotionProfiling::Init()
{

}

MotionProfiling::~MotionProfiling()
{

}

