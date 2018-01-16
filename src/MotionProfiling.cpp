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

	m_notifier = &MotionProfiling::PeriodicFeed();
	m_notifier.StartPeriodic(0.005);
}

void MotionProfiling::PeriodicFeed()
{

}

void MotionProfiling::Init()
{

}

MotionProfiling::~MotionProfiling()
{

}

