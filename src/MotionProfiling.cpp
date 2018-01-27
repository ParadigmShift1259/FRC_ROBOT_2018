/*
 * MotionProfiling.cpp
 *
 *  Created on: Jan 13, 2018
 *      Author: Matt Wildman
 */

#include <MotionProfiling.h>

MotionProfiling::MotionProfiling(WPI_TalonSRX *rightFront, WPI_TalonSRX *rightBack, WPI_TalonSRX *leftFront, WPI_TalonSRX *leftBack)
{
	m_rightLeadTalon = rightFront;
	m_rightFollowTalon = rightBack;
	m_leftLeadTalon = leftFront;
	m_leftFollowTalon = leftBack;

	//////////////////Not positive this works for motion profile trajectories//////////////////
	m_rightFollowTalon->Set(ControlMode::Follower, CAN_RIGHT_PORT);
	m_leftFollowTalon->Set(ControlMode::Follower, CAN_LEFT_PORT);

	//For peace of mind
	m_rightLeadTalon->ClearMotionProfileTrajectories();
	m_leftLeadTalon->ClearMotionProfileTrajectories();

	//Set to the recommended value of double the speed of execution of motion profile
	m_rightLeadTalon->ChangeMotionControlFramePeriod(10);
	m_leftLeadTalon->ChangeMotionControlFramePeriod(10);

	//Will call StartPeriodic every 5ms
	m_notifier = new Notifier(MotionProfiling::PeriodicFeed, this);

	stage = 0;
}

/*
 * An init, to be called once before running motion profile
 */
void MotionProfiling::Init()
{
	m_leftLeadTalon->Set(ControlMode::MotionProfile,0);
	m_rightLeadTalon->Set(ControlMode::MotionProfile,0);

	feedTopBuffer();

	stage = 0;
	m_rightLeadTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,0);
	m_rightLeadTalon->ConfigNominalOutputForward(0,0);
	m_rightLeadTalon->ConfigNominalOutputReverse(-0,0);
	m_rightLeadTalon->ConfigPeakOutputForward(11.7,0);
	m_rightLeadTalon->ConfigPeakOutputReverse(-11.7,0);
	//m_rightLeadTalon->ConfigMotionCruiseVelocity(1800,0); //Completely Arbitrary
	//m_rightLeadTalon->ConfigMotionAcceleration(500,0);//Also Completely Arbitrary
	m_rightLeadTalon->Config_kP(0,KP,0);
	m_rightLeadTalon->Config_kI(0,KI,0);
	m_rightLeadTalon->Config_kD(0,KD,0);
	m_rightLeadTalon->Config_kF(0,KF,0);

	m_leftLeadTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,0);
	m_leftLeadTalon->ConfigNominalOutputForward(0,0);
	m_leftLeadTalon->ConfigNominalOutputReverse(-0,0);
	m_leftLeadTalon->ConfigPeakOutputForward(11.7,0);
	m_leftLeadTalon->ConfigPeakOutputReverse(-11.7,0);
	//m_leftLeadTalon->ConfigMotionCruiseVelocity(1800,0); //Tested at 2250, not thouroughly tested, picking 1800 so that it is achievable
	//m_leftLeadTalon->ConfigMotionAcceleration(50,0);//Arbitrary, but thought about
	m_leftLeadTalon->Config_kP(0,KP,0);
	m_leftLeadTalon->Config_kI(0,KI,0);
	m_leftLeadTalon->Config_kD(0,KD,0);
	m_leftLeadTalon->Config_kF(0,KF,0);//1023/1800

	m_leftLeadTalon->SetSelectedSensorPosition(0,0,0);
	m_rightLeadTalon->SetSelectedSensorPosition(0,0,0);

	m_rightLeadTalon->SetSensorPhase(false);
	m_leftLeadTalon->SetSensorPhase(false);

	SmartDashboard::PutNumber("P",KP);
	SmartDashboard::PutNumber("I",KI);
	SmartDashboard::PutNumber("D",KD);

	m_rightFollowTalon->Set(ControlMode::Follower, CAN_RIGHT_PORT);
	m_leftFollowTalon->Set(ControlMode::Follower, CAN_LEFT_PORT);
	m_rightFollowTalon->Follow(*m_rightLeadTalon);
	m_leftFollowTalon->Follow(*m_leftLeadTalon);
	m_rightLeadTalon->Set(NeutralMode::Coast);
	m_rightFollowTalon->Set(NeutralMode::Coast);
	m_leftLeadTalon->Set(NeutralMode::Coast);
	m_leftFollowTalon->Set(NeutralMode::Coast);

	m_notifier->StartPeriodic(0.025);
}

void MotionProfiling::Loop()
{
	switch(stage)
	{
	case 0:
		m_leftLeadTalon->GetMotionProfileStatus(leftStatus);
		m_rightLeadTalon->GetMotionProfileStatus(rightStatus);
		if(leftStatus.btmBufferCnt > 5 && rightStatus.btmBufferCnt > 5)
			stage = 1;
		break;
	case 1:
		m_leftLeadTalon->Set(1);
		m_rightLeadTalon->Set(1);
		SmartDashboard::PutNumber("LeftPos",m_leftLeadTalon->GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("RightPos",m_rightLeadTalon->GetSelectedSensorPosition(0));
		m_rightLeadTalon->GetMotionProfileStatus(rightStatus);
		if (rightStatus.isLast)
			stage = 2;
		break;
	case 2:
		SmartDashboard::PutNumber("LeftPos",m_leftLeadTalon->GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("RightPos",m_rightLeadTalon->GetSelectedSensorPosition(0));
		m_leftLeadTalon->Set(ControlMode::PercentOutput,0);
		m_rightLeadTalon->Set(ControlMode::PercentOutput,0);
	}
}

/*
 * Every time this is called the talons will process their buffer. Currently
 * called every 5ms which is twice as fast as the talons process the points
 */
void MotionProfiling::PeriodicFeed(MotionProfiling *arg)
{
	arg->m_rightLeadTalon->ProcessMotionProfileBuffer();
	arg->m_leftLeadTalon->ProcessMotionProfileBuffer();
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

	m_leftLeadTalon->ClearMotionProfileTrajectories();
	m_rightLeadTalon->ClearMotionProfileTrajectories();

	m_rightLeadTalon->ConfigMotionProfileTrajectoryPeriod(0,0);
	m_leftLeadTalon->ConfigMotionProfileTrajectoryPeriod(0,0);

	for(int i=0;i<ProfileSize;++i){

		point.position = mMotionPosition[i];
		point.velocity = mMotionVelocity[i];
		point.timeDur = TrajectoryDuration_100ms;
		point.profileSlotSelect0 = 1;

		point.zeroPos = false;
		if (i == 0)
			point.zeroPos = true;

		point.isLastPoint = false;
		if( (i + 1) == ProfileSize )
			point.isLastPoint = true;

		m_leftLeadTalon->PushMotionProfileTrajectory(point);
	}

	for(int i=0;i<ProfileSize;++i){

		point.position = mMotionPosition[i];
		point.velocity = mMotionVelocity[i];
		point.timeDur = TrajectoryDuration_100ms;
		point.profileSlotSelect0 = 1;

		point.zeroPos = false;
		if (i == 0)
			point.zeroPos = true;

		point.isLastPoint = false;
		if( (i + 1) == ProfileSize )
			point.isLastPoint = true;

		m_rightLeadTalon->PushMotionProfileTrajectory(point);
	}
}

MotionProfiling::~MotionProfiling()
{

}
