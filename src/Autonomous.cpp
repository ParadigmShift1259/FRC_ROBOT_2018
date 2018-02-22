/*
 * Autonomous.cpp
 *
 *  Created on: Jan 20, 2018
 *      Author: Matt
 */

#include "Autonomous.h"
#include "Const.h"
#include <cmath>


Autonomous::Autonomous(OperatorInputs *inputs, DriveTrain *drivetrain, DrivePID *drivepid)
{
	m_inputs = inputs;
	m_drivetrain = drivetrain;
	m_drivepid = drivepid;

	m_timerstraight = new Timer();
	m_timerstraight->Reset();

	m_straightstate = kStart;
	m_turn = kInit;
	m_acceldistance = 0;
	m_timermod = ACCEL_TIME;
	m_notifier = new Notifier(Autonomous::VelocityAdjust, this);
	m_timervalue = 0;
	m_distance = 0;
	m_target = 0;
	stage = 0;
}


Autonomous::~Autonomous()
{
	delete m_drivepid;
	delete m_timerstraight;
}


void Autonomous::Init()
{
	DriverStation::ReportError("AutonomousInit");

	if (SmartDashboard::GetNumber("AutoDistance", 0) == 0)
		SmartDashboard::PutNumber("AutoDistance", 150);

	m_straightstate = kStart;
	m_distance = SmartDashboard::GetNumber("AutoDistance", 0);
	m_acceldistance = 0;
	m_timermod = ACCEL_TIME;
	m_timerstraight->Reset();
	stage = 0;
	m_turn = kInit;
	m_drivepid->Init(0.003, 0.0005, 0.0, true);			//Make Constants in future
}


void Autonomous::Loop()
{
	switch (stage)
	{
	case 0:
//		if(DriveStraight(10, 0.1, 0.2, 4))
		if(DriveStraight(40, 0.2, 0.3, 24.3))
			stage = 1;
//		if(DriveStraight(94.10, 1, 0.3, 51.57))
//			stage = 1;
		break;
	case 1:
		if (TurnAngle(50.39))
		{stage = 2;}
		break;
	case 2:
		if (DriveStraight(94.10, 1, 0.3, 51.57))
			stage = 3;
		break;
	case 3:
		if (TurnAngle(-50.39))
		{stage = 4;}
		break;
	case 4:
		if(DriveStraight(10, 0.1, 0.2, 4))
			stage = 5;
		break;
	}
	//	m_drivepid->Drive(-0.7,true);
}


/*!
 * Drives straight the specified number of ticks and returns true when the function is done.
 * not guarenteed to be perfectly accurate but is pretty close. Will not try to target after
 * done with the state machine and will hold the incorrect value.
 */
bool Autonomous::DriveStraight(double targetdistance, double acceltime, double autopower, double deceldistance)
{
	m_drivepid->SetRelativeAngle(0);
	SmartDashboard::PutNumber("LeftEncoder", m_drivetrain->GetLeftPosition()/CODES_PER_INCH);
	SmartDashboard::PutNumber("RightEncoder", m_drivetrain->GetRightPosition()/CODES_PER_INCH);
	m_target = targetdistance;

	m_timervalue = (((int)(m_timerstraight->Get() * 50)) / 50.0); //!<Stores the current timer value. Quantizes the timer value into increments of 20ms

	double leftdistance = m_drivetrain->GetLeftPosition()/CODES_PER_INCH;
	double rightdistance = m_drivetrain->GetRightPosition()/CODES_PER_INCH;
	m_distance = abs((abs(leftdistance) > abs(rightdistance)) ? leftdistance : rightdistance);   //!< Stores the absolute value of the greatest encoder distance

	switch (m_straightstate)
	{
	/*
	 * Accelerates during this case for a duration specified by ACCEL_TIME, feeds into kMaintain
	 */
	case kStart:
		m_drivetrain->ResetLeftPosition();
		m_drivetrain->ResetRightPosition();
		Wait(0.25);
		m_drivepid->EnablePID();
		m_timerstraight->Start();
		m_straightstate = kAccel;
		m_timermod = acceltime;
		m_distance = 0.0;		// @suppress("No break at end of case")

	case kAccel:
		//If acceleration has reached max time
		if (m_timervalue > acceltime)
		{
			m_acceldistance = m_distance;
			m_straightstate = kMaintain;
			m_timerstraight->Reset();
			m_timerstraight->Start();
		}
		else
		{
				m_drivepid->Drive(-1 * m_timervalue / acceltime * autopower);
//				m_drivetrain->Drive(0, -1 * timervalue / ACCEL_TIME, false);
				break;

		} // @suppress("No break at end of case")

	case kMaintain:
		if ((targetdistance - m_distance) <= deceldistance)
		{
			m_straightstate = kDecel;
			m_timerstraight->Reset();
			m_timerstraight->Start();
			m_timervalue = 0;
			SmartDashboard::PutNumber("StartDecel", m_distance);
		}
		/*
		 * If it appears that the robot will shoot past the proper time to decelerate
		 */
//		else if (targetdistance - ((m_drivetrain->LeftTalonLead()->GetSelectedSensorVelocity(0) / 5)/CODES_PER_INCH + m_distance) <= DECEL_DISTANCE)
//		{
//			m_notifier->StartSingle((targetdistance - m_drivetrain->GetLeftPosition()) / (m_drivetrain->LeftTalonLead()->GetSelectedSensorVelocity(0) * 10));
//			break;
//		}
		else
		{
			m_drivepid->Drive(-autopower);
//			m_drivetrain->Drive(0, -1, false);
			break;
		} // @suppress("No break at end of case")

	case kDecel:
		if ((m_timervalue > m_timermod) || (m_distance > (targetdistance - 100/CODES_PER_INCH)))
		{
			m_drivepid->Drive(0);
			m_drivepid->DisablePID();
			m_drivetrain->Drive(0, 0, false);
			SmartDashboard::PutNumber("DecelDistance", m_distance - SmartDashboard::GetNumber("StartDecel",0));
			return true;
		}
		else
		{
			m_drivepid->Drive(-1 * ((acceltime - m_timervalue) / acceltime) * autopower);
//			m_drivetrain->Drive(0, -1 * (m_timermod - timervalue) / ACCEL_TIME, false);
		}
	}
	return false;
}


void Autonomous::VelocityAdjust(Autonomous *arg)
{
	arg->m_straightstate = kDecel;
	arg->m_timerstraight->Reset();
	arg->m_timerstraight->Start();
	arg->m_timervalue = 0;
	//arg->DriveStraight(arg->m_target);
	SmartDashboard::PutNumber("StartDecel", arg->m_distance);
}


bool Autonomous::TurnAngle(double angle)
{

	SmartDashboard::PutNumber("Running", 600);


		pid[0] = SmartDashboard::GetNumber("P",pid[0]);
		m_drivepid->SetP(pid[0]);
		pid[1] = SmartDashboard::GetNumber("I",pid[1]);
		m_drivepid->SetI(pid[1]);
		pid[2] = SmartDashboard::GetNumber("D",pid[2]);
		m_drivepid->SetD(pid[2]);
		switch (m_turn)
		{
		case kInit:
			m_drivepid->Enable();
			m_drivepid->Init(pid[0], pid[1], pid[2], true);
			m_drivepid->SetRelativeAngle(angle);
			m_turn = kTurning;
			/* no break */
		case kTurning:
			m_drivepid->Drive(0,false);
			SmartDashboard::PutNumber("DriveAngle Setpoint",m_drivepid->GetSetpoint());
			if (m_drivepid->OnTarget())
			{
				return true;
			}

			break;
		}
		return false;
}


void Autonomous::Stop()
{
	m_drivepid->Stop();
}
