/*
 * Autonomous.cpp
 *
 *  Created on: Jan 20, 2018
 *      Author: Matt
 */

#include "Autonomous.h"
#include "Const.h"
#include <cmath>


Autonomous::Autonomous(OperatorInputs *inputs, DriveTrain *drivetrain)
{
	m_inputs = inputs;
	m_drivetrain = drivetrain;
	m_drivepid = new DrivePID(m_drivetrain, m_inputs);

	m_timerstraight = new Timer();
	m_timerstraight->Reset();

	m_straightstate = kStart;
	m_distance = 0;
	m_acceldistance = 0;
	m_timermod = ACCEL_TIME;
	m_notifier = new Notifier(Autonomous::VelocityAdjust, this);
	timervalue = 0;
	distance = 0;
	m_target = 0;
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

	m_drivepid->Init(0.003, 0.0005, 0.0, true);			//Make Constants in future
}


void Autonomous::Loop()
{
	DriveStraight(30000);
//	m_drivepid->Drive(-0.7,true);
}


/*!
 * Drives straight the specified number of ticks and returns true when the function is done.
 * not guarenteed to be perfectly accurate but is pretty close. Will not try to target after
 * done with the state machine and will hold the incorrect value.
 */
bool Autonomous::DriveStraight(double targetdistance)
{
	SmartDashboard::PutNumber("LeftEncoder", m_drivetrain->GetLeftPosition());
	SmartDashboard::PutNumber("RightEncoder", m_drivetrain->GetRightPosition());

	m_target = targetdistance;

	timervalue = (((int)(m_timerstraight->Get() * 50)) / 50.0); //!<Stores the current timer value

	double leftdistance = m_drivetrain->GetLeftPosition();
	double rightdistance = m_drivetrain->GetRightPosition();
	distance = abs((abs(leftdistance) > abs(rightdistance)) ? leftdistance : rightdistance);   //!< Stores the absolute value of the greatest encoder distance

	switch (m_straightstate)
	{
	/*
	 * Accelerates during this case for a duration specified by ACCEL_TIME, feeds into kMaintain unless
	 * 1/3rd the target distance is reached in which case kDecel is moved into
	 */
	case kStart:
		m_drivetrain->ResetLeftPosition();
		m_drivetrain->ResetRightPosition();
		Wait(0.25);
		m_drivepid->EnablePID();
		m_timerstraight->Start();
		m_straightstate = kAccel;
		m_timermod = ACCEL_TIME;
		distance = 0.0;		// @suppress("No break at end of case")

	case kAccel:
		//If acceleration has reached max time
		if (timervalue > ACCEL_TIME)			//Quantizes the timer value into increments of 20ms
		{
			m_acceldistance = distance;
			m_straightstate = kMaintain;
			m_timerstraight->Reset();
			m_timerstraight->Start();
			SmartDashboard::PutNumber("StopAccel", distance);
		}
		else
		{
			//Handles the case where a triangle acceleration is required
			if (distance > targetdistance / 3)
			{
				m_timermod = timervalue;
				m_timerstraight->Reset();
				m_timerstraight->Start();
				m_straightstate = kDecel;
			}
			else
			{
				m_drivepid->Drive(-1 * timervalue / ACCEL_TIME * AUTO_POWER);
//				m_drivetrain->Drive(0, -1 * timervalue / ACCEL_TIME, false);
				break;
			}
		} // @suppress("No break at end of case")

		/*
		 * Maintaines top speed until 2x the acceldistance away from targetdistance
		 */
	case kMaintain:
		if ((targetdistance - distance) <= DECEL_DISTANCE)
		{
			m_straightstate = kDecel;
			m_timerstraight->Reset();
			m_timerstraight->Start();
			timervalue = 0;
			SmartDashboard::PutNumber("StartDecel", distance);
		}
		else if (targetdistance - ((m_drivetrain->LeftTalonLead()->GetSelectedSensorVelocity(0) / 5) + distance) <= DECEL_DISTANCE)
		{
			m_notifier->StartSingle((targetdistance - m_drivetrain->GetLeftPosition()) / (m_drivetrain->LeftTalonLead()->GetSelectedSensorVelocity(0) * 10));
		}
		else
		{
			m_drivepid->Drive(-AUTO_POWER);
//			m_drivetrain->Drive(0, -1, false);
			break;
		} // @suppress("No break at end of case")

	case kDecel:
		if ((timervalue > m_timermod) || (distance > (targetdistance - 4)))
		{
			m_drivepid->Drive(0);
			m_drivepid->DisablePID();
			m_drivetrain->Drive(0, 0, false);
			SmartDashboard::PutNumber("DecelDistance", distance - SmartDashboard::GetNumber("StartDecel",0));
			return true;
		}
		else
		{
			m_drivepid->Drive(-1 * ((ACCEL_TIME - timervalue) / ACCEL_TIME) * AUTO_POWER);
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
	arg->timervalue = 0;
	arg->DriveStraight(arg->m_target);
	SmartDashboard::PutNumber("StartDecel", arg->distance);
}


void Autonomous::Stop()
{
	m_drivepid->Stop();
}
