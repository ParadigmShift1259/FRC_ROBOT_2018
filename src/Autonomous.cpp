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

	m_stage = kIdle;
	m_turning = false;

//	m_timerstraight = new Timer();
//	m_timerstraight->Reset();

//	m_straightstate = kStart;
//	m_distance = 0;
//	m_acceldistance = 0;
//	m_timermod = ACCEL_TIME;
}


Autonomous::~Autonomous()
{
	delete m_drivepid;
//	delete m_timerstraight;
}


void Autonomous::Init()
{
//	if (SmartDashboard::GetNumber("AutoDistance", 0) == 0)
//		SmartDashboard::PutNumber("AutoDistance", 150);

//	m_straightstate = kStart;
//	m_distance = SmartDashboard::GetNumber("AutoDistance", 0);
//	m_acceldistance = 0;
//	m_timermod = ACCEL_TIME;
//	m_timerstraight->Reset();
//	m_timerstraight->Start();

	m_stage = kTurn;
	m_drivetrain->ResetPositions();
	Wait(0.25);
	m_drivepid->Init(0.01, 0.001, 0.0, true);			//Make Constants in future
}


void Autonomous::Loop()
{
	switch (m_stage)
	{
	case kIdle:
			break;

	case kStraight:
			if (GoStraight(60, -0.5))
				m_stage = kIdle;
			break;

	case kTurn:
			if (TurnDegree(120))
				m_stage = kIdle;
			break;
	};
//	DriveStraight(35000);
//	m_drivepid->Drive(-0.7,true);
}


bool Autonomous::GoStraight(double inches, double power)
{
	double leftdistance = m_drivetrain->GetLeftDistance();
	double rightdistance = m_drivetrain->GetRightDistance();

	double distance = inches;

	double distancetotarget = abs(distance) - (abs((leftdistance - rightdistance) / 2));

	if (distancetotarget <= (0.5 * abs(distance)))
		power = 0.25 * (power/abs(power));

	SmartDashboard::PutNumber("AU1_distance", distancetotarget);
	SmartDashboard::PutNumber("AU2_power", power);

	if (distancetotarget <= 5)
	{
		m_drivepid->Stop();
		m_drivepid->Drive(0);
		m_drivetrain->Drive(0, 0);
		return true;
	}
	m_drivepid->Drive(power, true);
	return false;
}


bool Autonomous::TurnDegree(double degrees)
{
	DriverStation::ReportError("TurnDegree");

	if (!m_turning)
	{
		m_turning = true;
		m_drivepid->SetRelativeAngle(degrees);
	}
	else
	if (m_drivepid->OnTarget())
	{
		DriverStation::ReportError("OnTarget");
		m_drivepid->Stop();
		m_drivepid->Drive(0);
		m_drivetrain->Drive(0, 0);
		m_turning = false;
		return true;
	}
	return false;
}


/*!
 * Drives straight the specified number of ticks and returns true when the function is done.
 * not guarenteed to be perfectly accurate but is pretty close. Will not try to target after
 * done with the state machine and will hold the incorrect value.
 */
/*
bool Autonomous::DriveStraight(double targetdistance)
{
	SmartDashboard::PutNumber("LeftEncoder", m_drivetrain->GetLeftPosition());
	SmartDashboard::PutNumber("RightEncoder", m_drivetrain->GetRightPosition());

	double timervalue = (((int)(m_timerstraight->Get() * 50)) / 50.0); //!<Stores the current timer value

	double leftdistance = m_drivetrain->GetLeftPosition();
	double rightdistance = m_drivetrain->GetRightPosition();
	double distance = abs((abs(leftdistance) > abs(rightdistance)) ? leftdistance : rightdistance);   //!< Stores the absolute value of the greatest encoder distance

	switch (m_straightstate)
	{
	//
	// Accelerates during this case for a duration specified by ACCEL_TIME, feeds into kMaintain unless
	// 1/3rd the target distance is reached in which case kDecel is moved into
	//
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
				m_drivepid->Drive(-1 * timervalue / ACCEL_TIME *.5);
//				m_drivetrain->Drive(0, -1 * timervalue / ACCEL_TIME, false);
				break;
			}
		} // @suppress("No break at end of case")

		//
		// Maintaines top speed until 2x the acceldistance away from targetdistance
		//
	case kMaintain:
		if ((targetdistance - distance) <= 17000)
		{
			m_straightstate = kDecel;
			m_timerstraight->Reset();
			m_timerstraight->Start();
			timervalue = 0;
			SmartDashboard::PutNumber("StartDecel", distance);
		}
		else
		{
			m_drivepid->Drive(-0.5);
//			m_drivetrain->Drive(0, -1, false);
			break;
		} // @suppress("No break at end of case")

		//
		// Decelerates over the course of ACCEL_TIME which happens to be about
		// 2x the acceleration distance
		//
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
			m_drivepid->Drive(-1 * (m_timermod - timervalue) / ACCEL_TIME * .5);
//			m_drivetrain->Drive(0, -1 * (m_timermod - timervalue) / ACCEL_TIME, false);
		}
	}
	return false;
}
*/


void Autonomous::Stop()
{
	m_drivepid->Stop();
}
