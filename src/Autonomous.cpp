/*
 * Autonomous.cpp
 *
 *  Created on: Jan 20, 2018
 *      Author: Matt
 */

#include "Autonomous.h"
#include "Const.h"
#include <cmath>


Autonomous::Autonomous(OperatorInputs *inputs, DriveTrain *drivetrain, DrivePID *drivepid, Intake* intake)
{
	m_inputs = inputs;
	m_drivetrain = drivetrain;
	m_drivepid = drivepid;
	m_intake = intake;

	m_straightstate = kStart;
	m_turnstate = kInit;
	m_autostage = 0;

	m_timermod = 0;
	m_distance = 0;
}


Autonomous::~Autonomous()
{
}


void Autonomous::Init()
{
	DriverStation::ReportError("AutonomousInit");

	m_timer.Reset();
	m_timer.Start();

	m_straightstate = kStart;
	m_turnstate = kInit;
	m_autostage = 0;

	m_timermod = 0;
	m_distance = 0;

	m_drivepid->Init();
}


void Autonomous::Loop()
{
	switch (automode)
	{
	case kAutoDefault:
		break;

	case kAutoCenterSwitchLeft:
		AutoCenterSwitchLeft();
		break;

	case kAutoCenterSwitchRight:
		AutoCenterSwitchRight();
		break;

	case kAutoStraight:
		AutoStraight();
		break;

	case kAutoTest:
		AutoTest();
		break;
	}

	SmartDashboard::PutNumber("AU00_stage", m_autostage);
	SmartDashboard::PutNumber("AU01_leftinches", m_drivetrain->GetLeftPosition()/CODES_PER_INCH);
	SmartDashboard::PutNumber("AU02_rightinches", m_drivetrain->GetRightPosition()/CODES_PER_INCH);
	SmartDashboard::PutNumber("AU03_leftposition", m_drivetrain->GetLeftPosition());
	SmartDashboard::PutNumber("AU04_leftposition", m_drivetrain->GetRightPosition());
	SmartDashboard::PutNumber("AU05_distance", abs(m_drivetrain->GetMaxDistance()));
}


void Autonomous::Stop()
{
	m_drivepid->Stop();
}


bool Autonomous::DriveStraight(double targetdistance, double acceltime, double autopower, double deceldistance)
{
	double timervalue = m_timer.Get();
	m_distance = abs(m_drivetrain->GetMaxDistance());

	switch (m_straightstate)
	{
	case kStart:
		// accelerates during this case for a duration specified by ACCEL_TIME, feeds into kMaintain
		m_drivetrain->ResetLeftPosition();
		m_drivetrain->ResetRightPosition();
		m_drivepid->Init(m_pid[0], m_pid[1], m_pid[2], DrivePID::Feedback::kGyro);
		m_drivepid->EnablePID();
		m_drivepid->SetAbsoluteAngle(0);
		m_timer.Reset();
		m_timermod = acceltime;
		m_straightstate = kAccel;
		break;

	case kAccel:
		// if acceleration has reached max time
		if (timervalue > acceltime)
		{
			m_timer.Reset();
			m_straightstate = kMaintain;
		}
		else
		{
			m_drivepid->Drive(-1 * timervalue / acceltime * autopower);
		}
		break;

	case kMaintain:
		// maintain until decel distance
		if ((targetdistance - m_distance) <= deceldistance)
		{
			m_timer.Reset();
			m_straightstate = kDecel;
		}
		else
		{
			m_drivepid->Drive(-autopower);
		}
		break;

	case kDecel:
		// decelerate until target distance minus some fudge factor
		// abort decelerate if decelerate time + 1s has passed
		if ((m_distance > (targetdistance - 5.0)) || (timervalue > (acceltime+1)))
		{
			m_drivepid->Drive(0);
			m_drivepid->DisablePID();
			m_drivetrain->Drive(0, 0, false);
			m_straightstate = kStart;
			return true;
		}
		else
		{
			// make sure power never goes negative if time is longer than decel time
			double power = (acceltime - timervalue) < 0 ? 0.1 : (acceltime - timervalue) / acceltime * autopower;
			m_drivepid->Drive(-1 * power);
		}
		break;
	}
	return false;
}


bool Autonomous::TurnAngle(double angle)
{
	switch (m_turnstate)
	{
	case kInit:
		m_drivepid->Init(m_pid[0], m_pid[1], m_pid[2], DrivePID::Feedback::kGyro);
		m_drivepid->EnablePID();
		m_drivepid->SetAbsoluteAngle(angle);
		m_turnstate = kTurning;
		break;

	case kTurning:
		m_drivepid->Drive(0, false);
		if (m_drivepid->OnTarget())
		{
			m_drivepid->DisablePID();
			m_turnstate = kInit;
			return true;
		}
		break;
	}
	return false;
}


void Autonomous::AutoCenterSwitchLeft()
{
	switch (m_autostage)
	{
	case 0:
		if (DriveStraight(40, 0.5, 0.5, 24.0))		// targetdistance = 40", ramp = .5s, power = 50%, deceldistance = 24"
			m_autostage = 1;
		break;
	case 1:
		if (TurnAngle(60))							// angle = 60 (counterclockwise)
			m_autostage = 2;
		break;
	case 2:
		if (DriveStraight(64, 0.5, 0.5, 24.0))		// targetdistance = 64", ramp = .5s, power = 50%, deceldistance = 24"
			m_autostage = 3;
		break;
	case 3:
		if (TurnAngle(-60))							// angle = -60 (clockwise)
			m_autostage = 4;
		break;
	case 4:
		if (DriveStraight(30, 0.5, 0.25, 12.0))		// targetdistance = 30", ramp = .5s, power = 25%, deceldistance = 12"
			m_autostage = 5;
		break;
	case 5:
		m_intake->AutoEject();
		m_autostage = 6;
		break;
	case 6:
		m_drivetrain->Drive(0, 0);					// turn off drive motors
		break;
	}
}


void Autonomous::AutoCenterSwitchRight()
{
	switch (m_autostage)
	{
	case 0:
		if (DriveStraight(40, 0.5, 0.5, 24.0))		// targetdistance = 44", ramp = .5s, power = 50%, deceldistance = 24"
			m_autostage = 1;
		break;
	case 1:
		if (TurnAngle(-60))							// angle = -60 (clockwise)
			m_autostage = 2;
		break;
	case 2:
		if (DriveStraight(52, 0.5, 0.5, 24.0))		// targetdistance = 52", ramp = .5s, power = 50%, deceldistance = 24"
			m_autostage = 3;
		break;
	case 3:
		if (TurnAngle(60))							// angle = 60 (counterclockwise)
			m_autostage = 4;
		break;
	case 4:
		if (DriveStraight(32, 0.5, 0.25, 12.0))		// targetdistance = 30", ramp = .5s, power = 25%, deceldistance = 12"
			m_autostage = 5;
		break;
	case 5:
		m_intake->AutoEject();
		m_autostage = 6;
		break;
	case 6:
		m_drivetrain->Drive(0, 0);					// turn off drive motors
		break;
	}
}


void Autonomous::AutoStraight()
{
	switch (m_autostage)
	{
	case 0:
		if (DriveStraight(88, 0.5, 0.5, 24.0))		// targetdistance = 88", ramp = .5s, power = 50%, deceldistance = 24"
			m_autostage = 1;
		break;
	case 1:
		m_drivetrain->Drive(0, 0);
		break;
	}
}


void Autonomous::AutoTest()
{
	switch (m_autostage)
	{
	case 0:
		if (DriveStraight(100, 0.5, 0.5, 24.0))
			m_autostage = 1;
		break;
	case 1:
		m_drivetrain->Drive(0, 0);
		break;
	}
}
