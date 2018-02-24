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

	m_timerstraight = new Timer();
	m_timerstraight->Reset();

	m_straightstate = kStart;
	m_turn = kInit;
	m_acceldistance = 0;
	m_timermod = ACCEL_TIME;
	m_timervalue = 0;
	m_distance = 0;
	m_target = 0;
	m_stage = 0;
}


Autonomous::~Autonomous()
{
	delete m_drivepid;
	delete m_timerstraight;
}


void Autonomous::Init()
{
	DriverStation::ReportError("AutonomousInit");

	m_straightstate = kStart;
	m_acceldistance = 0;
	m_timermod = ACCEL_TIME;
	m_timerstraight->Reset();
	m_timerstraight->Start();
	m_stage = 0;
	m_turn = kInit;
	m_drivepid->Init(0.05, 0.0003, 0.05, true);			//Make Constants in future
}


void Autonomous::Loop()
{
	switch (m_stage)
	{
	case 0:
		if (DriveStraight(40, 0.5, 0.5, 24.0))
			m_stage = 1;
		break;
	case 1:
		if (TurnAngle(-60))
			m_stage = 2;
		break;
	case 2:
		if (DriveStraight(62, 0.5, 0.5, 24.0))
			m_stage = 3;
		break;
	case 3:
		if (TurnAngle(60))
			m_stage = 4;
		break;
	case 4:
		if (DriveStraight(30, 0.5, 0.25, 12.0))
			m_stage = 5;
		break;
	case 5:
		m_intake->AutoEnable();
		m_stage = 6;
		break;
	case 6:
		break;
	}
	SmartDashboard::PutNumber("AU00_stage", m_stage);
	SmartDashboard::PutNumber("AU01_leftinches", m_drivetrain->GetLeftPosition()/CODES_PER_INCH);
	SmartDashboard::PutNumber("AU02_rightinches", m_drivetrain->GetRightPosition()/CODES_PER_INCH);
	SmartDashboard::PutNumber("AU03_leftposition", m_drivetrain->GetLeftPosition());
	SmartDashboard::PutNumber("AU04_leftposition", m_drivetrain->GetRightPosition());
	SmartDashboard::PutNumber("AU05_distance", m_drivetrain->GetMaxDistance());
}


/*!
 * Drives straight the specified number of ticks and returns true when the function is done.
 * not guarenteed to be perfectly accurate but is pretty close. Will not try to target after
 * done with the state machine and will hold the incorrect value.
 */
bool Autonomous::DriveStraight(double targetdistance, double acceltime, double autopower, double deceldistance)
{
	m_target = targetdistance;
	m_timervalue = m_timerstraight->Get();
	m_distance = abs(m_drivetrain->GetMaxDistance());

	switch (m_straightstate)
	{
	case kStart:
		// Accelerates during this case for a duration specified by ACCEL_TIME, feeds into kMaintain
		m_drivetrain->ResetLeftPosition();
		m_drivetrain->ResetRightPosition();
		m_drivepid->Init();
		m_drivepid->EnablePID();
		m_drivepid->SetAbsoluteAngle(0);
		m_timerstraight->Reset();
		m_straightstate = kAccel;
		m_timermod = acceltime;
		m_straightstate = kAccel;
		break;

	case kAccel:
		// If acceleration has reached max time
		if (m_timervalue > acceltime)
		{
			m_acceldistance = m_distance;
			m_straightstate = kMaintain;
			m_timerstraight->Reset();
		}
		else
		{
			m_drivepid->Drive(-1 * m_timervalue / acceltime * autopower);
		}
		break;

	case kMaintain:
		if ((targetdistance - m_distance) <= deceldistance)
		{
			m_straightstate = kDecel;
			m_timerstraight->Reset();
			SmartDashboard::PutNumber("StartDecel", m_distance);
		}
		else
		{
			m_drivepid->Drive(-autopower);
		}
		break;

	case kDecel:
		if (m_distance > (targetdistance - 5.0))
		{
			m_drivepid->Drive(0);
			m_drivepid->DisablePID();
			m_drivetrain->Drive(0, 0, false);
			m_straightstate = kStart;
			return true;
		}
		else
		{
			m_drivepid->Drive(-1 * ((acceltime - m_timervalue) / acceltime) * autopower);
		}
		break;
	}
	return false;
}


bool Autonomous::TurnAngle(double angle)
{
	if (m_drivepid->OnTarget())
	{
		SmartDashboard::PutNumber("Ontarget", 1111);
	}
	else
		SmartDashboard::PutNumber("Ontarget", 0000);



//		pid[0] = SmartDashboard::GetNumber("P",pid[0]);
//		m_drivepid->SetP(pid[0]);
//		pid[1] = SmartDashboard::GetNumber("I",pid[1]);
//		m_drivepid->SetI(pid[1]);
//		pid[2] = SmartDashboard::GetNumber("D",pid[2]);
//		m_drivepid->SetD(pid[2]);
		switch (m_turn)
		{
		case kInit:
			m_drivepid->Init(pid[0], pid[1], pid[2], true);
			m_drivepid->EnablePID();
			m_drivepid->SetAbsoluteAngle(angle);
			m_turn = kTurning;
			/* no break */
		case kTurning:
			m_drivepid->Drive(0,false);
			SmartDashboard::PutNumber("DriveAngle Setpoint",m_drivepid->GetSetpoint());
			if (m_drivepid->OnTarget())
			{
				m_drivepid->DisablePID();
				m_turn = kInit;
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
