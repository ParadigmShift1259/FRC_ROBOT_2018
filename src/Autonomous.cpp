/*
 * Autonomous.cpp
 *
 *  Created on: Jan 20, 2018
 *      Author: Matt
 */

#include "Autonomous.h"
#include "Const.h"
#include <cmath>


Autonomous::Autonomous(OperatorInputs *inputs, DriveTrain *drivetrain, DrivePID *drivepid, Intake* intake, Lifter *lifter)
{
	m_inputs = inputs;
	m_drivetrain = drivetrain;
	m_drivepid = drivepid;
	m_intake = intake;
	m_lifter = lifter;

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

	case kAutoRightScaleRight:
		AutoRightScaleRight();
		break;

	case kAutoRightScaleLeft:
		AutoRightScaleLeft();
		break;

	case kAutoLeftScaleRight:
		AutoLeftScaleRight();
		break;

	case kAutoLeftScaleLeft:
		AutoLeftScaleLeft();
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

	case kAngle:
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
		if (((targetdistance - m_distance) <= deceldistance) || (m_drivetrain->GetMaxVelocity() < 10))
		{
			m_timer.Reset();
			m_straightstate = kDecel;
		}
		else
		{
			m_drivepid->Drive(-1 * autopower);
		}
		break;

	case kDecel:
		// decelerate until target distance minus some fudge factor
		// abort decelerate if decelerate time + 1s has passed
		if ((m_distance > (targetdistance - 5.0)) || (timervalue > (acceltime+1)) || (m_drivetrain->GetMaxVelocity() < 10))
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
			double power = (acceltime - timervalue) < 0 ? (autopower > 0 ? 0.1 : -0.1) : (acceltime - timervalue) / acceltime * autopower;
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
		if (m_drivepid->IsOnTarget(5))
		{
			m_drivepid->DisablePID();
			m_turnstate = kInit;
			return true;
		}
		break;
	}
	return false;
}


bool Autonomous::MiniStraight(double targetdistance, double autopower)
{
	switch (m_straightstate)
	{
	case kStart:
		// accelerates during this case for a duration specified by ACCEL_TIME, feeds into kMaintain
		m_drivetrain->ResetDeltaDistance();
		m_distance = abs(m_drivetrain->GetMaxDeltaDistance());
		SmartDashboard::PutNumber("MiniDistance", m_distance);
		m_drivepid->Init(m_pid[0], m_pid[1], m_pid[2], DrivePID::Feedback::kGyro);
		m_drivepid->EnablePID();
		m_drivepid->SetAbsoluteAngle(0);
		m_timer.Reset();
		m_straightstate = kMaintain;
		break;

	case kAngle:
	case kAccel:
	case kMaintain:
	case kDecel:
		// decelerate until target distance minus some fudge factor
		// abort decelerate if decelerate time + 1s has passed
		m_distance = 0.0;
		if (m_timer.Get() > 0.1)
			m_distance = abs(m_drivetrain->GetMaxDeltaDistance());
		SmartDashboard::PutNumber("MiniDistance", m_distance);
		if (m_distance > (targetdistance))
		{
			m_drivepid->Drive(0);
			m_drivepid->DisablePID();
			m_drivetrain->Drive(0, 0, false);
			m_straightstate = kStart;
			return true;
		}
		else
		{
			m_drivepid->Drive(-1 * autopower, true);
		}
		break;
	}
	return false;
}


bool Autonomous::AngleStraight(double angle, double targetdistance, double acceltime, double autopower, double deceldistance)
{
	double timervalue = m_timer.Get();
	m_distance = abs(m_drivetrain->GetMaxDistance());

	switch (m_straightstate)
	{
	case kStart:
		// accelerates during this case for a duration specified by ACCEL_TIME, feeds into kMaintain
		m_drivepid->Init(m_pid[0], m_pid[1], m_pid[2], DrivePID::Feedback::kGyro);
		m_drivepid->EnablePID();
		m_drivepid->SetAbsoluteAngle(angle);
		m_straightstate = kAngle;
		break;

	case kAngle:
		m_drivepid->Drive(0, false);
		if (m_drivepid->OnTarget())
		{
			m_drivepid->DisablePID();
			m_drivetrain->ResetLeftPosition();
			m_drivetrain->ResetRightPosition();
			m_timer.Reset();
			m_timermod = acceltime;
			m_drivepid->EnablePID();
			m_straightstate = kAccel;
		}
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
		if (((targetdistance - m_distance) <= deceldistance) || (m_drivetrain->GetMaxVelocity() < 10))
		{
			m_timer.Reset();
			m_straightstate = kDecel;
		}
		else
		{
			m_drivepid->Drive(-1 * autopower);
		}
		break;

	case kDecel:
		// decelerate until target distance minus some fudge factor
		// abort decelerate if decelerate time + 1s has passed
		if ((m_distance > (targetdistance - 5.0)) || (timervalue > (acceltime+1)) || (m_drivetrain->GetMaxVelocity() < 10))
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
			double power = (acceltime - timervalue) < 0 ? (autopower > 0 ? 0.1 : -0.1) : (acceltime - timervalue) / acceltime * autopower;
			m_drivepid->Drive(-1 * power);
		}
		break;
	}
	return false;
}


bool Autonomous::MiniAngleStraight(double angle, double targetdistance, double autopower)
{
	switch (m_straightstate)
	{
	case kStart:
		// accelerates during this case for a duration specified by ACCEL_TIME, feeds into kMaintain
		m_drivepid->Init(0.002, 0.0006, m_pid[2], DrivePID::Feedback::kGyro);
		m_drivepid->EnablePID();
		m_drivepid->SetAbsoluteAngle(angle);
		m_straightstate = kAngle;
		break;

	case kAngle:
		m_drivepid->Drive(0, false);
		if (m_drivepid->IsOnTarget(5))
		{
			m_drivepid->DisablePID();
			m_drivetrain->ResetDeltaDistance();
			m_drivepid->EnablePID();
			m_timer.Reset();
			m_straightstate = kMaintain;
		}
		break;

	case kAccel:
	case kDecel:
	case kMaintain:
		m_distance = 0.0;
		if (m_timer.Get() > 0.1)
			m_distance = abs(m_drivetrain->GetMaxDeltaDistance());
		SmartDashboard::PutNumber("MiniDistance", m_distance);
		if (m_distance > (targetdistance))
		{
			m_drivepid->Drive(0);
			m_drivepid->DisablePID();
			m_drivetrain->Drive(0, 0, false);
			m_straightstate = kStart;
			return true;
		}
		else
		{
			m_drivepid->Drive(-1 * autopower, true);
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
			m_autostage++;
		break;
	case 1:
		if (AngleStraight(60, 62, 0.5, 0.5, 24.0))		// targetdistance = 52", ramp = .5s, power = 50%, deceldistance = 24"
			m_autostage++;
		break;
	case 2:
		if (AngleStraight(-60, 32, 0.1, 0.25, 18.0))		// targetdistance = 32", ramp = .1s, power = 25%, deceldistance = 18"
			m_autostage++;
		break;
	case 3:
		m_intake->AutoEject();
		m_timer.Reset();
		m_autostage++;
		break;
	case 4:
		if (m_timer.Get() > 0.25)
			m_autostage++;
		break;
	case 5:
		m_lifter->AutoDeploy();
//		m_lifter->MoveBottom();
		if (MiniStraight(24, -0.5))
			m_autostage++;
		break;
	case 6:
		if (m_lifter->MoveBottom())
			m_autostage++;
		break;
	case 7:
		if (TurnAngle(-45))
		{
			m_intake->AutoIngest();
			m_autostage++;
		}
		break;
	case 8:
		if (MiniStraight(34, 0.5))
			m_autostage++;
		break;
	case 9:
		m_drivetrain->Drive(0, 0);					// turn off drive motors
		break;
	}
//	switch (m_autostage)
//	{
//	case 0:
//		if (DriveStraight(40, 0.5, 0.5, 24.0))		// targetdistance = 40", ramp = .5s, power = 50%, deceldistance = 24"
//			m_autostage++;
//		break;
//	case 1:
//		if (TurnAngle(60))							// angle = 60 (counterclockwise)
//			m_autostage++;
//		break;
//	case 2:
//		if (DriveStraight(52, 0.5, 0.5, 24.0))		// targetdistance = 64", ramp = .5s, power = 50%, deceldistance = 24"
//			m_autostage++;
//		break;
//	case 3:
//		if (TurnAngle(-60))							// angle = -60 (clockwise)
//			m_autostage++;
//		break;
//	case 4:
//		if (DriveStraight(30, 0.1, 0.25, 18.0))		// targetdistance = 30", ramp = .1s, power = 25%, deceldistance = 18"
//			m_autostage++;
//		break;
//	case 5:
//		m_intake->AutoEject();
//		m_autostage++;
//		break;
//	case 6:
//		m_drivetrain->Drive(0, 0);					// turn off drive motors
//		break;
//	}
}


void Autonomous::AutoCenterSwitchRight()
{
	switch (m_autostage)
	{
	case 0:
		if (DriveStraight(40, 0.5, 0.5, 24.0))		// targetdistance = 40", ramp = .5s, power = 50%, deceldistance = 24"
			m_autostage++;
		break;
	case 1:
		if (AngleStraight(-60, 52, 0.5, 0.5, 24.0))		// targetdistance = 52", ramp = .5s, power = 50%, deceldistance = 24"
			m_autostage++;
		break;
	case 2:
		if (AngleStraight(60, 32, 0.1, 0.25, 18.0))		// targetdistance = 32", ramp = .1s, power = 25%, deceldistance = 18"
			m_autostage++;
		break;
	case 3:
		m_intake->AutoEject();
		m_timer.Reset();
		m_autostage++;
		break;
	case 4:
		if (m_timer.Get() > 0.25)
			m_autostage++;
		break;
	case 5:
		m_lifter->AutoDeploy();
//		m_lifter->MoveBottom();
		if (MiniStraight(24, -0.5))
			m_autostage++;
		break;
	case 6:
		if (m_lifter->MoveBottom())
			m_autostage++;
		break;
	case 7:
		if (TurnAngle(45))
		{
			m_intake->AutoIngest();
			m_autostage++;
		}
		break;
	case 8:
		if (MiniStraight(24, 0.5))
			m_autostage++;
		break;
	case 9:
		m_drivetrain->Drive(0, 0);					// turn off drive motors
		break;
	}
//	switch (m_autostage)
//	{
//	case 0:
//		if (DriveStraight(40, 0.5, 0.5, 24.0))		// targetdistance = 40", ramp = .5s, power = 50%, deceldistance = 24"
//			m_autostage++;
//		break;
//	case 1:
//		if (TurnAngle(-60))							// angle = -60 (clockwise)
//			m_autostage++;
//		break;
//	case 2:
//		if (DriveStraight(52, 0.5, 0.5, 24.0))		// targetdistance = 52", ramp = .5s, power = 50%, deceldistance = 24"
//			m_autostage++;
//		break;
//	case 3:
//		if (TurnAngle(60))							// angle = 60 (counterclockwise)
//			m_autostage++;
//		break;
//	case 4:
//		if (DriveStraight(32, 0.1, 0.25, 18.0))		// targetdistance = 32", ramp = .1s, power = 25%, deceldistance = 18"
//			m_autostage++;
//		break;
//	case 5:
//		m_intake->AutoEject();
//		m_autostage++;
//		break;
//	case 6:
//		m_drivetrain->Drive(0, 0);					// turn off drive motors
//		break;
//	}
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


void Autonomous::AutoRightScaleRight()
{
	switch (m_autostage)
	{
	case 0:
		if (DriveStraight(230, 1.0, 0.5, 60.0))		// targetdistance = 290", everything else needs tuning
			m_autostage++;
		break;
	case 1:
		if (TurnAngle(35))							// angle = 45 (counter clockwise)
			m_autostage++;
		break;
	case 2:
		if (m_lifter->AutoDeploy())
			m_autostage++;
		break;
	case 3:
		if (m_lifter->AutoRaise())
			m_autostage++;
		break;
	case 4:
		m_intake->AutoEject();
		m_timer.Reset();
		m_autostage++;
		break;
	case 5:
		if (m_timer.Get() > 0.25)
			m_autostage++;
		break;
	case 6:
		m_lifter->MoveBottom();
		if (MiniStraight(24,-0.3))
			m_autostage++;
		break;
	case 7:
		if (m_lifter->MoveBottom())
			m_autostage++;
		break;
	case 8:
		m_drivetrain->Drive(0, 0);					// turn off drive motors
		break;
	}
}


void Autonomous::AutoRightScaleLeft()
{
	switch (m_autostage)
	{
	case 0:
		if (DriveStraight(209, 1.0, 0.5, 60.0))		//Goes 235, with the turn, trust me
			m_autostage++;
		break;
	case 1:
		if (TurnAngle(90))							// angle = 90, counter clockwise
			m_autostage++;
		break;
	case 2:
		if (DriveStraight(150, 1.0, 0.5, 60.0))//DriveStraight(233, 1.0, 0.5, 60.0))		// targetdistance = 230", tuned ish
			m_autostage++;
		break;
//	case 3:
//		if (TurnAngle(-125))						// angle = -105 (clockwise)
//			m_autostage++;
//		break;
//	case 4:
//		if (m_lifter->AutoDeploy())
//			m_autostage++;
//		break;
//	case 5:
//		if (DriveStraight(30, 0.1, 0.25, 18.0))		// targetdistance = 30", ramp = .1s, power = 25%, deceldistance = 18"
//			m_autostage++;
//		break;
//	case 6:
//		if (m_lifter->AutoRaise())
//			m_autostage++;
//		break;
//	case 7:
//		m_intake->AutoEject();
//		m_timer.Reset();
//		m_autostage++;
//		break;
//	case 8:
//		if (m_timer.Get() > 0.25)
//			m_autostage++;
//		break;
//	case 9:
//		m_lifter->MoveBottom();
//		if (MiniStraight(24,-0.3))
//			m_autostage++;
//		break;
//	case 10:
//		if (m_lifter->MoveBottom())
//			m_autostage++;
//		break;
//	case 11:
//		m_drivetrain->Drive(0, 0);					// turn off drive motors
//		break;
	}
}


void Autonomous::AutoLeftScaleRight()
{
	switch (m_autostage)
	{
	case 0:
		if (MiniStraight(175, 0.9))		//Goes 235, with the turn, trust me
			m_autostage++;
		break;
	case 1:
		if (MiniAngleStraight(-88, 100, 0.9))							// angle = -90, clockwise
			m_autostage++;
		break;
	case 2:
		if (m_lifter->AutoDeploy()/* && m_timer.Get() > 1.5*/)
			m_autostage++;
		break;
	case 3:
		if (TurnAngle(90))		// targetdistance = 230", tuned ish
			m_autostage++;
		break;
	case 4:
		if (m_lifter->AutoRaise())
			m_autostage++;
		break;
	case 5:
		m_intake->AutoEject();
		m_timer.Reset();
		m_autostage++;
		break;
//	case 3:
//		if (TurnAngle(125))							// angle = 105 (counter clockwise)
//			m_autostage++;
//		break;
//	case 4:
//		if (m_lifter->AutoDeploy())
//			m_autostage++;
//		break;
//	case 5:
//		if (DriveStraight(30, 0.1, 0.25, 18.0))		// targetdistance = 30", ramp = .1s, power = 25%, deceldistance = 18"
//			m_autostage++;
//		break;
//	case 6:
//		if (m_lifter->AutoRaise())
//			m_autostage++;
//		break;
//	case 7:
//		m_intake->AutoEject();
//		m_autostage++;
//		m_timer.Reset();
//		break;
//	case 8:
//		if (m_timer.Get() > 0.25)
//			m_autostage++;
//		break;
//	case 9:
//		m_lifter->MoveBottom();
//		if (MiniStraight(24,-0.3))
//			m_autostage++;
//		break;
//	case 10:
//		if (m_lifter->MoveBottom())
//			m_autostage++;
//		break;
	case 6:
		m_drivetrain->Drive(0, 0);					// turn off drive motors
		break;
	}
}


void Autonomous::AutoLeftScaleLeft()
{
	switch (m_autostage)
	{
	case 0:
		if (MiniStraight(171, 0.9))
		//if (DriveStraight(290, 1.8, 1, 250))		// targetdistance = 290", everything else needs tuning
			m_autostage++;
		break;
	case 1:
		if (TurnAngle(-15))							// angle = -45 (clockwise)
			m_autostage++;
			m_timer.Reset();
		break;
	case 2:
		if (m_lifter->AutoDeploy()/* && m_timer.Get() > 1.5*/)
			m_autostage++;
		break;
	case 3:
		if (m_lifter->AutoRaise())
			m_autostage++;
		break;
	case 4:
		m_intake->AutoEject();
		m_timer.Reset();
		m_autostage++;
		break;
	case 5:
		if (m_timer.Get() > 0.25)
			m_autostage++;
		break;
	case 6:
		m_lifter->MoveBottom();
		if (MiniStraight(12,-0.3))
			m_autostage++;
		break;
	case 7:
		m_lifter->MoveBottom();
		if (TurnAngle(-110))
			m_autostage++;
		break;
	case 8:
		m_lifter->MoveBottom();
		m_intake->AutoIngest();
		if(MiniStraight(15,0.9))
		{
			m_timer.Reset();
			m_autostage++;
		}
		break;
	case 9:
		if (m_timer.Get() > 1)
		{
			m_intake->FinishAutoIngest();
			m_autostage++;
		}
		break;
	case 10:
		m_lifter->AutoRaise();
		if (MiniStraight(15, -0.9))
			m_autostage++;
		break;
	case 11:
		m_lifter->AutoRaise();
		if (TurnAngle(95))
			m_autostage++;
		break;
	case 12:
		if (m_lifter->AutoRaise())
			m_autostage++;
		break;
	case 13:

		if (MiniStraight(15, 0.9))
			m_autostage++;
		break;
	case 14:
		m_intake->AutoEject();
		m_timer.Reset();
		m_autostage++;
		break;
	case 15:
		if (m_timer.Get() > 0.25)
			m_autostage++;
		break;
	case 16:
		if (MiniStraight(24,-0.3))
			m_autostage++;
		m_lifter->MoveBottom();
		break;
	case 17:
		m_lifter->MoveBottom();
		break;
	}
}


void Autonomous::AutoTest()
{
	switch (m_autostage)
	{
	case 0:
		if (DriveStraight(40, 0.5, 0.5, 24.0))		// targetdistance = 40", ramp = .5s, power = 50%, deceldistance = 24"
			m_autostage++;
		break;
	case 1:
		if (AngleStraight(-60, 52, 0.5, 0.5, 24.0))		// targetdistance = 52", ramp = .5s, power = 50%, deceldistance = 24"
			m_autostage++;
		break;
	case 2:
		if (AngleStraight(60, 32, 0.1, 0.25, 18.0))		// targetdistance = 32", ramp = .1s, power = 25%, deceldistance = 18"
			m_autostage++;
		break;
	case 3:
		m_intake->AutoEject();
		m_timer.Reset();
		m_autostage++;
		break;
	case 4:
		if (m_timer.Get() > 0.25)
			m_autostage++;
		break;
	case 5:
		m_lifter->AutoDeploy();
		m_lifter->MoveBottom();
		if (MiniStraight(24, -0.5))
			m_autostage++;
		break;
	case 6:
		if (m_lifter->MoveBottom())
			m_autostage++;
		break;
	case 7:
		if (TurnAngle(45))
			m_autostage++;
		break;
	case 8:
		m_drivetrain->Drive(0, 0);					// turn off drive motors
		break;
	}
//	switch (m_autostage)
//	{
//	case 0:
//		if (DriveStraight(209, 1.0, 0.5, 60.0))		//Goes 235, with the turn, trust me
//			m_autostage++;
//		break;
//	case 1:
//		if (AngleStraight(90, 100, 1.0, 0.5, 60.0))		// targetdistance = 230", tuned ish
//			m_autostage++;
//		break;
//	case 2:
//		if (DriveStraight(100, 1.0, -0.5, 60.0))		// targetdistance = 230", tuned ish
//			m_autostage++;
//		break;
//	case 3:
//		if (AngleStraight(-90, 209, 1.0, -0.5, 60.0))		// targetdistance = 230", tuned ish
//			m_autostage++;
//		break;
//	case 4:
//		m_drivetrain->Drive(0, 0);					// turn off drive motors
//		break;
//	}
}
