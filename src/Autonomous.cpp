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
	case kAutoCenterSwitchLeft1:
		AutoCenterSwitchLeft1();
		break;
	case kAutoCenterSwitchRight1:
		AutoCenterSwitchRight1();
		break;
	case kAutoCenterSwitchLeft3:
		AutoCenterSwitchLeft3();
		break;
	case kAutoCenterSwitchRight3:
		AutoCenterSwitchRight3();
		break;
	case kAutoRightScaleRight2:
		AutoRightScaleRight2();
		break;
	case kAutoRightScaleLeft1:
		AutoRightScaleLeft1();
		break;
	case kAutoLeftScaleRight1:
		AutoLeftScaleRight1();
		break;
	case kAutoLeftScaleLeft2:
		AutoLeftScaleLeft2();
		break;
	case kAutoLeftScaleLeft1:
		AutoLeftScaleLeft1();
		break;
	case kAutoRightScaleRight1:
		AutoRightScaleRight1();
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
	SmartDashboard::PutNumber("AU05_distance", abs(m_drivetrain->GetMaxDeltaDistance()));
}


void Autonomous::Stop()
{
	m_drivepid->Stop();
}



// old straight code
bool Autonomous::RampStraight(double targetdistance, double acceltime, double autopower, double deceldistance)
{
	double timervalue = m_timer.Get();
	m_distance = abs(m_drivetrain->GetMaxDeltaDistance());

	switch (m_straightstate)
	{
	case kStart:
		// accelerates during this case for a duration specified by ACCEL_TIME, feeds into kMaintain
		m_drivetrain->ResetDeltaDistance();
		m_drivepid->Init(m_pidold[0], m_pidold[1], m_pidold[2], DrivePID::Feedback::kGyro);
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


// old angle straight code
bool Autonomous::AngleStraight(double angle, double targetdistance, double acceltime, double autopower, double deceldistance)
{
	double timervalue = m_timer.Get();
	m_distance = abs(m_drivetrain->GetMaxDeltaDistance());

	switch (m_straightstate)
	{
	case kStart:
		// accelerates during this case for a duration specified by ACCEL_TIME, feeds into kMaintain
		m_drivepid->Init(m_pidold[0], m_pidold[1], m_pidold[2], DrivePID::Feedback::kGyro);
		m_drivepid->EnablePID();
		m_drivepid->SetAbsoluteAngle(angle);
		m_straightstate = kAngle;
		break;

	case kAngle:
		m_drivepid->Drive(0, false);
		if (m_drivepid->IsOnTarget())
		{
			m_drivepid->DisablePID();
			m_drivetrain->ResetDeltaDistance();
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


bool Autonomous::MiniStraight(double targetdistance, double autopower, bool reset)
{
	double modifier;

	switch (m_straightstate)
	{
	case kStart:
		// accelerates during this case for a duration specified by ACCEL_TIME, feeds into kMaintain
		m_drivetrain->ResetDeltaDistance();
		m_distance = m_drivetrain->GetMaxDeltaDistance();
		SmartDashboard::PutNumber("MiniDistance", m_distance);
		m_drivepid->Init(m_pidstraight[0], m_pidstraight[1], m_pidstraight[2], DrivePID::Feedback::kGyro, reset);
		m_drivepid->EnablePID();
		if (reset)
			m_drivepid->SetAbsoluteAngle(0);
		m_timer.Reset();
		m_straightstate = kMaintain;
		break;

	case kAngle:
	case kAccel:
	case kMaintain:
	case kDecel:
		m_distance = 0;
		if (m_timer.Get() > 0.1)
			m_distance = m_drivetrain->GetMaxDeltaDistance();
		SmartDashboard::PutNumber("MiniDistance", m_distance);
		modifier = (targetdistance > 0) ? 1 : -1;
		m_distance *= modifier;
		targetdistance *= modifier;

		if (m_distance > targetdistance)
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


bool Autonomous::TurnAngle(double angle, bool reset)
{
//	double p = SmartDashboard::GetNumber("P", 0);
//	double i = SmartDashboard::GetNumber("I", 0);
//	double d = SmartDashboard::GetNumber("D", 0);

	switch (m_turnstate)
	{
	case kInit:
		// m_drivepid->Init(p, i, d, DrivePID::Feedback::kGyro);

		m_drivepid->Init(m_pidangle[0], m_pidangle[1], m_pidangle[2], DrivePID::Feedback::kGyro, reset);
		m_drivepid->EnablePID();
		m_drivepid->SetRelativeAngle(angle);
		m_turnstate = kTurning;
		break;

	case kTurning:
		m_drivepid->Drive(0, false);
		if (m_drivepid->IsOnTarget(3))
		{
			m_drivepid->DisablePID();
			m_turnstate = kInit;
			return true;
		}
		break;
	}
	return false;
}


void Autonomous::AutoCenterSwitchLeft1()
{
	switch (m_autostage)
	{
	case 0:
		m_pidangle[0] = m_pidold[0];
		m_pidangle[1] = m_pidold[1];
		m_pidangle[2] = m_pidold[2];

		if (RampStraight(40, 0.5, 0.5, 24.0))		// targetdistance = 40", ramp = .5s, power = 50%, deceldistance = 24"
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
		if (MiniStraight(-24, -0.5))
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
}


void Autonomous::AutoCenterSwitchRight1()
{
	switch (m_autostage)
	{
	case 0:
		m_pidangle[0] = m_pidold[0];
		m_pidangle[1] = m_pidold[1];
		m_pidangle[2] = m_pidold[2];

		if (RampStraight(40, 0.5, 0.5, 24.0))		// targetdistance = 40", ramp = .5s, power = 50%, deceldistance = 24"
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
		if (MiniStraight(-24, -0.5))
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
}


void Autonomous::AutoCenterSwitchLeft3()
{
	switch (m_autostage)
	{
	case 0:
		m_pidangle[0] = m_pidswitch[0];
		m_pidangle[1] = m_pidswitch[1];
		m_pidangle[2] = m_pidswitch[2];

		m_lifter->AutoDeploy();
		if (MiniStraight(3, 0.6, true))		/// RampStraight(10, 0.1, 0.3, 6)
			m_autostage++;
		break;
	case 1:
		if (TurnAngle(30, false))
			m_autostage++;
		break;
	case 2:
		if (MiniStraight(78, 0.6, false))		// original: 92
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
		m_lifter->MoveBottom();
		if (MiniStraight(-24, -0.8, false))		/// original: 44.5
			m_autostage++;
		break;
	case 6:
		m_lifter->MoveBottom();
		if (TurnAngle(-45, false))
			m_autostage++;
		break;
	case 7:
		m_lifter->MoveBottom();
		m_intake->AutoIngest();
		if (MiniStraight(10, 0.6, false))
			m_autostage++;
		break;
	case 8:
		m_timer.Reset();
		m_timer.Start();
		m_autostage++;
		break;
	case 9:
		if(m_timer.HasPeriodPassed(0.4))
		{
			m_intake->FinishAutoIngest();
			m_autostage++;
		}
		else
			m_drivetrain->Drive(0, 0);
		break;
	case 10:
		m_lifter->AutoRaiseSwitch();
		if (MiniStraight(-10, -0.6, false))
			m_autostage++;
		break;
	case 11:
		m_lifter->AutoRaiseSwitch();
		if (TurnAngle(45, false))
			m_autostage++;
		break;
	case 12:
		m_lifter->AutoRaiseSwitch();
		if (MiniStraight(58, 1, false))		/// added 7 inches to compensate for turning
			m_autostage++;
		break;
	case 13:
		m_intake->AutoEject();		///Second Eject
		m_timer.Reset();
		m_autostage++;
		break;

	case 14:
		if (m_timer.Get() > 0.25)
		{
			m_timer.Reset();
			m_autostage++;
		}
		break;
	case 15:
		if (MiniStraight(-12, -1, false))
			m_autostage++;
		if (m_timer.Get() > 0.25)
			m_lifter->MoveBottom();
		break;
	case 16:
		m_lifter->MoveBottom();
		if (TurnAngle(-48, false))
			m_autostage++;
		break;
	case 17:
		m_lifter->MoveBottom();
		m_intake->AutoIngest();
		if (MiniStraight(20, 0.6, false))
			m_autostage++;
		break;
	case 18:
		m_timer.Reset();
		m_timer.Start();
		m_autostage++;
		break;
	case 19:
		if(m_timer.HasPeriodPassed(0.5))
		{
			m_intake->FinishAutoIngest();
			m_autostage++;
		}
		else
			m_drivetrain->Drive(0, 0);
		break;
	case 20:
		m_lifter->AutoRaiseSwitch();
		if (MiniStraight(-10, -0.6, false))
			m_autostage++;
		break;

	case 21:
		m_lifter->AutoRaiseSwitch();
		if (TurnAngle(48, false))
			m_autostage++;
		break;
	case 22:
		m_lifter->AutoRaiseSwitch();
		if (MiniStraight(38, 1, false))		/// added 7 inches to compensate for turning
			m_autostage++;
		break;
	case 23:
		m_intake->AutoEject();
		m_timer.Reset();
		m_autostage++;
		break;
	case 24:
		m_drivetrain->Drive(0, 0);
		break;
	}
}


void Autonomous::AutoCenterSwitchRight3()
{
	switch (m_autostage)
	{
	case 0:
		m_pidangle[0] = m_pidswitch[0];
		m_pidangle[1] = m_pidswitch[1];
		m_pidangle[2] = m_pidswitch[2];

		m_lifter->AutoDeploy();
		if (MiniStraight(5, 0.6, true))		/// RampStraight(10, 0.1, 0.3, 6)
			m_autostage++;
		break;
	case 1:
		if (TurnAngle(-20, false))
			m_autostage++;
		break;
	case 2:
		if (MiniStraight(80, 0.6, false))		// original: 92
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
		m_lifter->MoveBottom();
		if (MiniStraight(-22, -0.8, false))		/// original: 44.5
			m_autostage++;
		break;
	case 6:
		m_lifter->MoveBottom();
		if (TurnAngle(45, false))
			m_autostage++;
		break;
	case 7:
		m_lifter->MoveBottom();
		m_intake->AutoIngest();
		if (MiniStraight(20, 0.6, false))
			m_autostage++;
		break;
	case 8:
		m_timer.Reset();
		m_timer.Start();
		m_autostage++;
		break;
	case 9:
		if(m_timer.HasPeriodPassed(0.4))
		{
			m_intake->FinishAutoIngest();
			m_autostage++;
		}
		else
			m_drivetrain->Drive(0, 0);
		break;
	case 10:
		m_lifter->AutoRaiseSwitch();
		if (MiniStraight(-15, -0.6, false))
			m_autostage++;
		break;
	case 11:
		m_lifter->AutoRaiseSwitch();
		if (TurnAngle(-45, false))
			m_autostage++;
		break;
	case 12:
		m_lifter->AutoRaiseSwitch();
		if (MiniStraight(45, 1, false))		/// added 7 inches to compensate for turning
			m_autostage++;
		break;
	case 13:
		m_intake->AutoEject();		///Second Eject
		m_timer.Reset();
		m_autostage++;
		break;
	case 14:
		if (m_timer.Get() > 0.25)
		{
			m_timer.Reset();
			m_autostage++;
		}
		break;
	case 15:
		if (MiniStraight(-15, -1, false))
			m_autostage++;
		if (m_timer.Get() > 0.25)
			m_lifter->MoveBottom();
		break;
	case 16:
		m_lifter->MoveBottom();
		if (TurnAngle(50, false))
			m_autostage++;
		break;
	case 17:
		m_lifter->MoveBottom();
		m_intake->AutoIngest();
		if (MiniStraight(20, 0.6, false))
			m_autostage++;
		break;
	case 18:
		m_timer.Reset();
		m_timer.Start();
		m_autostage++;
		break;
	case 19:
		if(m_timer.HasPeriodPassed(0.5))
		{
			m_intake->FinishAutoIngest();
			m_autostage++;
		}
		else
			m_drivetrain->Drive(0, 0);
		break;
	case 20:
		m_lifter->AutoRaiseSwitch();
		if (MiniStraight(-5, -0.6, false))
			m_autostage++;
		break;
	case 21:
		m_lifter->AutoRaiseSwitch();
		if (TurnAngle(-50, false))
			m_autostage++;
		break;
	case 22:
		m_lifter->AutoRaiseSwitch();
		if (MiniStraight(35, 1, false))		/// added 7 inches to compensate for turning
			m_autostage++;
		break;
	case 23:
		m_intake->AutoEject();
		m_timer.Reset();
		m_autostage++;
		break;
	case 24:
		m_drivetrain->Drive(0, 0);
		break;
	}
}


void Autonomous::AutoLeftScaleLeft2()
{
	switch (m_autostage)
	{
	case 0:
		m_pidangle[0] = m_pidscale[0];
		m_pidangle[1] = m_pidscale[1];
		m_pidangle[2] = m_pidscale[2];

		if (MiniStraight(204, 0.9, true))
			m_autostage++;
		break;
	case 1:
		if (TurnAngle(-20, false))							// angle = -45 (clockwise)
		{
			m_autostage++;
			m_timer.Reset();
		}
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
		if (MiniStraight(-15, -0.3, false))
			m_autostage++;
		break;
	case 7:
		m_lifter->MoveBottom();
		if (TurnAngle(-115, false))
			m_autostage++;
		break;
	case 8:
		m_lifter->MoveBottom();
		m_intake->AutoIngest();
		if(MiniStraight(13, 0.9, false))
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
		if (MiniStraight(-20, -0.9, false))
			m_autostage++;
		break;
	case 11:
		m_lifter->AutoRaise();
		if (TurnAngle(95, false))
			m_autostage++;
		break;
	case 12:
		m_lifter->AutoRaise();
		if (MiniStraight(12, 0.9, false))
			m_autostage++;
		break;
	case 13:
		if (m_lifter->AutoRaise())
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
		if (MiniStraight(-24, -0.8, false))
			m_autostage++;
		m_lifter->MoveBottom();
		break;
	case 17:
		if (m_lifter->MoveBottom())
			m_autostage++;
		break;
	case 18:
		m_drivetrain->Drive(0, 0);
		break;
	}
}


void Autonomous::AutoLeftScaleLeft1()
{
	switch (m_autostage)
	{
	case 0:
		m_pidangle[0] = m_pidscale[0];
		m_pidangle[1] = m_pidscale[1];
		m_pidangle[2] = m_pidscale[2];

		if (MiniStraight(265, 0.6, true))
			m_autostage++;
		break;
	case 1:
		if (TurnAngle(-90, false))
		{
			m_autostage++;
			m_timer.Reset();
		}
		break;
	case 2:
		if (m_lifter->AutoDeploy() && (m_timer.Get() > 1.5))
		{
			m_autostage++;
			m_timer.Reset();
		}
		break;
	case 3:
		if (MiniStraight(-6, -0.6, false))
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
	case 6:
		if (m_timer.Get() > 0.25)
			m_autostage++;
		break;
	case 7:
		if (m_lifter->MoveBottom())
			m_autostage++;
		break;
	case 8:
		m_drivetrain->Drive(0, 0);
		break;
	}
}


void Autonomous::AutoRightScaleRight2()
{
	switch (m_autostage)
	{
	case 0:
		m_pidangle[0] = m_pidscale[0];
		m_pidangle[1] = m_pidscale[1];
		m_pidangle[2] = m_pidscale[2];

		if (MiniStraight(214, 0.9, true))
			m_autostage++;
		break;
	case 1:
		if (TurnAngle(20, false))							// angle = -45 (clockwise)
		{
			m_autostage++;
			m_timer.Reset();
		}
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
		if (MiniStraight(-15, -0.3, false))
			m_autostage++;
		break;
	case 7:
		m_lifter->MoveBottom();
		if (TurnAngle(115, false))
			m_autostage++;
		break;
	case 8:
		m_lifter->MoveBottom();
		m_intake->AutoIngest();
		if(MiniStraight(13, 0.9, false))
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
		if (MiniStraight(-20, -0.9, false))
			m_autostage++;
		break;
	case 11:
		m_lifter->AutoRaise();
		if (TurnAngle(-95, false))
			m_autostage++;
		break;
	case 12:
		m_lifter->AutoRaise();
		if (MiniStraight(12, 0.9, false))
			m_autostage++;
		break;
	case 13:
		if (m_lifter->AutoRaise())
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
		if (MiniStraight(-24, -0.8, false))
			m_autostage++;
		m_lifter->MoveBottom();
		break;
	case 17:
		if (m_lifter->MoveBottom())
			m_autostage++;
		break;
	case 18:
		m_drivetrain->Drive(0, 0);
		break;
	}
}


void Autonomous::AutoRightScaleRight1()
{
	switch (m_autostage)
	{
	case 0:
		m_pidangle[0] = m_pidscale[0];
		m_pidangle[1] = m_pidscale[1];
		m_pidangle[2] = m_pidscale[2];

		if (MiniStraight(265, 0.6, true))
			m_autostage++;
		break;
	case 1:
		if (TurnAngle(90, false))
		{
			m_autostage++;
			m_timer.Reset();
		}
		break;
	case 2:
		if (m_lifter->AutoDeploy() && (m_timer.Get() > 1.5))
		{
			m_autostage++;
			m_timer.Reset();
		}
		break;
	case 3:
		if (MiniStraight(-6, -0.6, false))
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
	case 6:
		if (m_timer.Get() > 0.25)
			m_autostage++;
		break;
	case 7:
		if (m_lifter->MoveBottom())
			m_autostage++;
		break;
	case 8:
		m_drivetrain->Drive(0, 0);
		break;
	}
}


void Autonomous::AutoLeftScaleRight1()
{
	switch (m_autostage)
	{
	case 0:
		m_pidangle[0] = m_pidscale[0];
		m_pidangle[1] = m_pidscale[1];
		m_pidangle[2] = m_pidscale[2];

		if (MiniStraight(190, 0.6, true))
			m_autostage++;
		break;
	case 1:
		if (TurnAngle(-90, false))							// angle = -90, clockwise
			m_autostage++;
		break;
	case 2:
		if (MiniStraight(170, 0.6, false))
		{
			m_timer.Reset();
			m_autostage++;
		}
		break;
	case 3:
		if (m_timer.Get() > 0.5)
		{
			if (TurnAngle(115, false))
			{
				m_timer.Reset();
				m_autostage++;
			}
		}
		break;
	case 4:
		if (m_lifter->AutoDeploy() && m_timer.Get() > 0.5)
			m_autostage++;
		break;
	case 5:
		m_lifter->AutoRaise();
		if (MiniStraight(10, 0.9, false))
			m_autostage++;
		break;
	case 6:
		if (m_lifter->AutoRaise())
		{
			m_intake->AutoEject();
			m_timer.Reset();
			m_autostage++;
		}
		break;
	case 7:
		if (m_timer.Get() > 0.25)
			m_autostage++;
		break;
	case 8:
		if (MiniStraight(-10, -0.9, false))
			m_autostage++;
		break;
	case 9:
		if (m_lifter->MoveBottom())
			m_autostage++;
		break;
	case 10:
		m_drivetrain->Drive(0, 0);					// turn off drive motors
		break;
	}
}


void Autonomous::AutoRightScaleLeft1()
{
	switch (m_autostage)
	{
	case 0:
		m_pidangle[0] = m_pidscale[0];
		m_pidangle[1] = m_pidscale[1];
		m_pidangle[2] = m_pidscale[2];

		if (MiniStraight(190, 0.6, true))
			m_autostage++;
		break;
	case 1:
		if (TurnAngle(90, false))							// angle = -90, clockwise
			m_autostage++;
		break;
	case 2:
		if (MiniStraight(170, 0.6, false))
		{
			m_timer.Reset();
			m_autostage++;
		}
		break;
	case 3:
		if (m_timer.Get() > 0.5)
		{
			if (TurnAngle(-115, false))
			{
				m_timer.Reset();
				m_autostage++;
			}
		}
		break;
	case 4:
		if (m_lifter->AutoDeploy() && m_timer.Get() > 0.5)
			m_autostage++;
		break;
	case 5:
		m_lifter->AutoRaise();
		if (MiniStraight(10, 0.9, false))
			m_autostage++;
		break;
	case 6:
		if (m_lifter->AutoRaise())
		{
			m_intake->AutoEject();
			m_timer.Reset();
			m_autostage++;
		}
		break;
	case 7:
		if (m_timer.Get() > 0.25)
			m_autostage++;
		break;
	case 8:
		if (MiniStraight(-10, -0.9, false))
			m_autostage++;
		break;
	case 9:
		if (m_lifter->MoveBottom())
			m_autostage++;
		break;
	case 10:
		m_drivetrain->Drive(0, 0);					// turn off drive motors
		break;
	}
}


void Autonomous::AutoStraight()
{
	switch (m_autostage)
	{
	case 0:
		if (RampStraight(88, 0.5, 0.5, 24.0))		// targetdistance = 88", ramp = .5s, power = 50%, deceldistance = 24"
			m_autostage = 1;
		break;
	case 1:
		m_drivetrain->Drive(0, 0);
		break;
	}
}


void Autonomous::AutoTest()
{
	double pidscale[3] = {0.013, 0.0002, 0.045};

	m_pidangle[0] = pidscale[0];
	m_pidangle[1] = pidscale[1];
	m_pidangle[2] = pidscale[2];

	switch (m_autostage)
	{
	case 0:
		if (TurnAngle(90))		// targetdistance = 40", ramp = .5s, power = 50%, deceldistance = 24"
			m_autostage++;
		break;
	case 1:
		if (TurnAngle(90, false))		// targetdistance = 40", ramp = .5s, power = 50%, deceldistance = 24"
			m_autostage++;
		break;
	case 2:
		if (TurnAngle(90, false))		// targetdistance = 40", ramp = .5s, power = 50%, deceldistance = 24"
			m_autostage++;
		break;
	case 3:
		if (TurnAngle(90, false))		// targetdistance = 40", ramp = .5s, power = 50%, deceldistance = 24"
			m_autostage++;
		break;
	case 4:
		m_drivetrain->Drive(0, 0);					// turn off drive motors
	}
}
