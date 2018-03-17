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
	m_curvestate = kCurveStart;
	m_autostage = 0;

	m_timermod = 0;
	m_distance = 0;
	m_seg = 1;
	m_accelskip = false;
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
	m_curvestate = kCurveStart;
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
			m_drivepid->Drive(-1 * autopower);
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
			double power = (acceltime - timervalue) < 0 ? (autopower > 0 ? 0.1 : -0.1) : (acceltime - timervalue) / acceltime * autopower;
			m_drivepid->Drive(-1 * power);
		}
		break;
	}
	return false;
}


bool Autonomous::CurveAuto(double a, double b, double s, double chu, double accel, double deceldist, double autopower)
{
	double timervalue = m_timer.Get();
	/*
	double r;
	double pi = 3.14159265359;
	double ang;
	double tdist = m_tdist;
	double x, y;
	double prevx, prevy;
	double rang = m_rang;
	double dist1;
	double dist2;
	double rdist = m_rdist;
	//double rdist1 = 0;
	//double rdist2 = 0;
	double xp;
	double yp;
	double acceltime = autopower / 2;
	double segtime = m_segtime;
	*/
	s = s / 360;
	double pi = 3.14159265359;
	double seg = m_seg;
	bool accelskip = m_accelskip;
	double ang = s * seg * (2 * pi) / (chu + 1);
	double r = (abs(a) * abs(b)) / sqrt(abs(a) * abs(a) * sin(ang) * sin(ang) + abs(b) * abs(b) * cos(ang) * cos(ang));
	double xp;
	double yp;
	double prevx;
	double prevy;
	if (seg > 1)
	{
		xp = r * cos(ang * seg);
		yp = r * sin(ang * (seg - 1));
		prevx = r * cos(ang * (seg - 1));
		prevy = r * sin(ang * (seg - 1));
	}
	else
	{
		xp = 0;
		yp = 0;
		prevx = 0;
		prevy = 0;
	}
	double tdist = pi * s * (abs(a) + abs(b)) * (3 * ((abs(a) - abs(b)) * (abs(a) - abs(b)) / (abs(a) + abs(b)) * (abs(a) + abs(b)) * (sqrt(-3 * ((abs(a) - abs(b)) * (abs(a) - abs(b)) / (abs(a) + abs(b)) * (abs(a) + abs(b))) + 14))) + 1);
	double x = r * cos(ang * seg);
	double y = r * sin(ang * seg);
	double acceltime = autopower / 2;
	double dist1 = sqrt((r - r * cos(ang) + (a > 0 ? - xp : + xp))*(r - r * cos(ang) + (a > 0 ? - xp : + xp))+(r * sin(ang) + (b > 0 ? - yp : + yp))*(r * sin(ang) + (b > 0 ? - yp : + yp)));		// sqrt(x^2 + y^2)
	double dist2 = tdist * ((x / y) / (abs(a) / abs(b)) - (prevx / prevy) / (abs(a) / abs(b)));	// circumference * portion of circle / total angle segments
	double rdist = ((dist1 + dist2) / 2) /*+ (2 * pi * ang) * WHEEL_TRACK / (2) * (360)*/;
	//rdist1 = rdist;
	//rdist2 = rdist - (2 * pi * ang) * WHEEL_TRACK / 360;
	double segtime = 100;			// change
	double rang = atan((r * sin(ang) + (b > 0 ? - yp : + yp))/(r - r * cos(ang) + (a > 0 ? - xp : + xp)));
	rang = a > 0 ? rang * -1 : rang;
	m_distance = abs(m_drivetrain->GetAverageMaxDistance());

	switch (m_curvestate)
	{
	case kCurveStart:
		// accelerates during this case for a duration specified by ACCEL_TIME, feeds into kMaintain
		m_drivetrain->ResetLeftPosition();
		m_drivetrain->ResetRightPosition();
		m_drivepid->Init(m_pid[0], m_pid[1], m_pid[2], DrivePID::Feedback::kGyro);
		m_drivepid->EnablePID();
		m_timer.Reset();
		m_curvestate = kCalculate;
		break;

	case kCalculate:

		m_drivepid->SetAbsoluteAngle(rang * 180 / (2 * pi));
		m_curvestate = kDrive;
		break;

/*
	case kAccel:
		if (accel == true || accelskip == false)
		{
			// if acceleration has reached max time
			if (timervalue > (acceltime))
			{
				m_timer.Reset();
				m_straightstate = kMaintain;
			}
			else
			{
				m_drivepid->Drive(-1 * timervalue / acceltime * autopower); // change
			}
		}
		else
			m_straightstate = kMaintain;
		break;
*/

	case kDrive:
		if (accel == true || accelskip == false)			/// acceleration
		{
			// if acceleration has reached max time
			if (timervalue > (acceltime))
			{
				m_timer.Reset();
				accelskip = true;
			}
			else
			{
				if (((rdist - m_distance) < 0) && m_drivepid->OnTarget())
				{
					seg++;
					m_curvestate = kCalculate;
					m_drivetrain->ResetLeftPosition();
					m_drivetrain->ResetRightPosition();
				}
				else
				{
					m_drivepid->Drive(-1 * timervalue / acceltime * autopower); // change
				}
			}
		}
		// maintain until decel distance
		if ((tdist - m_distance) > (deceldist) || (timervalue > (segtime + 1)))			/// maintain
		{
			if (((rdist - m_distance) < 0) && m_drivepid->OnTarget())
			{
				m_timer.Reset();
				seg++;
				if (chu + 1 == seg)
					m_curvestate = kStop;
				else
					m_curvestate = kCalculate;
					m_drivetrain->ResetLeftPosition();
					m_drivetrain->ResetRightPosition();
			}
			else
			{
				m_drivepid->Drive(-1 * autopower);
			}
		}
		else
		{
			if ((m_distance > tdist) || (timervalue > (acceltime + 1)))			/// deceleration
			{
				if (((rdist - m_distance) < 0) && m_drivepid->OnTarget())
				{
					seg++;
					if (chu + 1 == seg)
						m_curvestate = kStop;
					else
						m_curvestate = kCalculate;
						m_drivetrain->ResetLeftPosition();
						m_drivetrain->ResetRightPosition();
				}
				else
				{
					double power = ((acceltime) - timervalue) < 0 ? (autopower > 0 ? 0.1 : -0.1) : (acceltime - timervalue) / acceltime * autopower;
					m_drivepid->Drive(-1 * power);
				}
			}
		}
		m_seg = seg;
		m_accelskip = accelskip;

		break;

	case kStop:
		m_drivepid->Drive(0);
		m_drivepid->DisablePID();
		m_drivetrain->Drive(0, 0, false);
		m_curvestate = kCurveStart;
		seg = 1;
		m_seg = seg;
		return true;
		break;
/*
	case kDecel:
		if (decel == true)
		{
			// decelerate until target distance minus some fudge factor
			// abort decelerate if decelerate time + 1s has passed
			if ((m_distance > (rdist - 5.0)) || (timervalue > (acceltime+1)))
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
				double power = ((acceltime) - timervalue) < 0 ? (autopower > 0 ? 0.1 : -0.1) : (acceltime - timervalue) / acceltime * autopower;
				m_drivepid->Drive(-1 * power);
			}
		}
		else
			m_straightstate = kStart;
			return true;
		break;
		*/
	}
	return false;
	SmartDashboard::PutNumber("CurveDistance", rdist);
	SmartDashboard::PutNumber("CurveAngle", rang);
	SmartDashboard::PutNumber("CurveRadius", r);
	SmartDashboard::PutNumber("TotalCurve", rang * 180 / (2 * pi));
	SmartDashboard::PutNumber("SegmentNumber", m_seg);
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
	/*
	switch (m_autostage)
	{
	case 0:
		if (DriveStraight(40, 0.5, 0.5, 24.0))		// targetdistance = 40", ramp = .5s, power = 50%, deceldistance = 24"
			m_autostage++;
		break;
	case 1:
		if (TurnAngle(60))							// angle = 60 (counterclockwise)
			m_autostage++;
		break;
	case 2:
		if (DriveStraight(64, 0.5, 0.5, 24.0))		// targetdistance = 64", ramp = .5s, power = 50%, deceldistance = 24"
			m_autostage++;
		break;
	case 3:
		if (TurnAngle(-60))							// angle = -60 (clockwise)
			m_autostage++;
		break;
	case 4:
		if (DriveStraight(30, 0.1, 0.25, 18.0))		// targetdistance = 30", ramp = .1s, power = 25%, deceldistance = 18"
			m_autostage++;
		break;
	case 5:
		m_intake->AutoEject();
		m_autostage++;
		break;
	case 6:
		m_drivetrain->Drive(0, 0);					// turn off drive motors
		break;
	}
	*/
	switch (m_autostage)
	{
	case 0:
		if(CurveAuto(-40.0, 40.0, -90, 5, 0.5, 24.0, 0.5))	// 40 left, 40 up, 90 degrees left, 5 portions, 0.5 acceltime, 24 decel distance, 0.5 power
			m_autostage++;
		break;
	case 1:
		m_drivetrain->Drive(0, 0);
		break;
	}
}


void Autonomous::AutoCenterSwitchRight()
{
	/*
	switch (m_autostage)
	{
	case 0:
		if (DriveStraight(40, 0.5, 0.5, 24.0))		// targetdistance = 40", ramp = .5s, power = 50%, deceldistance = 24"
			m_autostage++;
		break;
	case 1:
		if (TurnAngle(-60))							// angle = -60 (clockwise)
			m_autostage++;
		break;
	case 2:
		if (DriveStraight(52, 0.5, 0.5, 24.0))		// targetdistance = 52", ramp = .5s, power = 50%, deceldistance = 24"
			m_autostage++;
		break;
	case 3:
		if (TurnAngle(60))							// angle = 60 (counterclockwise)
			m_autostage++;
		break;
	case 4:
		if (DriveStraight(32, 0.1, 0.25, 18.0))		// targetdistance = 32", ramp = .1s, power = 25%, deceldistance = 18"
			m_autostage++;
		break;
	case 5:
		m_intake->AutoEject();
		m_autostage++;
		break;
	case 6:
		m_drivetrain->Drive(0, 0);					// turn off drive motors
		break;
	}
	*/
	switch (m_autostage)
	{
	case 0:
		if(CurveAuto(40.0, 40.0, 90, 5, 0.5, 24.0, 0.5))	// 40 right, 40 up, 90 degrees right, 5 portions, 0.5 acceltime, 24 decel distance, 0.5 power
			m_autostage++;
		break;
	case 1:
		m_drivetrain->Drive(0, 0);
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


void Autonomous::AutoRightScaleRight()
{
	switch (m_autostage)
	{
	case 0:
		if (DriveStraight(252, 1.0, 0.5, 60.0))		// targetdistance = 290", everything else needs tuning
			m_autostage++;
		break;
	case 1:
		if (TurnAngle(45))							// angle = 45 (counter clockwise)
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
		m_autostage++;
		break;
	case 5:
		if (m_lifter->MoveBottom())
			m_autostage++;
		break;
	case 6:
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
		if (DriveStraight(233, 1.0, 0.5, 60.0))		// targetdistance = 230", tuned ish
			m_autostage++;
		break;
	case 3:
		if (TurnAngle(-135))						// angle = -105 (clockwise)
			m_autostage++;
		break;
	case 4:
		if (m_lifter->AutoDeploy())
			m_autostage++;
		break;
	case 5:
		if (DriveStraight(30, 0.1, 0.25, 18.0))		// targetdistance = 30", ramp = .1s, power = 25%, deceldistance = 18"
			m_autostage++;
		break;
	case 6:
		if (m_lifter->AutoRaise())
			m_autostage++;
		break;
	case 7:
		m_intake->AutoEject();
		m_autostage++;
		break;
	case 8:
		if (m_lifter->MoveBottom())
			m_autostage++;
		break;
	case 9:
		m_drivetrain->Drive(0, 0);					// turn off drive motors
		break;
	}
}


void Autonomous::AutoLeftScaleRight()
{
	switch (m_autostage)
	{
	case 0:
		if (DriveStraight(209, 1.0, 0.5, 60.0))		//Goes 235, with the turn, trust me
			m_autostage++;
		break;
	case 1:
		if (TurnAngle(-88))							// angle = -90, clockwise
			m_autostage++;
		break;
	case 2:
		if (DriveStraight(233, 1.0, 0.5, 60.0))		// targetdistance = 230", tuned ish
			m_autostage++;
		break;
	case 3:
		if (TurnAngle(135))							// angle = 105 (counter clockwise)
			m_autostage++;
		break;
	case 4:
		if (m_lifter->AutoDeploy())
			m_autostage++;
		break;
	case 5:
		if (DriveStraight(30, 0.1, 0.25, 18.0))		// targetdistance = 30", ramp = .1s, power = 25%, deceldistance = 18"
			m_autostage++;
		break;
	case 6:
		if (m_lifter->AutoRaise())
			m_autostage++;
		break;
	case 7:
		m_intake->AutoEject();
		m_autostage++;
		break;
	case 8:
		if (m_lifter->MoveBottom())
			m_autostage++;
		break;
	case 9:
		m_drivetrain->Drive(0, 0);					// turn off drive motors
		break;
	}
}


void Autonomous::AutoLeftScaleLeft()
{
	switch (m_autostage)
	{
	case 0:
		if (DriveStraight(252, 1.0, 0.5, 60.0))		// targetdistance = 290", everything else needs tuning
			m_autostage++;
		break;
	case 1:
		if (TurnAngle(-45))							// angle = -45 (clockwise)
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
		m_autostage++;
		break;
	case 5:
		if (m_lifter->MoveBottom())
			m_autostage++;
		break;
	case 6:
		m_drivetrain->Drive(0, 0);					// turn off drive motors
		break;
	}
}


void Autonomous::AutoTest()
{
	switch (m_autostage)
	{
	case 0:
		if (DriveStraight(100, 0.25, 0.5, 24.0))
			m_autostage++;
		break;
	case 1:
		if (DriveStraight(100, 0.25, -0.5, 24.0))
			m_autostage++;
		break;
	case 2:
		m_drivetrain->Drive(0, 0);
		break;
	}
}
