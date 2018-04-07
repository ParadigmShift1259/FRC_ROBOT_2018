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
	m_curvestate = kCurveStart;
	m_turnstate = kInit;
	m_autostage = 0;
	m_seg = 1;
	m_accelskip = false;

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

double Autonomous::ellipse(double a, double b)
{
    double pi = 3.14159265359;
    double h = pow(abs(a) - abs(b) , 2.0) / pow(abs(a) + abs(b) , 2.0);
    double c = abs(pi *(abs(a) + abs(b)) * (1 + (3 * h / (10 + sqrt(4 - 3 * h)))));
    return c;
}


bool Autonomous::CurveAuto(double a, double b, double por, double chu, bool accel, bool decelskip, double autopower)
{
	double timervalue = m_timer.Get();
	double pi = 3.14159265359;
    double r;

    double seg = m_seg;
    double x;
    double y;
    double xp;
    double yp;
    double dist1;
    double dist2;
    double rdist;
    double segtime;
	double acceltime = autopower / 2;
	double accelskip = m_accelskip;
	double deceldist;

    double tdistseg = 1;
    double tdist = 0;
    double tdistang = 2 * pi * (por / (chu + 1)) / 360;

    while (!(tdistseg == chu + 2))           // goes through the iterations of calculations to determine tdist every time
    {
        r = (abs(a) * abs(b)) / sqrt(pow(abs(a), 2.0) * pow(sin(tdistang), 2.0) + pow(abs(b), 2.0)* pow(cos(tdistang) , 2.0));

        x = r * cos(tdistang);
        y = r * sin(tdistang);
        xp = r * cos(tdistang * (tdistseg - 1) / tdistseg);
    	yp = r * sin(tdistang * (tdistseg - 1) / tdistseg);

    	dist1 = sqrt(pow((xp - x), 2.0) + pow((y - yp) , 2.0));           // linear path of the robot
	    dist2 = ellipse(xp - x, y - yp)/4;                              // curved path of the robot

	    rdist = ((dist1 + dist2) / 2);
        tdist = tdist + rdist;
        tdistang = tdistang + tdistang / tdistseg;
        tdistseg++;
    }

    double rang = por * seg / (chu + 1);        // reseting numbers back to original angle after tdist calculations for small curve
	double ang = rang * 2 * pi / 360;
    r = (abs(a) * abs(b)) / sqrt(pow(abs(a), 2.0) * pow(sin(ang), 2.0) + pow(abs(b), 2.0)* pow(cos(ang) , 2.0));

	x = r * cos(ang);                       // fixes the calculations to the original angle
	y = r * sin(ang);
    xp = r * cos(ang * (seg - 1) / seg);
	yp = r * sin(ang * (seg - 1) / seg);

	dist1 = sqrt(pow((xp - x), 2.0) + pow((y - yp) , 2.0));           // linear path of the robot
	dist2 = ellipse((xp - x), (y - yp)) / 4;                               // curved path of the robot

	rdist = ((dist1 + dist2) / 2);

	segtime = rdist / 20 * autopower;			// change
	deceldist = tdist / 5;

	if (a < 0 || b < 0)
		rdist = rdist * -1;				/// flips the drive direction if the bot is going backwards
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

		m_drivepid->SetAbsoluteAngle(-rang);
		m_curvestate = kDrive;
		break;

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
				if (((abs(rdist) - m_distance) < 0) && m_drivepid->OnTarget())
				{
					seg++;
					m_curvestate = kCalculate;
					m_drivetrain->ResetLeftPosition();
					m_drivetrain->ResetRightPosition();
				}
				else
				{
					m_drivepid->Drive(-1 * timervalue / acceltime * autopower * (rdist / abs(rdist))); // change
				}
			}
		}
		// maintain until decel distance
		if (((tdist - m_distance) > (deceldist) || (timervalue > (segtime + 1))) && decelskip == false)			/// maintain
		{
			if (((abs(rdist) - m_distance) < 0) && m_drivepid->OnTarget())
			{
				m_timer.Reset();
				seg++;
				if (chu + 2 == seg)
					m_curvestate = kStop;
				else
					m_curvestate = kCalculate;
					m_drivetrain->ResetLeftPosition();
					m_drivetrain->ResetRightPosition();
			}
			else
			{
				m_drivepid->Drive(-1 * autopower * (rdist / abs(rdist)));
			}
		}
		else
		{
			if ((m_distance > tdist) || (timervalue > (acceltime + 1)))			/// deceleration
			{
				m_curvestate = kStop;
			}
			else
			{
				if (((rdist - m_distance) < 0) && m_drivepid->OnTarget())
				{
					seg++;
					if (chu + 2 == seg)
						m_curvestate = kStop;
					else
						m_curvestate = kCalculate;
						m_drivetrain->ResetLeftPosition();
						m_drivetrain->ResetRightPosition();
				}
				else
				{
					double power = ((acceltime) - timervalue) < 0 ? (autopower > 0 ? 0.1 : -0.1) : (acceltime - timervalue) / acceltime * autopower;
					m_drivepid->Drive(-1 * power * (rdist / abs(rdist)));
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

	}
	return false;
	SmartDashboard::PutNumber("AUTO_CurveDistance", rdist);
	SmartDashboard::PutNumber("AUTO_CurveAngle", rang);
	SmartDashboard::PutNumber("AUTO_CurveRadius", r);
	SmartDashboard::PutNumber("AUTO_SegmentNumber", m_seg);
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


bool Autonomous::MiniStraight(double targetdistance, double autopower)
{
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
		m_straightstate = kMaintain;
		break;

	case kAngle:
	case kAccel:
	case kMaintain:
	case kDecel:
		// decelerate until target distance minus some fudge factor
		// abort decelerate if decelerate time + 1s has passed
		if (m_distance > (targetdistance - 2.0))
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

	switch (m_autostage)
	{
	case 0:
		if (CurveAuto(40, 50, 90, 1, true, true, 0.5))
			m_autostage++;
		break;
	case 1:
			m_drivetrain->Drive(0, 0);					// turn off drive motors
			break;
	}
	*/
	switch(m_autostage)
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
		if (MiniStraight(20, -0.6))
			m_autostage++;
		break;

	case 6:
		m_lifter->MoveBottom();
		if (AngleStraight(45, 50, 0.5, -0.75, 30))
			m_autostage++;
		break;

	case 7:
		if (TurnAngle(45))
			m_autostage++;
		break;

	case 8:
		if (MiniStraight(14.5, -0.6))
			m_autostage++;
		break;

	case 9:
		if (TurnAngle(-90))
			m_autostage++;
		break;

	case 10:
		if (MiniStraight(24, 0.6))
			m_autostage++;
		break;

	case 11:
		m_intake->AutoIngest();
			m_timer.Reset();
			m_timer.Start();
			m_autostage++;
		break;

	case 12:
		if(m_timer.HasPeriodPassed(0.25))
		{
			m_intake->FinishAutoIngest();
			m_autostage++;
		}
		else
			m_drivetrain->Drive(0, 0);
		break;

	case 13:
		m_lifter->AutoAuto();
		if (MiniStraight(12, -0.6))
			m_autostage++;
		break;

	case 14:
		m_lifter->AutoAuto();
		if (TurnAngle(90))
			m_autostage++;
		break;

	case 15:
		m_lifter->AutoAuto();
		if (MiniStraight(14.5, 0.6))
			m_autostage++;
		break;

	case 16:
		m_lifter->AutoAuto();
		if (AngleStraight(-45, 50, 0.5, 0.75, 30))
			m_autostage++;
		break;

	case 17:
		if (TurnAngle(-45))
			m_autostage++;
		break;

	case 18:
		if (MiniStraight(10, 0.6))
			m_autostage++;
		break;

	case 19:
		m_intake->AutoEject();
		m_timer.Reset();
		m_autostage++;
		break;

	case 20:
		if (m_timer.Get() > 0.25)
			m_autostage++;
		break;
	case 21:
		if (MiniStraight(11, -0.6))
			m_autostage++;
		break;

	case 22:
		m_lifter->MoveBottom();
		if (AngleStraight(45, 50, 0.5, -0.75, 30))
			m_autostage++;
		break;

	case 23:
		if (TurnAngle(45))
			m_autostage++;
		break;

	case 24:
		if (MiniStraight(14.5, -0.6))
			m_autostage++;
		break;

	case 25:
		if (TurnAngle(-75))
			m_autostage++;
		break;

	case 26:
		if (MiniStraight(24, 0.6))
			m_autostage++;
		break;

	case 27:
		m_intake->AutoIngest();
		m_timer.Reset();
		m_timer.Start();
		m_autostage++;
		break;

	case 28:
		if (m_timer.HasPeriodPassed(0.25))
		{
			m_intake->FinishAutoIngest();
			m_autostage++;
		}
		else
			m_drivetrain->Drive(0, 0);
		break;

	case 29:
		m_lifter->AutoAuto();
		if (MiniStraight(12, -0.6))
				m_autostage++;
		break;

	case 30:
		m_lifter->AutoAuto();
		if (TurnAngle(75))
			m_autostage++;
		break;

	case 31:
		m_lifter->AutoAuto();
		if (MiniStraight(14.5, 0.6))
			m_autostage++;
		break;

	case 32:
		m_lifter->AutoAuto();
		if (AngleStraight(-45, 50, 0.5, 0.75, 30))
			m_autostage++;
		break;

	case 33:
		if (TurnAngle(-45))
			m_autostage++;
		break;

	case 34:
		if (MiniStraight(10, 0.6))
			m_autostage++;
		break;

	case 35:
		m_intake->AutoEject();
			m_autostage++;
		break;

	case 36:
		m_drivetrain->Drive(0, 0);					// turn off drive motors
		break;
	}
}


void Autonomous::AutoCenterSwitchRight()
{
	/*
	switch (m_autostage)
	{
	case 0:
		if (DriveStraight(40, 0.5, 0.75, 30.0))		// targetdistance = 40", ramp = .5s, power = 50%, deceldistance = 24"
			m_autostage++;
		break;
	case 1:
		if (AngleStraight(-60, 52, 0.5, 0.75, 35.0))		// targetdistance = 52", ramp = .5s, power = 50%, deceldistance = 24"
			m_autostage++;
		break;
	case 2:
		if (AngleStraight(60, 32, 0.1, 0.3, 18.0))		// targetdistance = 32", ramp = .1s, power = 25%, deceldistance = 18"
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
		if (MiniStraight(20, -0.6))
			m_autostage++;
		break;

	case 6:
		m_lifter->MoveBottom();
		if (AngleStraight(-45, 50, 0.5, -0.75, 30))
			m_autostage++;
		break;

	case 7:
		if (TurnAngle(-45))
			m_autostage++;
		break;

	case 8:
		if (MiniStraight(6, -0.6))
			m_autostage++;
		break;

	case 9:
		if (TurnAngle(90))
			m_autostage++;
		break;

	case 10:
		if (MiniStraight(24, 0.6))
			m_autostage++;
		break;

	case 11:
		m_intake->AutoIngest();
			m_timer.Reset();
			m_timer.Start();
			m_autostage++;
		break;

	case 12:
		if(m_timer.HasPeriodPassed(0.25))
		{
			m_intake->FinishAutoIngest();
			m_autostage++;
		}
		else
			m_drivetrain->Drive(0, 0);
		break;

	case 13:
		m_lifter->AutoAuto();
		if (MiniStraight(12, -0.6))
			m_autostage++;
		break;

	case 14:
		m_lifter->AutoAuto();
		if (TurnAngle(-90))
			m_autostage++;
		break;

	case 15:
		m_lifter->AutoAuto();
		if (MiniStraight(6, 0.6))
			m_autostage++;
		break;

	case 16:
		m_lifter->AutoAuto();
		if (AngleStraight(45, 50, 0.5, 0.75, 30))
			m_autostage++;
		break;

	case 17:
		if (TurnAngle(45))
			m_autostage++;
		break;

	case 18:
		if (MiniStraight(10, 0.6))
			m_autostage++;
		break;

	case 19:
		m_intake->AutoEject();
		m_timer.Reset();
		m_autostage++;
		break;

	case 20:
		if (m_timer.Get() > 0.25)
			m_autostage++;
		break;
	case 21:
		if (MiniStraight(11, -0.6))
			m_autostage++;
		break;

	case 22:
		m_lifter->MoveBottom();
		if (AngleStraight(-45, 50, 0.5, -0.75, 30))
			m_autostage++;
		break;

	case 23:
		if (TurnAngle(-45))
			m_autostage++;
		break;

	case 24:
		if (MiniStraight(6, -0.6))
			m_autostage++;
		break;

	case 25:
		if (TurnAngle(75))
			m_autostage++;
		break;

	case 26:
		if (MiniStraight(24, 0.6))
			m_autostage++;
		break;

	case 27:
		m_intake->AutoIngest();
		m_timer.Reset();
		m_timer.Start();
		m_autostage++;
		break;

	case 28:
		if (m_timer.HasPeriodPassed(0.25))
		{
			m_intake->FinishAutoIngest();
			m_autostage++;
		}
		else
			m_drivetrain->Drive(0, 0);
		break;

	case 29:
		m_lifter->AutoAuto();
		if (MiniStraight(12, -0.6))
				m_autostage++;
		break;

	case 30:
		m_lifter->AutoAuto();
		if (TurnAngle(-75))
			m_autostage++;
		break;

	case 31:
		m_lifter->AutoAuto();
		if (MiniStraight(6, 0.6))
			m_autostage++;
		break;

	case 32:
		m_lifter->AutoAuto();
		if (AngleStraight(45, 50, 0.5, 0.75, 30))
			m_autostage++;
		break;

	case 33:
		if (TurnAngle(45))
			m_autostage++;
		break;

	case 34:
		if (MiniStraight(10, 0.6))
			m_autostage++;
		break;

	case 35:
		m_intake->AutoEject();
			m_autostage++;
		break;

	case 36:
		m_drivetrain->Drive(0, 0);					// turn off drive motors
		break;
	}
	*/
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
	switch(m_autostage)
	{
	case 0:
		if (DriveStraight(10, 0.1, 0.3, 6))
			m_autostage++;
		break;

	case 1:
		if (TurnAngle(-20))
			m_autostage++;
		break;

	case 2:
		m_lifter->AutoDeploy();
		if (DriveStraight(90, 0.5, 0.8, 65))		// original: 92
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
		if (DriveStraight(39.5, 0.5, -0.6, 30))		/// original: 44.5
			m_autostage++;
		break;

	case 6:
		m_lifter->MoveBottom();
		if (TurnAngle(50))
			m_autostage++;
		break;

	case 7:
		m_lifter->MoveBottom();
		m_intake->AutoIngest();
		if (MiniStraight(20, 0.6))
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
		m_lifter->AutoAuto();
		if (MiniStraight(20, -0.6))
			m_autostage++;
		break;

	case 11:
		m_lifter->AutoAuto();
		if (TurnAngle(-40))
			m_autostage++;
		break;

	case 12:
		m_lifter->AutoAuto();
		if (DriveStraight(46.5, 0.5, 0.6, 30))		/// added 7 inches to compensate for turning
			m_autostage++;
		break;

	case 13:
		m_intake->AutoEject();
		m_timer.Reset();
		m_autostage++;
		break;

	case 14:
		if (m_timer.Get() > 0.25)
			m_autostage++;
		break;

	case 15:
		m_lifter->MoveBottom();
		if (DriveStraight(37.5, 0.25, -0.6, 30))
			m_autostage++;
		break;

	case 16:
		m_lifter->MoveBottom();
		if (TurnAngle(37.5))
			m_autostage++;
		break;

	case 17:
		m_lifter->MoveBottom();
		m_intake->AutoIngest();
		if (MiniStraight(20, 0.6))
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
		m_lifter->AutoAuto();
		if (MiniStraight(20, -0.6))
			m_autostage++;
		break;

	case 21:
		m_lifter->AutoAuto();
		if (TurnAngle(-37.5))
			m_autostage++;
		break;

	case 22:
		m_lifter->AutoAuto();
		if (DriveStraight(44.5, 0.25, 0.6, 30))		/// added 7 inches to compensate for turning
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
		if (TurnAngle(125))							// angle = 105 (counter clockwise)
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
		m_timer.Reset();
		break;
	case 8:
		if (m_timer.Get() > 0.25)
			m_autostage++;
		break;
	case 9:
		m_lifter->MoveBottom();
		if (MiniStraight(24,-0.3))
			m_autostage++;
		break;
	case 10:
		if (m_lifter->MoveBottom())
			m_autostage++;
		break;
	case 11:
		m_drivetrain->Drive(0, 0);					// turn off drive motors
		break;
	}
}


void Autonomous::AutoLeftScaleLeft()
{
	switch (m_autostage)
	{
	case 0:
		if (DriveStraight(230, 1.0, 0.5, 60.0))		// targetdistance = 290", everything else needs tuning
			m_autostage++;
		break;
	case 1:
		if (TurnAngle(-35))							// angle = -45 (clockwise)
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
