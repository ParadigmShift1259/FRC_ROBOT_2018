/*
 * DrivePID.cpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Developer
 */


#include "DrivePID.h"
#include "const.h"


DrivePID::DrivePID(DriveTrain *drivetrain, PigeonIMU *pigeon, OperatorInputs *inputs): PIDSubsystem(0.0, 0.0, 0.0)
{
	m_drivetrain = drivetrain;
	m_inputs = inputs;
	m_p = 0.0;
	m_i = 0.0;
	m_d = 0.0;
	m_y = 0.0;
	m_ramp = false;
	m_angle = 0.0;
	m_pigeon = pigeon;
}


DrivePID::~DrivePID()
{
}


void DrivePID::Init(double p, double i, double d, bool enable)
{
	m_p = p;
	m_i = i;
	m_d = d;
	GetPIDController()->SetPID(m_p, m_i, m_d);
	SetSetpoint(0);
	SetAbsoluteTolerance(0.5);
	if (enable)
		EnablePID();
}


void DrivePID::Stop()
{
	DisablePID();
}


void DrivePID::Drive(double y, bool ramp)
{
	SetY(y);
	m_ramp = ramp;
}

bool DrivePID::GetEnabled()
{
	return GetPIDController()->IsEnabled();
}

void DrivePID::SetP(double p)
{
	m_p = p;
	GetPIDController()->SetPID(m_p, m_i, m_d);
}


void DrivePID::SetI(double i)
{
	m_i = i;
	GetPIDController()->SetPID(m_p, m_i, m_d);
}


void DrivePID::SetD(double d)
{
	m_d = d;
	GetPIDController()->SetPID(m_p, m_i, m_d);
}


void DrivePID::SetY(double y)
{
	m_y = (y >= -1.0) ? ((y <= 1.0) ? y : 1.0) : -1.0;
}


void DrivePID::SetRelativeAngle(double angle)
{
	m_angle += angle;
	SetSetpointRelative(m_angle);
}


void DrivePID::SetAbsoluteAngle(double angle)
{
	m_angle = angle;
	SetSetpointRelative(m_angle);
}


void DrivePID::EnablePID()
{
	GetPIDController()->SetPID(m_p, m_i, m_d);
	GetPIDController()->Reset();
	Enable();
}


void DrivePID::DisablePID()
{
	if (GetPIDController()->IsEnabled())
	{
		Disable();
	}
}


double DrivePID::ReturnPIDInput()
{
	double m_leftpos = m_drivetrain->LeftTalonLead()->GetSelectedSensorPosition(0);
	m_leftpos = m_leftpos / CODES_PER_REV;

	double m_rightpos = m_drivetrain->RightTalonLead()->GetSelectedSensorPosition(0);
	m_rightpos = m_rightpos / CODES_PER_REV;

	double retval = (360 / (2 * 3.1415926535)) * (m_leftpos + m_rightpos) * WHEEL_DIAMETER * 3.1415926535 / WHEEL_TRACK;

	SmartDashboard::PutNumber("ReturnPosition(Enc)", m_leftpos + m_rightpos);
	SmartDashboard::PutNumber("ReturnCurrentPosition(Enc)", retval);
	retval = m_pigeon->GetFusedHeading();
	SmartDashboard::PutNumber("ReturnCurrentPosition(Gyro)", retval);

	return retval;
}


void DrivePID::UsePIDOutput(double output)
{
	m_drivetrain->Drive(output, m_y, m_ramp);
}
