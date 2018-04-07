/*
 * DrivePID.cpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Developer
 */


#include "DrivePID.h"
#include "const.h"


DrivePID::DrivePID(DriveTrain *drivetrain, OperatorInputs *inputs): PIDSubsystem(0.0, 0.0, 0.0)
{
	m_drivetrain = drivetrain;
	m_inputs = inputs;
	m_p = 0.0;
	m_i = 0.0;
	m_d = 0.0;
	m_y = 0.0;
	m_ramp = false;
	m_pigeon = new PigeonIMU(0);
	m_feedback = kDisabled;
	m_gyroval[0] = 0;
	m_gyroval[1] = 0;
	m_gyroval[2] = 0;
	m_ontarget = 0;
}


DrivePID::~DrivePID()
{
}


void DrivePID::Init(double p, double i, double d, Feedback feedback, bool reset)
{
	m_p = p;
	m_i = i;
	m_d = d;
	m_feedback = feedback;
	GetPIDController()->SetPID(m_p, m_i, m_d);
	SetAbsoluteTolerance(2);
	SetOutputRange(-0.7,0.7);
	if (reset)
	{
		SetSetpoint(0);
		m_pigeon->SetFusedHeading(0,0);
	}
	if (feedback != kDisabled)
		EnablePID();
	m_ontarget = 0;
}


void DrivePID::Loop()
{
	m_pigeon->GetAccumGyro(m_gyroval);
	SmartDashboard::PutNumber("Gyrox", m_gyroval[0]);
	SmartDashboard::PutNumber("Gyroy", m_gyroval[1]);
	SmartDashboard::PutNumber("Gyroz", m_gyroval[2]);
	SmartDashboard::PutNumber("GyroFused",m_pigeon->GetFusedHeading());
}


void DrivePID::Stop()
{
	DisablePID();
	SetAbsoluteAngle(0);
	m_gyroval[0] = 0;
	m_gyroval[1] = 0;
	m_gyroval[2] = 0;
	m_pigeon->SetFusedHeading(0,0);
	m_pigeon->SetYaw(0,0);
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
	SetSetpointRelative(angle);
}


void DrivePID::SetAbsoluteAngle(double angle)
{
	SetSetpoint(angle);
}


void DrivePID::ResetGyro()
{
	m_pigeon->SetFusedHeading(0, 0);
}


bool DrivePID::IsOnTarget(double count)
{
	if (OnTarget())
	{
		if (count == 0)
		{
			m_ontarget = 0;
			return true;
		}
		else
		{
			m_ontarget++;
			if (m_ontarget > count)
			{
				m_ontarget = 0;
				return true;
			}
			return false;
		}
	}
	m_ontarget = 0;
	return false;
}


void DrivePID::EnablePID()
{
	GetPIDController()->SetPID(m_p, m_i, m_d);
	GetPIDController()->Reset();
	m_ontarget = 0;
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
	if (m_feedback == kEncoder)
	{
		double m_leftpos = m_drivetrain->LeftTalonLead()->GetSelectedSensorPosition(0);
		m_leftpos = m_leftpos / CODES_PER_REV;

		double m_rightpos = m_drivetrain->RightTalonLead()->GetSelectedSensorPosition(0);
		m_rightpos = m_rightpos / CODES_PER_REV;

		double retval = (360 / (2 * 3.1415926535)) * (m_leftpos + m_rightpos) * WHEEL_DIAMETER * 3.1415926535 / WHEEL_TRACK;

		SmartDashboard::PutNumber("ReturnPosition(Enc)", m_leftpos + m_rightpos);
		SmartDashboard::PutNumber("ReturnCurrentPosition(Enc)", retval);

		return retval;
	}
	else
	if (m_feedback == kGyro)
	{
		double retval = m_pigeon->GetFusedHeading();

		SmartDashboard::PutNumber("ReturnPIDInput(Gyro)", retval);

		return retval;
	}
	return 0;
}


void DrivePID::UsePIDOutput(double output)
{
	m_drivetrain->Drive(output, m_y, m_ramp);
}
