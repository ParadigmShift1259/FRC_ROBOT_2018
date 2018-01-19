/**
 *  DriveAnglePID2018.cpp
 *  Date:
 *  Last Edited By:
 */


#include <DriveAnglePID2018.h>
#include <math.h>


DriveAnglePID2018::DriveAnglePID2018(DriveTrainWPI *drive) : PIDSubsystem("DriveAngle2018", 0.075 , 0.008, 0.03)
{
	m_P = 0.075;
	m_I = 0.007;
	m_D = 0.03;
	m_DriveTrainWPI = drive;
	isInitialized = false;
	isActive = false;
	m_y = 0;
	m_ramp = false;
	SetAbsoluteTolerance(1);
	SmartDashboard::PutNumber("DP00_P", m_P);
	SmartDashboard::PutNumber("DP00_I", m_I);
	SmartDashboard::PutNumber("DP00_D", m_D);
	SetOutputRange(-0.7,0.7);
}


DriveAnglePID2018::~DriveAnglePID2018()
{
}

bool DriveAnglePID2018::IsEnabled()
{
	return GetPIDController()->IsEnabled();
}

double DriveAnglePID2018::ReturnPIDInput()
{
	return ReturnCurrentPosition();
}


void DriveAnglePID2018::ChangeActive(bool newState)
{
	if (newState)
	{
		Enable();
		isActive = true;
	}
	else
	{
		Disable();
		isActive = false;
		isInitialized = false;
	}
}


bool DriveAnglePID2018::IsDone()
{
	return GetPIDController()->OnTarget();
}


void DriveAnglePID2018::SetSetpointRelativeToError(double newSetPoint)
{
	if (!isInitialized)
	{
		GetPIDController()->Reset();
		Enable();
		//SetSetpoint(ReturnCurrentPosition());
		SetSetpointRelative(0);
		isInitialized = true;
	}
	else
	{
		if ((newSetPoint > 0) == (GetPIDController()->GetError() > 0))
			newSetPoint -= GetPIDController()->GetError();
		SetSetpointRelative(newSetPoint);
	}
}

void DriveAnglePID2018::SetRelativeSetpoint(double newSetPoint) {
	if (!isInitialized)
	{
		GetPIDController()->Reset();
		Enable();
		//SetSetpoint(ReturnCurrentPosition());
		SetSetpointRelative(0);
		isInitialized = true;
	}
	else
		SetSetpointRelative(newSetPoint);
}

void DriveAnglePID2018::UsePIDOutput(double output)
{
	SmartDashboard::PutNumber("DP01_output", output);
	SmartDashboard::PutBoolean("DP01_IS_ACTIVE",isActive);
	if (isActive)
	{
		//output = std::abs(output) > 0.25 ? output : 0.25 * output / std::abs(output);
		m_DriveTrainWPI->Drive(output, m_y, true);
	}
}


void DriveAnglePID2018::CheckPIDValues()
{
	 m_P = SmartDashboard::GetNumber("DP00_P", m_P);
	 m_I = SmartDashboard::GetNumber("DP00_I", m_I);
	 m_D = SmartDashboard::GetNumber("DP00_D", m_D);

	GetPIDController()->SetPID(m_P, m_I, m_D);
/*
	if (GetPIDController()->GetP() != SmartDashboard::GetValue("DB/Slider 0")->GetDouble() ||
		GetPIDController()->GetI() != SmartDashboard::GetValue("DB/Slider 1")->GetDouble() ||
		GetPIDController()->GetD() != SmartDashboard::GetValue("DB/Slider 2")->GetDouble())
		GetPIDController()->SetPID(SmartDashboard::GetValue("DB/Slider 0")->GetDouble(),
								   SmartDashboard::GetValue("DB/Slider 1")->GetDouble(),
								   SmartDashboard::GetValue("DB/Slider 2")->GetDouble());
*/
}


double DriveAnglePID2018::ReturnCurrentPosition()
{
	double retval = ((360/(2*3.1415926535))*((m_DriveTrainWPI->LeftTalon()->GetQuadrature()+m_DriveTrainWPI->RightTalon()->GetQuadrature())*WHEEL_CIRCUMFERENCE*3.1415926535)/(WHEEL_BASE));
	SmartDashboard::PutNumber("DP02_ontarget", IsDone());
	SmartDashboard::PutNumber("DP03_angle", retval);
	return retval;
}


void DriveAnglePID2018::SetY(double y)
{
	m_y = (y >= -1.0) ? ((y <= 1.0) ? y : 1.0) : -1.0;
}


double DriveAnglePID2018::GetY()
{
	return m_y;
}


void DriveAnglePID2018::SetRamp(bool ramp)
{
	m_ramp = ramp;
}


bool DriveAnglePID2018::GetRamp()
{
	return m_ramp;
}
