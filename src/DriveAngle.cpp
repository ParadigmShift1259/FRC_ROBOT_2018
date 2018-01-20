/**
 *  DriveAngle.cpp
 *  Date:
 *  Last Edited By:
 */


#include <DriveAngle.h>


DriveAngle::DriveAngle(DriveTrainWPI *DriveTrainWPI, OperatorInputs *inputs)
{
	m_DriveTrainWPI = DriveTrainWPI;
	m_inputs = inputs;
	m_driveAnglePID = new DriveAnglePID(DriveTrainWPI);
	m_angle = 0;
}


DriveAngle::~DriveAngle()
{
	delete m_driveAnglePID;

}


bool DriveAngle::IsEnabled()
{
	return m_driveAnglePID->IsEnabled();
}


void DriveAngle::EnableAnglePID()
{
	m_driveAnglePID->CheckPIDValues();
	m_driveAnglePID->SetRelativeSetpoint(0);
	m_driveAnglePID->ChangeActive(true);
}


void DriveAngle::DisableAnglePID()
{
	m_driveAnglePID->ChangeActive(false);
}


void DriveAngle::SetRelativeAngle(double angleTarget)
{
	m_driveAnglePID->CheckPIDValues();
	m_driveAnglePID->SetRelativeSetpoint(angleTarget);
}


double DriveAngle::GetAngle()
{
	return m_driveAnglePID->ReturnCurrentPosition();
}


bool DriveAngle::IsOnTarget()
{
	return m_driveAnglePID->OnTarget();
}


void DriveAngle::Init(bool enable)
{
	m_driveAnglePID->SetP(0.425);
	m_driveAnglePID->SetI(0.0062);
	m_driveAnglePID->SetD(0.03);
	SmartDashboard::PutNumber("DB/Slider 3", m_angle);
	m_driveAnglePID->SetSetpoint(0);
	if (enable)
		EnableAnglePID();
}


void DriveAngle::SetToCurrentAngle()
{
	m_driveAnglePID->SetSetpoint(m_driveAnglePID->ReturnCurrentPosition());
}


void DriveAngle::RunNormalDrive()
{
	m_DriveTrainWPI->Loop();
}


void DriveAngle::Stop()
{
	DisableAnglePID();
	m_driveAnglePID->SetSetpoint(0);
	m_angle = 0;
}


void DriveAngle::Drive(double y, bool ramp)
{
	m_driveAnglePID->SetY(y);
	m_driveAnglePID->SetRamp(ramp);
}


void DriveAngle::SetVisionAngle(double angle)
{
	m_driveAnglePID->CheckPIDValues();
	m_driveAnglePID->SetSetpointRelativeToError(angle);
}
