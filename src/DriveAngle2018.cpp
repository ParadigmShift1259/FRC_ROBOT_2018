/**
 *  DriveAngle2018.cpp
 *  Date:
 *  Last Edited By:
 */


#include <DriveAngle2018.h>


DriveAngle2018::DriveAngle2018(DriveTrainWPI *DriveTrainWPI, OperatorInputs *inputs)
{
	m_DriveTrainWPI = DriveTrainWPI;
	m_inputs = inputs;
	m_DriveAnglePID2018 = new DriveAngle2018PID(DriveTrainWPI);
	m_angle = 0;
}


DriveAngle2018::~DriveAngle2018()
{
	delete m_DriveAnglePID2018;

}

bool DriveAngle2018::IsHighGear() {
	return m_DriveTrainWPI->m_ishighgear;
}

void DriveAngle2018::Shift() {
	m_DriveTrainWPI->Shift();
}

bool DriveAngle2018::IsEnabled()
{
	return m_DriveAnglePID2018->IsEnabled();
}

void DriveAngle2018::EnableAnglePID()
{
	m_DriveAnglePID2018->CheckPIDValues();
	m_DriveAnglePID2018->SetRelativeSetpoint(0);
	m_DriveAnglePID2018->ChangeActive(true);
}


void DriveAngle2018::DisableAnglePID()
{
	m_DriveAnglePID2018->ChangeActive(false);
}


void DriveAngle2018::SetRelativeAngle(double angleTarget)
{
	m_DriveAnglePID2018->CheckPIDValues();
	m_DriveAnglePID2018->SetRelativeSetpoint(angleTarget);
}


double DriveAngle2018::GetAngle()
{
	return m_DriveAnglePID2018->ReturnCurrentPosition();
}


bool DriveAngle2018::IsOnTarget()
{
	return m_DriveAnglePID2018->OnTarget();
}


void DriveAngle2018::Init(bool enable)
{
	m_DriveAnglePID2018->SetP(0.425);
	m_DriveAnglePID2018->SetI(0.0062);
	m_DriveAnglePID2018->SetD(0.03);
	SmartDashboard::PutNumber("DB/Slider 3", m_angle);
	m_DriveAnglePID2018->SetSetpoint(0);
	if (enable)
		EnableAnglePID();

}

void DriveAngle2018::SetToCurrentAngle() {
	m_DriveAnglePID2018->SetSetpoint(m_DriveAnglePID2018->ReturnCurrentPosition());
}

void DriveAngle2018::Loop()
{
	static unsigned int loopcnt = 0;
	static unsigned int shiftcnt = 0;
	double y;

	double newangle = SmartDashboard::GetNumber("DB/Slider 3", 0);
	if (m_angle != newangle)
	{
		SetRelativeAngle(newangle);
		m_angle = newangle;
	}
	SmartDashboard::PutNumber("DTAngle", GetAngle());

	m_DriveAnglePID2018->CheckPIDValues();

	if (m_inputs->xBoxLeftTrigger())
	{
		m_DriveTrainWPI->m_shift = true;
		m_DriveTrainWPI->m_lowspeedmode = false;
	}

	m_DriveTrainWPI->LowSpeedDriving();

	y = m_inputs->xBoxLeftY();

	if (m_DriveTrainWPI->m_isdownshifting)
		y = 0;

	if (m_DriveTrainWPI->m_lowspeedmode)
	{
		//x = x * LOWSPEED_MODIFIER_X;
		y = y * LOWSPEED_MODIFIER_Y;
	}

	Drive(y, true);

	if (m_DriveTrainWPI->m_shift)
	{
		shiftcnt += 4;
		if (m_DriveTrainWPI->m_ishighgear)
		{
			m_DriveTrainWPI->m_isdownshifting = true;
			shiftcnt += 2;
		}
		else
		{
			m_DriveTrainWPI->Shift();
			m_DriveTrainWPI->m_isdownshifting = false;
			shiftcnt += 1;
		}
	}

	if (m_DriveTrainWPI->m_isdownshifting && (abs(m_DriveTrainWPI->m_previousx * X_SCALING) < ENCODER_TOP_SPEED) && (abs(m_DriveTrainWPI->m_previousy * Y_SCALING) < ENCODER_TOP_SPEED))
	{
		loopcnt++;
		m_DriveTrainWPI->Shift();
		m_DriveTrainWPI->m_isdownshifting = false;
	}

	SmartDashboard::PutNumber("DT02_y", y);
	SmartDashboard::PutNumber("DT03_top", ENCODER_TOP_SPEED);
	SmartDashboard::PutNumber("DT04_loop_count", loopcnt);
	SmartDashboard::PutNumber("DT05_shift", m_DriveTrainWPI->m_shift);
	SmartDashboard::PutNumber("DT06_shift_count", shiftcnt);
	SmartDashboard::PutNumber("DT07_shift_down", m_DriveTrainWPI->m_isdownshifting);
	SmartDashboard::PutNumber("DT08_abs_x", (abs(m_DriveTrainWPI->m_previousx * X_SCALING) < ENCODER_TOP_SPEED));
	SmartDashboard::PutNumber("DT09_abs_y", (abs(m_DriveTrainWPI->m_previousy * Y_SCALING) < ENCODER_TOP_SPEED));
}

void DriveAngle2018::RunNormalDrive() {
	m_DriveTrainWPI->Loop();
}

void DriveAngle2018::Stop()
{
	DisableAnglePID();
	m_DriveAnglePID2018->SetSetpoint(0);
	m_angle = 0;
}


void DriveAngle2018::Drive(double y, bool ramp)
{
	m_DriveAnglePID2018->SetY(y);
	m_DriveAnglePID2018->SetRamp(ramp);
}

void DriveAngle2018::SetVisionAngle(double angle)
{
	m_DriveAnglePID2018->CheckPIDValues();
	m_DriveAnglePID2018->SetSetpointRelativeToError(angle);
}