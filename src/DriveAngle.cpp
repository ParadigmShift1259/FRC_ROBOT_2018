/**
 *  DriveAngle.cpp
 *  Date:
 *  Last Edited By:
 */


#include "DriveAngle.h"
#include "Const.h"


/*!
 * Constructor
 * This will be initializing variables
 * Adds pointers to Drivetrain WPI and Operator Inputs
 */
DriveAngle::DriveAngle(DriveTrain *drivetrain, OperatorInputs *inputs)
{
	m_drivetrain = drivetrain;
	m_inputs = inputs; //!< Member variable
	m_driveAnglePID = new DriveAnglePID(drivetrain);
	m_angle = 0;
}


/*!
 * Destructor
 * Deletes all variables
 */
DriveAngle::~DriveAngle()
{
	delete m_driveAnglePID;

}


/*!
 * Checks if the PID is enabled
 *
 * Returns true if PID is enabled
 */
bool DriveAngle::IsEnabled()
{
	return m_driveAnglePID->IsEnabled();
}


/*!
 * Enables the PID
 *
 */
void DriveAngle::EnableAnglePID()
{
	m_driveAnglePID->CheckPIDValues();
	m_driveAnglePID->SetRelativeSetpoint(0);
	m_driveAnglePID->ChangeActive(true);
}


/*!
 * Disables the PID
 */
void DriveAngle::DisableAnglePID()
{
	m_driveAnglePID->ChangeActive(false);
}


/*!
 * Sets the angle relative to the robot (?)
 */
void DriveAngle::SetRelativeAngle(double angleTarget)
{
	m_driveAnglePID->CheckPIDValues();
	m_driveAnglePID->SetRelativeSetpoint(angleTarget);
}


/*!
 * Returns the angle
 */
double DriveAngle::GetAngle()
{
	return m_driveAnglePID->ReturnCurrentPosition();
}


/*!
 *
 */
bool DriveAngle::IsOnTarget()
{
	return m_driveAnglePID->OnTarget();
}


/*!
 * Initializes the PID
 * Sets the values for P, I, and D
 */
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


/*!
 * Passes in the current position to the set point
 */
void DriveAngle::SetToCurrentAngle()
{
	m_driveAnglePID->SetSetpoint(m_driveAnglePID->ReturnCurrentPosition());
}


/*!
 * Loops the PID
 */
void DriveAngle::RunNormalDrive()
{
	m_drivetrain->Loop();
}


/*!
 * Stops the PID
 */
void DriveAngle::Stop()
{
	DisableAnglePID();
	m_driveAnglePID->SetSetpoint(0);
	m_angle = 0;
}


/*!
 * Sets
 */
void DriveAngle::Drive(double y, bool ramp)
{
	m_driveAnglePID->SetY(y);
	m_driveAnglePID->SetRamp(ramp);
}


/*!
 * Sets the vision angle
 */
void DriveAngle::SetVisionAngle(double angle)
{
	m_driveAnglePID->CheckPIDValues();
	m_driveAnglePID->SetSetpointRelativeToError(angle);
}
