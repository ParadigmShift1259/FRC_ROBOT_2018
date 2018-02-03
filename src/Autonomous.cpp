#include <Autonomous.h>
#include <cmath>

Autonomous::Autonomous(OperatorInputs *inputs, DriveTrain *drivetrain)
{
	m_inputs = inputs;
	m_drivetrain = drivetrain;
	m_drivepid = new DrivePID(m_drivetrain, m_inputs);
	m_timerstraight = new Timer();
	m_timerstraight->Reset();

	m_straightstate = kStart;
	m_acceldistance = 0;
	m_timermod = ACCEL_TIME;
}


Autonomous::~Autonomous()
{

}


void Autonomous::Init()
{
	DriverStation::ReportError("AutonomousInit");

	if (SmartDashboard::GetNumber("AutoDistance", 0) == 0)
		SmartDashboard::PutNumber("AutoDistance", 30000);
	m_straightstate = kStart;
	m_acceldistance = 0;
	m_timermod = ACCEL_TIME;
	m_timerstraight->Start();
	m_timerstraight->Reset();
	m_drivepid->Init(0.001, 0.0005, 0.0, true);
	m_drivepid->SetRelativeAngle(90);
}

void Autonomous::Loop()
{
	DriveStraight(SmartDashboard::GetNumber("AutoDistance",0));
}

/*!
 * Drives straight the specified number of ticks and returns true when the function is done.
 * not guarenteed to be perfectly accurate but is pretty close. Will not try to target after
 * done with the state machine and will hold the incorrect value.
 */
bool Autonomous::DriveStraight(double targetdistance)
{
	SmartDashboard::PutNumber("LeftEncoder",m_drivetrain->LeftTalonPosition());
	SmartDashboard::PutNumber("RightEncoder",m_drivetrain->RightTalonPosition());
	double timervalue = m_timerstraight->Get(); //!<Stores the current timer value
	double greatestdistance = abs((abs(m_drivetrain->RightTalonPosition()) > abs(m_drivetrain->LeftTalonPosition())) ?
			m_drivetrain->RightTalonPosition() : m_drivetrain->LeftTalonPosition()); //!< Stores the absolute value of the greatest encoder distance

	switch (m_straightstate)
	{
	/*
	 * Accelerates during this case for a duration specified by ACCEL_TIME, feeds into kMaintain unless
	 * 1/3rd the target distance is reached in which case kDecel is moved into
	 */
	case kStart:
		m_timerstraight->Start();
		m_drivetrain->LeftTalonLead()->SetSelectedSensorPosition(0, 0, 0);
		m_drivetrain->RightTalonLead()->SetSelectedSensorPosition(0, 0, 0);
		m_drivetrain->LeftTalonLead()->SetNeutralMode(NeutralMode::Brake);
		m_drivetrain->RightTalonLead()->SetNeutralMode(NeutralMode::Brake);
		m_straightstate = kAccel;
		m_timermod = ACCEL_TIME; // @suppress("No break at end of case")
	case kAccel:
		//If acceleration has reached max time
		if((((int)(timervalue*50))/50.0) > ACCEL_TIME)
		{
			m_acceldistance = greatestdistance;
			m_straightstate = kMaintain;
			m_timerstraight->Reset();
			m_timerstraight->Start();
			SmartDashboard::PutNumber("StopAccel", greatestdistance);
		}
		else
		{
			//Handles the case where a triangle acceleration is required
			if(greatestdistance > targetdistance/3)
			{
				m_timermod = timervalue;
				m_timerstraight->Reset();
				m_timerstraight->Start();
				m_straightstate = kDecel;
			}
			else
			{
				m_drivetrain->Drive(0,-1*(((int)(timervalue*50))/50.0)/ACCEL_TIME,false);
				break;
			}
		} // @suppress("No break at end of case")
		/*
		 * Maintaines top speed until 2x the acceldistance away from targetdistance
		 */
	case kMaintain:
		if (targetdistance-greatestdistance <= m_acceldistance*2)
		{
			m_straightstate = kDecel;
			m_timerstraight->Reset();
			m_timerstraight->Start();
			timervalue = 0;
			SmartDashboard::PutNumber("StartDecel", greatestdistance);
		}
		else
		{
			m_drivetrain->Drive(0,-1,false);
			break;
		} // @suppress("No break at end of case")
		/*
		 * Decelerates over the course of ACCEL_TIME which happens to be about
		 * 2x the acceleration distance
		 */
	case kDecel:
		if((((int)(timervalue*50))/50.0) > m_timermod)
		{
			m_drivetrain->Drive(0,0,false);
			return true;
		}
		else
		{
			m_drivetrain->Drive(0,-1*(m_timermod-(((int)(timervalue*50))/50.0))/ACCEL_TIME,false);
		}
	}
	return false;
}

void Autonomous::Stop()
{
	m_drivepid->Stop();
}
