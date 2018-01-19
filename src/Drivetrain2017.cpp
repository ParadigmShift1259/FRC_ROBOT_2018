/**
 *  Drivetrain.cpp
 *  Date:
 *  Last Edited By:
 */


#include "Const.h"
#include "smartdashboard/smartdashboard.h"
#include <Timer.h>
#include <Talon.h>
#include <Drivetrain2017.h>
#include <Encoder.h>
#include <cmath>
#include <String>


using namespace std;


OldDriveTrain::OldDriveTrain(OperatorInputs *inputs, DriverStation *ds)
{
	m_inputs = inputs;
	m_driverstation = ds;

	m_lefttalonlead = new WPI_TalonSRX(CAN_LEFT_PORT);
	m_lefttalonfollow = new WPI_TalonSRX(CAN_SECOND_LEFT_PORT);
	m_righttalonlead = new WPI_TalonSRX(CAN_RIGHT_PORT);
	m_righttalonfollow = new WPI_TalonSRX(CAN_SECOND_RIGHT_PORT);

	m_lefttalonlead->Set(ControlMode::PercentOutput, 0);
	m_lefttalonfollow->Set(ControlMode::Follower, CAN_LEFT_PORT);

	m_righttalonlead->Set(ControlMode::PercentOutput, 0);
	m_righttalonfollow->Set(ControlMode::Follower, CAN_RIGHT_PORT);

	m_lefttalonlead->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
	//m_lefttalonlead->ConfigEncoderCodesPerRev(CODES_PER_REV);
	m_lefttalonlead->SetSensorPhase(false);
	m_lefttalonlead->SetSelectedSensorPosition(0, 0, 0);
	m_lefttalonlead->SetNeutralMode(NeutralMode::Brake);

	m_righttalonlead->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
	//m_righttalonlead->ConfigEncoderCodesPerRev(CODES_PER_REV);
	m_righttalonlead->SetSensorPhase(false);
	m_righttalonlead->SetSelectedSensorPosition(0, 0, 0);
	m_righttalonlead->SetNeutralMode(NeutralMode::Brake);

	m_lefttalonfollow->SetNeutralMode(NeutralMode::Brake);

	m_righttalonfollow->SetNeutralMode(NeutralMode::Brake);

	m_leftspeed = 0;
	m_rightspeed = 0;
	m_leftposition = 0;
	m_rightposition = 0;

	m_shifter = new Solenoid(PCM_SHIFT_MODULE, PCM_SHIFT_PORT_LOW);
	// Robot starts in low gear
	m_ishighgear = false;
	// Starts in low gear
	m_shifter->Set(FLIP_HIGH_GEAR ^ m_ishighgear);
	m_isdownshifting = false;
	m_lowspeedmode = false;
	m_shift = false;

	m_leftpow = 0;
	m_rightpow = 0;
	m_previousx = 0;
	m_previousy = 0;
	m_coasting = 1;
	m_invertleft = INVERT_LEFT;
	m_invertright = INVERT_RIGHT;
	m_direction = DT_DEFAULT_DIRECTION;

	m_timerramp = new Timer();
	m_rampmax = RAMPING_RATE_MAX;
}


OldDriveTrain::~OldDriveTrain()
{
	delete m_lefttalonlead;
	delete m_lefttalonfollow;
	delete m_righttalonlead;
	delete m_righttalonfollow;
	delete m_shifter;
	delete m_timerramp;
}


void OldDriveTrain::Init()
{
	m_lefttalonlead->Set(ControlMode::PercentOutput, 0);
	m_lefttalonfollow->Set(ControlMode::Follower, CAN_LEFT_PORT);

	m_righttalonlead->Set(ControlMode::PercentOutput, 0);
	m_righttalonfollow->Set(ControlMode::Follower, CAN_RIGHT_PORT);

	m_lefttalonlead->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
	//m_lefttalonlead->ConfigEncoderCodesPerRev(CODES_PER_REV);
	m_lefttalonlead->SetSensorPhase(false);
	m_lefttalonlead->SetSelectedSensorPosition(0, 0, 0);
	m_lefttalonlead->SetNeutralMode(NeutralMode::Brake);

	m_righttalonlead->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
	//m_righttalonlead->ConfigEncoderCodesPerRev(CODES_PER_REV);
	m_righttalonlead->SetSensorPhase(false);
	m_righttalonlead->SetSelectedSensorPosition(0, 0, 0);
	m_righttalonlead->SetNeutralMode(NeutralMode::Brake);

	m_lefttalonfollow->SetNeutralMode(NeutralMode::Brake);

	m_righttalonfollow->SetNeutralMode(NeutralMode::Brake);

	m_leftpow = 0;
	m_rightpow = 0;
	m_leftspeed = 0;
	m_rightspeed = 0;
	m_leftposition = 0;
	m_rightposition = 0;
	m_previousx = 0;
	m_previousy = 0;
	m_coasting = 1;
	m_lefttalonlead->Set(0);
	//m_lefttalonfollow->Set(0);
	m_righttalonlead->Set(0);
	//m_righttalonfollow->Set(0);
	m_timerramp->Reset();
	m_timerramp->Start();
	m_ishighgear = true;
	// Starts in high gear
	m_shifter->Set(FLIP_HIGH_GEAR ^ m_ishighgear);
	m_isdownshifting = false;
	m_lowspeedmode = false;
	m_shift = false;
	m_direction = DT_DEFAULT_DIRECTION;
	SmartDashboard::PutString("DT10_Direction", "Gear Forward");
}


void OldDriveTrain::Loop()
{
	static unsigned int loopcnt = 0;
	static unsigned int shiftcnt = 0;
	double x;
	double y;

	if (m_inputs->xBoxR3())
	{
		ChangeDirection();
	}
	if (m_inputs->xBoxLeftTrigger())
	{
		m_shift = true;
		m_lowspeedmode = false;
	}

	LowSpeedDriving();

	x = m_inputs->xBoxLeftX();
	y = m_inputs->xBoxLeftY();

	if (m_isdownshifting)
		y = 0;

	if (m_lowspeedmode)
	{
		x = x * LOWSPEED_MODIFIER_X;
		y = y * LOWSPEED_MODIFIER_Y;
	}

	Drive(x, y, true);

	if (m_shift)
	{
		shiftcnt += 4;
		if (m_ishighgear)
		{
			m_isdownshifting = true;
			shiftcnt += 2;
		}
		else
		{
			Shift();
			m_isdownshifting = false;
			shiftcnt += 1;
		}
	}

	if (m_isdownshifting && (abs(m_previousx * X_SCALING) < ENCODER_TOP_SPEED) && (abs(m_previousy * Y_SCALING) < ENCODER_TOP_SPEED))
	{
		loopcnt++;
		Shift();
		m_isdownshifting = false;
	}


	SmartDashboard::PutNumber("DIRECTION", m_direction);
	SmartDashboard::PutNumber("DT01_x", x);
	SmartDashboard::PutNumber("DT02_y", y);
	SmartDashboard::PutNumber("DT03_top", ENCODER_TOP_SPEED);
	SmartDashboard::PutNumber("DT04_loop_count", loopcnt);
	SmartDashboard::PutNumber("DT05_shift", m_shift);
	SmartDashboard::PutNumber("DT06_shift_count", shiftcnt);
	SmartDashboard::PutNumber("DT07_shift_down", m_isdownshifting);
	SmartDashboard::PutNumber("DT08_abs_x", (abs(m_previousx * X_SCALING) < ENCODER_TOP_SPEED));
	SmartDashboard::PutNumber("DT09_abs_y", (abs(m_previousy * Y_SCALING) < ENCODER_TOP_SPEED));
}


void OldDriveTrain::Stop()
{
	m_ishighgear = true;
	m_shifter->Set(FLIP_HIGH_GEAR ^ m_ishighgear);
	//Drive(0, 0, false);
}


void OldDriveTrain::Drive(double x, double y, bool ramp)
{
	double yd = y * m_direction;
	double maxpower;

	if (x == 0 || yd == 0)
	{
		maxpower = 1;
	}
	else
	{
		if (abs(x) > abs(yd))
			maxpower = (abs(yd) / abs(x)) + 1;
		else
			maxpower = (abs(x) / abs(yd)) + 1;
	}
	if (!ramp)
	{
		m_previousx = x;	//rampInput(previousX, joyStickX, BatteryRampingMin, BatteryRampingMax);
		m_previousy = yd;
		m_leftpow = m_previousy - m_previousx;
		m_rightpow = m_previousy + m_previousx;
	}
	else
	{
		double battery = m_driverstation->GetInstance().GetBatteryVoltage();
		double rampmin = RAMPING_RATE_MIN / battery;
		double rampmax = m_rampmax / battery;
		SmartDashboard::PutNumber("DT10_battery", battery);
		m_previousx = x;	//rampInput(previousX, joyStickX, rampmin, rampmax);
		m_previousy = Ramp(m_previousy, yd, rampmin, rampmax);
		m_leftpow = m_previousy * Y_SCALING - (m_previousx * X_SCALING);
		m_rightpow = m_previousy * Y_SCALING + (m_previousx * X_SCALING);
	}
	m_leftspeed = m_lefttalonlead->GetSelectedSensorVelocity(0);
	m_rightspeed = m_righttalonlead->GetSelectedSensorVelocity(0);
	m_leftposition = m_lefttalonlead->GetSelectedSensorPosition(0);
	m_rightposition = m_righttalonlead->GetSelectedSensorPosition(0);
	//m_leftposition = m_lefttalonlead->GetSensorCollection().GetQuadraturePosition() / CODES_PER_REV;
	//m_rightposition = m_righttalonlead->GetSensorCollection().GetQuadraturePosition() / CODES_PER_REV;


	//m_lefttalonlead->Set(0);
	//m_lefttalonfollow->Set(0);
	//m_righttalonlead->Set(0);
	//m_righttalonfollow->Set(0);

	m_lefttalonlead->Set(m_invertleft * m_coasting * LeftMotor(maxpower));
	m_righttalonlead->Set(m_invertright * m_coasting * RightMotor(maxpower));

	SmartDashboard::PutNumber("DT11_turningramp", m_previousx); 			//Left Motors are forward=negative
	SmartDashboard::PutNumber("DT12_drivingramp", m_previousy); 			//Right Motors are forward=positive
	SmartDashboard::PutNumber("DT13_leftpow", m_invertleft*m_leftpow); 		//Left Motors are forward=negative
	SmartDashboard::PutNumber("DT14_rightpow", m_invertright*m_rightpow); 	//Right Motors are forward=positive
	SmartDashboard::PutNumber("DT15_gear", m_ishighgear);
	SmartDashboard::PutNumber("DT16_leftspeed", m_leftspeed);
	SmartDashboard::PutNumber("DT17_rightspeed", m_rightspeed);
	SmartDashboard::PutNumber("DT18_leftposition", m_leftposition);
	SmartDashboard::PutNumber("DT19_rightposition", m_rightposition);
}


// sets the motors to coasting mode, shifts, and then sets them back to break mode
void OldDriveTrain::Shift()
{
	//m_lefttalonlead->SetNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Coast);
	//m_lefttalonfollow->SetNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Coast);
	//m_righttalonlead->SetNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Coast);
	//m_righttalonfollow->SetNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Coast);
	m_ishighgear = !m_ishighgear;
	m_shifter->Set(FLIP_HIGH_GEAR ^ m_ishighgear);
	//m_lefttalonlead->SetNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
	//m_lefttalonfollow->SetNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
	//m_righttalonlead->SetNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
	//m_righttalonfollow->SetNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
	m_shift = false;
}


// change direction and return true if going forward
bool OldDriveTrain::ChangeDirection()
{
	m_direction *= -1.0;
	return (m_direction == DT_DEFAULT_DIRECTION);
}


void OldDriveTrain::LowSpeedDriving()
{
	SmartDashboard::PutNumber("DT16_lowspeed", m_lowspeedmode);
	if (m_inputs->button10())
	{
		m_lowspeedmode = !m_lowspeedmode;
		if (m_ishighgear && m_lowspeedmode)
		{
			m_shift = true;
		}
	}
}


// ramp the power
double OldDriveTrain::Ramp(double previous, double desired, double rampmin, double rampmax)
{
	double newpow = previous;

	bool timepassed = m_timerramp->HasPeriodPassed(RAMPING_RATE_PERIOD);

	if (timepassed)
	{
		double delta = abs(desired - previous);

		// Makes it so that robot can't go stop to full
		if (delta <= rampmin)
			newpow = desired;
		else
		if (previous < desired)
			newpow += max((delta*rampmax), rampmin);
		else
		if (previous > desired)
			newpow -= max((delta*rampmax), rampmin);
		//leftTalons1->Set(-previousLeftPow);
	}
	return newpow;
}


/*
void Drivetrain::rampRightPower(double desiredPow, double rampSpeedMin, double rampSpeedMax)
{
	//Makes it so that robot can't go stop to full
	if (abs(desiredPow - previousRightPow) < rampSpeedMin)
	{
		previousRightPow = desiredPow;
	}
	else
	if (previousRightPow < desiredPow)
	{
		previousRightPow += rampSpeedMin;
	}
	else
	if (previousRightPow > desiredPow)
	{
		previousRightPow -= rampSpeedMin;
	}
	rightTalons->Set(previousRightPow);
	//rightTalons1->Set(previousRightPow);
}
*/


double OldDriveTrain::LeftMotor(double &maxpower)
{
	//moved rightSpeed to class scope, it is being set in setPower()

	double leftpow = m_leftpow * LEFT_MOTOR_SCALING / maxpower;

	/*if (m_leftpow != 0 && m_rightpow != 0)
	{
		m_leftencodermax = abs(m_leftspeed / m_leftpow);
		if (min(abs(m_leftspeed), abs(m_rightspeed)) > ENCODER_TOP_SPEED)
			CheckEncoderTimer();
		if (m_isleftfaster)
			leftpow = m_ratiolr * leftpow;
	}*/
	return leftpow;
}


double OldDriveTrain::RightMotor(double &maxpower)
{
	//moved rightSpeed to class scope, it is being set in setPower()

	double rightpow = m_rightpow * RIGHT_MOTOR_SCALING / maxpower;

	/*if (m_leftpow != 0 && m_rightpow != 0)
	{
		m_rightencodermax = abs(m_rightspeed / m_rightpow);
		if (min(abs(m_leftspeed), abs(m_rightspeed)) > ENCODER_TOP_SPEED)
			CheckEncoderTimer();
		if (!m_isleftfaster)
			rightpow = m_ratiolr * rightpow;
	}*/
	return rightpow;
}
