/**
 *  DriveTrain.cpp
 *  Date:
 *  Last Edited By:
 */


#include <Drivetrain.h>
#include "Const.h"
#include <cmath>


using namespace std;


DriveTrain::DriveTrain(OperatorInputs *inputs, WPI_TalonSRX *left1, WPI_TalonSRX *left2, WPI_TalonSRX *left3, WPI_TalonSRX *right1, WPI_TalonSRX *right2, WPI_TalonSRX *right3)
{
	m_inputs = inputs;

	m_mode = kFollower;

	m_lefttalon1owner = false;
	m_lefttalon2owner = false;
	m_lefttalon3owner = false;
	m_righttalon1owner = false;
	m_righttalon2owner = false;
	m_righttalon3owner = false;

	m_lefttalon1 = left1;
	m_lefttalon2 = left2;
	m_lefttalon3 = left3;
	m_righttalon1 = right1;
	m_righttalon2 = right2;
	m_righttalon3 = right3;

	m_leftscgroup = nullptr;
	m_rightscgroup = nullptr;
	m_differentialdrive = nullptr;

	m_leftspeed = 0;
	m_rightspeed = 0;
	m_leftposition = 0;
	m_rightposition = 0;

	m_shifter = nullptr;
	if (PCM_SHIFT_PORT_LOW != -1)
	{
		m_shifter = new Solenoid(PCM_SHIFT_MODULE, PCM_SHIFT_PORT_LOW);
		m_shifter->Set(FLIP_HIGH_GEAR ^ m_ishighgear);
	}
	// Robot starts in low gear
	m_ishighgear = false;
	// Starts in low gear
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

	m_prevleftdistance = 0;
	m_prevrightdistance = 0;
}


DriveTrain::~DriveTrain()
{
	if ((m_lefttalon1 != nullptr) && m_lefttalon1owner)
		delete m_lefttalon1;
	if ((m_lefttalon2 != nullptr) && m_lefttalon2owner)
		delete m_lefttalon2;
	if ((m_lefttalon3 != nullptr) && m_lefttalon3owner)
		delete m_lefttalon3;
	if ((m_righttalon1 != nullptr) && m_righttalon1owner)
		delete m_righttalon1;
	if ((m_righttalon2 != nullptr) && m_righttalon2owner)
		delete m_righttalon2;
	if ((m_righttalon3 != nullptr) && m_righttalon3owner)
		delete m_righttalon3;
	if (m_shifter != nullptr)
		delete m_shifter;
	delete m_timerramp;
}


void DriveTrain::Init(DriveMode mode)
{
	m_mode = mode;

	if ((m_lefttalon1 == nullptr) && (CAN_LEFT_PORT_1 != -1))
	{
		m_lefttalon1 = new WPI_TalonSRX(CAN_LEFT_PORT_1);
		m_lefttalon1owner = true;
	}
	if ((m_lefttalon2 == nullptr) && (CAN_LEFT_PORT_2 != -1))
	{
		m_lefttalon2 = new WPI_TalonSRX(CAN_LEFT_PORT_2);
		m_lefttalon2owner = true;
	}
	if ((m_lefttalon3 == nullptr) && (CAN_LEFT_PORT_3 != -1))
	{
		m_lefttalon3 = new WPI_TalonSRX(CAN_LEFT_PORT_3);
		m_lefttalon3owner = true;
	}
	if ((m_righttalon1 == nullptr) && (CAN_RIGHT_PORT_1 != -1))
	{
		m_righttalon1 = new WPI_TalonSRX(CAN_RIGHT_PORT_1);
		m_righttalon1owner = true;
	}
	if ((m_righttalon2 == nullptr) && (CAN_RIGHT_PORT_2 != -1))
	{
		m_righttalon2 = new WPI_TalonSRX(CAN_RIGHT_PORT_2);
		m_righttalon2owner = true;
	}
	if ((m_righttalon3 == nullptr) && (CAN_RIGHT_PORT_3 != -1))
	{
		m_righttalon3 = new WPI_TalonSRX(CAN_RIGHT_PORT_3);
		m_righttalon3owner = true;
	}

	if ((m_lefttalon1 == nullptr) || (m_lefttalon2 == nullptr) || (m_righttalon1 == nullptr) || (m_righttalon2 == nullptr))
		m_mode = kNone;

	switch (m_mode)
	{
	case DriveMode::kFollower:
		m_lefttalon1->Set(ControlMode::PercentOutput, 0);
		m_lefttalon2->Set(ControlMode::Follower, CAN_LEFT_PORT_1);
		if (m_lefttalon3 != nullptr)
			m_lefttalon3->Set(ControlMode::Follower, CAN_LEFT_PORT_1);
		m_righttalon1->Set(ControlMode::PercentOutput, 0);
		m_righttalon2->Set(ControlMode::Follower, CAN_RIGHT_PORT_1);
		if (m_righttalon3 != nullptr)
			m_righttalon3->Set(ControlMode::Follower, CAN_RIGHT_PORT_1);
		break;

	case DriveMode::kTank:
	case DriveMode::kArcade:
	case DriveMode::kCurvature:
		if ((m_lefttalon3 == nullptr) || (m_righttalon3 == nullptr))
		{
			m_leftscgroup = new SpeedControllerGroup(*m_lefttalon1, *m_lefttalon2);
			m_rightscgroup = new SpeedControllerGroup(*m_righttalon1, *m_righttalon2);
			m_differentialdrive = new DifferentialDrive(*m_leftscgroup, *m_rightscgroup);
		}
		else
		{
			m_leftscgroup = new SpeedControllerGroup(*m_lefttalon1, *m_lefttalon2, *m_lefttalon3);
			m_rightscgroup = new SpeedControllerGroup(*m_righttalon1, *m_righttalon2, *m_righttalon3);
			m_differentialdrive = new DifferentialDrive(*m_leftscgroup, *m_rightscgroup);
		} 																						// @suppress("No break at end of case")

	case DriveMode::kDiscrete:
		m_lefttalon1->Set(ControlMode::PercentOutput, 0);
		m_lefttalon2->Set(ControlMode::PercentOutput, 0);
		if (m_lefttalon3 != nullptr)
			m_lefttalon3->Set(ControlMode::PercentOutput, 0);
		m_righttalon1->Set(ControlMode::PercentOutput, 0);
		m_righttalon2->Set(ControlMode::PercentOutput, 0);
		if (m_righttalon3 != nullptr)
			m_righttalon3->Set(ControlMode::PercentOutput, 0);
		break;

	case DriveMode::kNone:
		break;
	}

	if (m_lefttalon1 != nullptr)
	{
		m_lefttalon1->ConfigSelectedFeedbackSensor(ENC_LEFT_1, 0, 0);
		m_lefttalon1->SetSensorPhase(false);
		m_lefttalon1->SetNeutralMode(NeutralMode::Brake);
	}

	if (m_righttalon1 != nullptr)
	{
		m_righttalon1->ConfigSelectedFeedbackSensor(ENC_RIGHT_1, 0, 0);
		m_righttalon1->SetSensorPhase(false);
		m_righttalon1->SetNeutralMode(NeutralMode::Brake);
	}

	if (m_lefttalon2 != nullptr)
	{
		m_lefttalon2->ConfigSelectedFeedbackSensor(ENC_LEFT_2, 0, 0);
		m_lefttalon2->SetSensorPhase(false);
		m_lefttalon2->SetNeutralMode(NeutralMode::Brake);
	}

	if (m_righttalon2 != nullptr)
	{
		m_righttalon2->ConfigSelectedFeedbackSensor(ENC_RIGHT_2, 0, 0);
		m_righttalon2->SetSensorPhase(false);
		m_righttalon2->SetNeutralMode(NeutralMode::Brake);
	}

	if (m_lefttalon3 != nullptr)
	{
		m_lefttalon3->SetNeutralMode(NeutralMode::Brake);
	}

	if (m_righttalon3 != nullptr)
	{
		m_righttalon3->SetNeutralMode(NeutralMode::Brake);
	}

	m_leftpow = 0;
	m_rightpow = 0;
	m_leftspeed = 0;
	m_rightspeed = 0;
	m_leftposition = 0;
	m_rightposition = 0;
	m_previousx = 0;
	m_previousy = 0;
	m_coasting = 1;
	m_timerramp->Reset();
	m_timerramp->Start();
	m_ishighgear = true;
	// Starts in high gear
	if (m_shifter != nullptr)
		m_shifter->Set(FLIP_HIGH_GEAR ^ m_ishighgear);
	m_isdownshifting = false;
	m_lowspeedmode = false;
	m_shift = false;
	m_direction = DT_DEFAULT_DIRECTION;

	m_prevleftdistance = 0;
	m_prevrightdistance = 0;
}


void DriveTrain::Loop()
{
	static unsigned int loopcnt = 0;
	static unsigned int shiftcnt = 0;
	double x;
	double y;

	if (m_inputs->xBoxR3(OperatorInputs::ToggleChoice::kToggle, 0 * INP_DUAL))
	{
		ChangeDirection();
	}
	if (m_inputs->xBoxLeftTrigger(OperatorInputs::ToggleChoice::kToggle, 0 * INP_DUAL))
	{
		m_shift = true;
		m_lowspeedmode = false;
	}

	LowSpeedDriving();

	x = m_inputs->xBoxLeftX(0 * INP_DUAL);
	y = m_inputs->xBoxLeftY(0 * INP_DUAL);

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

	if (m_isdownshifting && (abs(m_previousx * X_SCALING) < CHILD_PROOF_SPEED) && (abs(m_previousy * Y_SCALING) < CHILD_PROOF_SPEED))
	{
		loopcnt++;
		Shift();
		m_isdownshifting = false;
	}

	SmartDashboard::PutNumber("DT00_direction", m_direction);
	SmartDashboard::PutNumber("DT01_x", x);
	SmartDashboard::PutNumber("DT02_y", y);
	SmartDashboard::PutNumber("DT03_top", CHILD_PROOF_SPEED);
	SmartDashboard::PutNumber("DT04_loop_count", loopcnt);
	SmartDashboard::PutNumber("DT05_shift", m_shift);
	SmartDashboard::PutNumber("DT06_shift_count", shiftcnt);
	SmartDashboard::PutNumber("DT07_shift_down", m_isdownshifting);
	SmartDashboard::PutNumber("DT08_abs_x", (abs(m_previousx * X_SCALING) < CHILD_PROOF_SPEED));
	SmartDashboard::PutNumber("DT09_abs_y", (abs(m_previousy * Y_SCALING) < CHILD_PROOF_SPEED));
}


void DriveTrain::Stop()
{
	m_ishighgear = true;
	if (m_shifter != nullptr)
		m_shifter->Set(FLIP_HIGH_GEAR ^ m_ishighgear);
	//Drive(0, 0, false);
}


void DriveTrain::Drive(double x, double y, bool ramp)
{
	double yd = y * m_direction;
	double maxpower;
	double templeft, tempright, tempforward, temprotate;
	bool tempspin;

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
		double battery = DriverStation::GetInstance().GetBatteryVoltage();
		double rampmin = RAMPING_RATE_MIN / battery;
		double rampmax = m_rampmax / battery;
		SmartDashboard::PutNumber("DT10_battery", battery);
		m_previousx = x;	//rampInput(previousX, joyStickX, rampmin, rampmax);
		m_previousy = Ramp(m_previousy, yd, rampmin, rampmax);
		m_leftpow = m_previousy * Y_SCALING - (m_previousx * X_SCALING);
		m_rightpow = m_previousy * Y_SCALING + (m_previousx * X_SCALING);
	}
	if (m_lefttalon1 != nullptr)
	{
		m_leftspeed = m_lefttalon1->GetSelectedSensorVelocity(0);
		m_leftposition = m_lefttalon1->GetSelectedSensorPosition(0);
	}
	if (m_righttalon1 != nullptr)
	{
		m_rightspeed = m_righttalon1->GetSelectedSensorVelocity(0);
		m_rightposition = m_righttalon1->GetSelectedSensorPosition(0);
	}

	switch (m_mode)
	{
	case DriveMode::kFollower:
		// can talon follower mode
		m_lefttalon1->Set(m_invertleft * m_coasting * LeftMotor(maxpower));
		m_righttalon1->Set(m_invertright * m_coasting * RightMotor(maxpower));
		break;

	case DriveMode::kDiscrete:
		// can talon discrete mode
		m_lefttalon1->Set(m_invertleft * m_coasting * LeftMotor(maxpower));
		m_lefttalon2->Set(m_invertleft * m_coasting * LeftMotor(maxpower));
		if (m_lefttalon3 != nullptr)
			m_lefttalon3->Set(m_invertleft * m_coasting * LeftMotor(maxpower));
		m_righttalon1->Set(m_invertright * m_coasting * RightMotor(maxpower));
		m_righttalon2->Set(m_invertright * m_coasting * RightMotor(maxpower));
		if (m_righttalon3 != nullptr)
			m_righttalon3->Set(m_invertleft * m_coasting * LeftMotor(maxpower));
		break;

	case DriveMode::kTank:
		// differentialdrive tank drive
		templeft = m_invertleft * m_coasting * LeftMotor(maxpower);
		tempright = m_invertright * m_coasting * RightMotor(maxpower) * -1;		// -1 added to adjust for DifferentialDrive
		m_differentialdrive->TankDrive(templeft, tempright, false);
		break;

	case DriveMode::kArcade:
		// differentialdrive arcade drive
		templeft = m_invertleft * m_coasting * LeftMotor(maxpower);
		tempright = m_invertright * m_coasting * RightMotor(maxpower) * -1;		// -1 added to adjust for DifferentialDrive
		tempforward = (templeft + tempright) / 2.0;
		temprotate = (templeft - tempright) / 2.0;
		m_differentialdrive->ArcadeDrive(tempforward, temprotate, false);
		break;

	case DriveMode::kCurvature:
		// differentialdrive curvature drive
		templeft = m_invertleft * m_coasting * LeftMotor(maxpower);
		tempright = m_invertright * m_coasting * RightMotor(maxpower) * -1;		// -1 added to adjust for DifferentialDrive
		tempforward = (templeft + tempright) / 2.0;
		temprotate = (templeft - tempright) / 2.0;
		tempspin = abs(tempforward) < DEADZONE_Y;
		m_differentialdrive->CurvatureDrive(tempforward, temprotate, tempspin);
		break;

	case DriveMode::kNone:
		break;
	}

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
void DriveTrain::Shift()
{
	m_ishighgear = !m_ishighgear;
	if (m_shifter != nullptr)
		m_shifter->Set(FLIP_HIGH_GEAR ^ m_ishighgear);
	m_shift = false;
}


// change direction and return true if going forward
bool DriveTrain::ChangeDirection()
{
	m_direction *= -1.0;
	return (m_direction == DT_DEFAULT_DIRECTION);
}


void DriveTrain::LowSpeedDriving()
{
	SmartDashboard::PutNumber("DT16_lowspeed", m_lowspeedmode);
	if (m_inputs->xBoxLeftTrigger(OperatorInputs::ToggleChoice::kToggle, 0 * INP_DUAL))
	{
		m_lowspeedmode = !m_lowspeedmode;
		if (m_ishighgear && m_lowspeedmode)
		{
			m_shift = true;
		}
	}
}


// ramp the power
double DriveTrain::Ramp(double previous, double desired, double rampmin, double rampmax)
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


double DriveTrain::LeftMotor(double &maxpower)
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


double DriveTrain::RightMotor(double &maxpower)
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


double DriveTrain::GetLeftPosition(int encoder)
{
	switch (encoder)
	{
	case 0:
		if (m_lefttalon1 != nullptr)
			return m_lefttalon1->GetSelectedSensorPosition(0);
		else
			return 0;
	case 1:
		if (m_lefttalon2 != nullptr)
			return m_lefttalon2->GetSelectedSensorPosition(0);
		else
			return 0;
	}
	return 0;
}


double DriveTrain::GetRightPosition(int encoder)
{
	switch (encoder)
	{
	case 0:
		if (m_righttalon1 != nullptr)
			return m_righttalon1->GetSelectedSensorPosition(0);
		else
			return 0;
	case 1:
		if (m_righttalon2 != nullptr)
			return m_righttalon2->GetSelectedSensorPosition(0);
		else
			return 0;
	}
	return 0;
}


double DriveTrain::GetLeftVelocity(int encoder)
{
	switch (encoder)
	{
	case 0:
		if (m_lefttalon1 != nullptr)
			return m_lefttalon1->GetSelectedSensorVelocity(0);
		else
			return 0;
	case 1:
		if (m_lefttalon2 != nullptr)
			return m_lefttalon2->GetSelectedSensorVelocity(0);
		else
			return 0;
	}
	return 0;
}


double DriveTrain::GetRightVelocity(int encoder)
{
	switch (encoder)
	{
	case 0:
		if (m_righttalon1 != nullptr)
			return m_righttalon1->GetSelectedSensorVelocity(0);
		else
			return 0;
	case 1:
		if (m_righttalon2 != nullptr)
			return m_righttalon2->GetSelectedSensorVelocity(0);
		else
			return 0;
	}
	return 0;
}


double DriveTrain::GetLeftDistance(int encoder)
{
	double distance = GetLeftPosition(encoder);
	distance = (distance / CODES_PER_REV) * WHEEL_DIAMETER * 3.1415926535;
	return distance;
}


double DriveTrain::GetRightDistance(int encoder)
{
	double distance = GetRightPosition(encoder);
	distance = (distance / CODES_PER_REV) * WHEEL_DIAMETER * 3.1415926535;
	return distance;
}


double DriveTrain::GetMaxDistance(int encoder)
{
	double maxleft = GetLeftDistance(encoder);
	double maxright = GetRightDistance(encoder);
	return abs(maxleft) > abs(maxright) ? maxleft : -maxright;
}


double DriveTrain::GetAverageMaxDistance(int encoder)
{
	double maxleft = GetLeftDistance(encoder);
	double maxright = GetRightDistance(encoder);
	return (abs(maxleft) + abs(maxright)) / 2;
}


double DriveTrain::GetMaxVelocity(int encoder)
{
	double maxleft = GetLeftVelocity(encoder);
	double maxright = GetRightVelocity(encoder);
	return abs(abs(maxleft) > abs(maxright) ? maxleft : -maxright);
}


void DriveTrain::ResetDeltaDistance(int encoder)
{
	m_prevleftdistance = GetLeftDistance(encoder);
	m_prevrightdistance = GetRightDistance(encoder);
}


double DriveTrain::GetMaxDeltaDistance(int encoder)
{
	double maxleft = GetLeftDistance(encoder)- m_prevleftdistance;
	double maxright = GetRightDistance(encoder) - m_prevrightdistance;
	return abs(maxleft) > abs(maxright) ? maxleft : -maxright;
}

