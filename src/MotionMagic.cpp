/*
 * MotionMagic.cpp
 *
 *  Created on: Jan 18, 2018
 *      Author: Developer
 */

#include <MotionMagic.h>

/*!
 * Constructor for MotionMagic, takes four pointers for each of the Talons, the state of the
 * Talons does not matter. Does not initialize anything for testDrive or testTarget besides
 * OperatorInputs.
 */
MotionMagic::MotionMagic(WPI_TalonSRX *rightFront, WPI_TalonSRX *rightBack, WPI_TalonSRX *leftFront, WPI_TalonSRX *leftBack, OperatorInputs *input)
{
	m_rightLeadTalon = rightFront;
	m_rightFollowTalon = rightBack;
	m_leftLeadTalon = leftFront;
	m_leftFollowTalon = leftBack;

	m_oi = input;

	//////////////////Not positive this works for motion magic trajectories//////////////////
	m_rightFollowTalon->Follow(*m_rightLeadTalon);
	m_leftFollowTalon->Follow(*m_leftLeadTalon);

	m_p = KP;
	m_i = KI;
	m_d = KD;
	m_maxVel = 0.0;

	m_drive = nullptr;
	m_rightSide = nullptr;
	m_leftSide = nullptr;
}

/*!
 * Init's the MotionMagic, must be called at least once before loop and after any
 * Init's that tinker with Talon's with the exception of testDriveInit if you
 * are testing.
 */
void MotionMagic::Init()
{
	m_rightLeadTalon->Set(ControlMode::MotionMagic, 0);
	m_rightLeadTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,0);
	m_rightLeadTalon->ConfigNominalOutputForward(0,0);
	m_rightLeadTalon->ConfigNominalOutputReverse(-0,0);
	m_rightLeadTalon->ConfigPeakOutputForward(12,0);
	m_rightLeadTalon->ConfigPeakOutputReverse(-12,0);
	m_rightLeadTalon->ConfigMotionCruiseVelocity(100,0); //Completely Arbitrary
	m_rightLeadTalon->ConfigMotionAcceleration(5,0);//Also Completely Arbitrary
	m_rightLeadTalon->Config_kP(0,KP,0);
	m_rightLeadTalon->Config_kI(0,KI,0);
	m_rightLeadTalon->Config_kD(0,KD,0);
	m_rightLeadTalon->Config_kF(0,KF,0);

	m_leftLeadTalon->Set(ControlMode::MotionMagic, 0);
	m_leftLeadTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,0);
	m_leftLeadTalon->ConfigNominalOutputForward(0,0);
	m_leftLeadTalon->ConfigNominalOutputReverse(-0,0);
	m_leftLeadTalon->ConfigPeakOutputForward(12,0);
	m_leftLeadTalon->ConfigPeakOutputReverse(-12,0);
	m_leftLeadTalon->ConfigMotionCruiseVelocity(100,0); //Completely Arbitrary
	m_leftLeadTalon->ConfigMotionAcceleration(5,0);//Also Completely Arbitrary
	m_leftLeadTalon->Config_kP(0,KP,0);
	m_leftLeadTalon->Config_kI(0,KI,0);
	m_leftLeadTalon->Config_kD(0,KD,0);
	m_leftLeadTalon->Config_kF(0,KF,0);

	SmartDashboard::PutNumber("P",KP);
	SmartDashboard::PutNumber("I",KI);
	SmartDashboard::PutNumber("D",KD);
}

/*!
 * Needs to be called every time the roborio cycles.
 * TargetRotation is the target to approach in native units.
 */
void MotionMagic::Loop(double targetRotation)
{
	m_rightLeadTalon->Set(targetRotation);
	m_leftLeadTalon->Set(-targetRotation);
}

/*!
 * Must be called before running testDrive but after any other Init's that tinker with the talon's
 */
void MotionMagic::testDriveInit()
{
	m_p = KP;
	m_i = KI;
	m_d = KD;
	m_rightLeadTalon->Set(ControlMode::PercentOutput,0);
	m_rightFollowTalon->Set(ControlMode::PercentOutput,0);
	m_leftLeadTalon->Set(ControlMode::PercentOutput,0);
	m_leftFollowTalon->Set(ControlMode::PercentOutput,0);

	m_leftSide = new SpeedControllerGroup(*m_leftLeadTalon, *m_leftFollowTalon);
	m_rightSide = new SpeedControllerGroup(*m_rightLeadTalon, *m_rightFollowTalon);

	m_drive = new DifferentialDrive(*m_leftSide, *m_rightSide);
	m_maxVel = 0;
}

/*!
 * Runs a basic Arcade Drive and outputs velocity and position to the SmartDashboard for recording purposes
 */
void MotionMagic::testDrive()
{

	m_drive->ArcadeDrive(-m_oi->xBoxLeftY(), -m_oi->xBoxLeftX(), false);
	if (m_rightLeadTalon->GetSelectedSensorVelocity(0) > m_maxVel)
		m_maxVel = m_rightLeadTalon->GetSelectedSensorVelocity(0);
	SmartDashboard::PutNumber("MaxVelocity", m_maxVel);
	SmartDashboard::PutNumber("rightTalonVelocity", m_rightLeadTalon->GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("leftTalonVelocity", m_leftLeadTalon->GetSelectedSensorVelocity(0));

	SmartDashboard::PutNumber("rightTalonPosition", m_rightLeadTalon->GetSelectedSensorPosition(0));
	SmartDashboard::PutNumber("leftTalonPosition", m_leftLeadTalon->GetSelectedSensorPosition(0));
}

/*!
 * For testing PID values, sets the target to the xBoxLeftY * 5000 in native units.
 * Also allows for PID Tuning through the SmartDashboard. Don't forget to put the PID
 * values into Const.h once done.
 */
void MotionMagic::testTarget()
{
	if (SmartDashboard::GetNumber("P",0) != m_p)
	{
		m_p = SmartDashboard::GetNumber("P",0);
		SmartDashboard::PutNumber("P",m_p);
	}
	if (SmartDashboard::GetNumber("I", 0) != m_i)
	{
		m_i = SmartDashboard::GetNumber("I",0);
		SmartDashboard::PutNumber("I",m_i);
	}
	if (SmartDashboard::GetNumber("D", 0) != m_d)
	{
		m_i = SmartDashboard::GetNumber("D",0);
		SmartDashboard::PutNumber("D",m_d);
	}

	m_rightLeadTalon->Set(-m_oi->xBoxLeftY()*5000);
	m_leftLeadTalon->Set(m_oi->xBoxLeftY()*5000);

	SmartDashboard::PutNumber("rightTalonVelocity", m_rightLeadTalon->GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("leftTalonVelocity", m_leftLeadTalon->GetSelectedSensorVelocity(0));

	SmartDashboard::PutNumber("rightTalonPosition", m_rightLeadTalon->GetSelectedSensorPosition(0));
	SmartDashboard::PutNumber("leftTalonPosition", m_leftLeadTalon->GetSelectedSensorPosition(0));
}

MotionMagic::~MotionMagic() {

}
