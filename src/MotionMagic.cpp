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
	rightFrontTalon = rightFront;
	rightBackTalon = rightBack;
	leftFrontTalon = leftFront;
	leftBackTalon = leftBack;

	oi = input;

	//////////////////Not positive this works for motion magic trajectories//////////////////
	rightBackTalon->Follow(*rightFrontTalon);
	leftBackTalon->Follow(*leftFrontTalon);

}

/*!
 * Init's the MotionMagic, must be called at least once before loop and after any
 * Init's that tinker with Talon's with the exception of testDriveInit if you
 * are testing.
 */
void MotionMagic::Init()
{
	rightFrontTalon->Set(ControlMode::MotionMagic, 0);
	rightFrontTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,0);
	rightFrontTalon->ConfigNominalOutputForward(0,0);
	rightFrontTalon->ConfigNominalOutputReverse(-0,0);
	rightFrontTalon->ConfigPeakOutputForward(12,0);
	rightFrontTalon->ConfigPeakOutputReverse(-12,0);
	rightFrontTalon->ConfigMotionCruiseVelocity(100,0); //Completely Arbitrary
	rightFrontTalon->ConfigMotionAcceleration(5,0);//Also Completely Arbitrary
	rightFrontTalon->Config_kP(0,KP,0);
	rightFrontTalon->Config_kI(0,KI,0);
	rightFrontTalon->Config_kD(0,KD,0);
	rightFrontTalon->Config_kF(0,KF,0);

	leftFrontTalon->Set(ControlMode::MotionMagic, 0);
	leftFrontTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,0);
	leftFrontTalon->ConfigNominalOutputForward(0,0);
	leftFrontTalon->ConfigNominalOutputReverse(-0,0);
	leftFrontTalon->ConfigPeakOutputForward(12,0);
	leftFrontTalon->ConfigPeakOutputReverse(-12,0);
	leftFrontTalon->ConfigMotionCruiseVelocity(100,0); //Completely Arbitrary
	leftFrontTalon->ConfigMotionAcceleration(5,0);//Also Completely Arbitrary
	leftFrontTalon->Config_kP(0,KP,0);
	leftFrontTalon->Config_kI(0,KI,0);
	leftFrontTalon->Config_kD(0,KD,0);
	leftFrontTalon->Config_kF(0,KF,0);

	frc::SmartDashboard::PutNumber("P",KP);
	frc::SmartDashboard::PutNumber("I",KI);
	frc::SmartDashboard::PutNumber("D",KD);
}

/*!
 * Needs to be called every time the roborio cycles.
 * TargetRotation is the target to approach in native units.
 */
void MotionMagic::Loop(double targetRotation)
{
	rightFrontTalon->Set(targetRotation);
	leftFrontTalon->Set(-targetRotation);
}

/*!
 * Must be called before running testDrive but after any other Init's that tinker with the talon's
 */
void MotionMagic::testDriveInit()
{
	p=KP;
	i=KI;
	d=KD;
	rightFrontTalon->Set(ControlMode::PercentOutput,0);
	rightBackTalon->Set(ControlMode::PercentOutput,0);
	leftFrontTalon->Set(ControlMode::PercentOutput,0);
	leftBackTalon->Set(ControlMode::PercentOutput,0);

	leftSide = new frc::SpeedControllerGroup(*leftFrontTalon, *leftBackTalon);
	rightSide = new frc::SpeedControllerGroup(*rightFrontTalon, *rightBackTalon);

	drive = new frc::DifferentialDrive(*leftSide, *rightSide);
	maxVel = 0;
}

/*!
 * Runs a basic Arcade Drive and outputs velocity and position to the SmartDashboard for recording purposes
 */
void MotionMagic::testDrive()
{

	drive->ArcadeDrive(-oi->xBoxLeftY(), -oi->xBoxLeftX(), false);
	if (rightFrontTalon->GetSelectedSensorVelocity(0) > maxVel)
		maxVel = rightFrontTalon->GetSelectedSensorVelocity(0);
	frc::SmartDashboard::PutNumber("MaxVelocity", maxVel);
	frc::SmartDashboard::PutNumber("rightTalonVelocity", rightFrontTalon->GetSelectedSensorVelocity(0));
	frc::SmartDashboard::PutNumber("leftTalonVelocity", leftFrontTalon->GetSelectedSensorVelocity(0));

	frc::SmartDashboard::PutNumber("rightTalonPosition", rightFrontTalon->GetSelectedSensorPosition(0));
	frc::SmartDashboard::PutNumber("leftTalonPosition", leftFrontTalon->GetSelectedSensorPosition(0));
}

/*!
 * For testing PID values, sets the target to the xBoxLeftY * 5000 in native units.
 * Also allows for PID Tuning through the SmartDashboard. Don't forget to put the PID
 * values into Const.h once done.
 */
void MotionMagic::testTarget()
{
	if(frc::SmartDashboard::GetNumber("P",0) != p)
	{
		p = frc::SmartDashboard::GetNumber("P",0);
		frc::SmartDashboard::PutNumber("P",p);
	}
	if(frc::SmartDashboard::GetNumber("I", 0) != i)
	{
		i = frc::SmartDashboard::GetNumber("I",0);
		frc::SmartDashboard::PutNumber("I",i);
	}
	if(frc::SmartDashboard::GetNumber("D", 0) != d)
	{
		i = frc::SmartDashboard::GetNumber("D",0);
		frc::SmartDashboard::PutNumber("D",d);
	}

	rightFrontTalon->Set(-oi->xBoxLeftY()*5000);
	leftFrontTalon->Set(oi->xBoxLeftY()*5000);

	frc::SmartDashboard::PutNumber("rightTalonVelocity", rightFrontTalon->GetSelectedSensorVelocity(0));
	frc::SmartDashboard::PutNumber("leftTalonVelocity", leftFrontTalon->GetSelectedSensorVelocity(0));

	frc::SmartDashboard::PutNumber("rightTalonPosition", rightFrontTalon->GetSelectedSensorPosition(0));
	frc::SmartDashboard::PutNumber("leftTalonPosition", leftFrontTalon->GetSelectedSensorPosition(0));
}

MotionMagic::~MotionMagic() {

}
