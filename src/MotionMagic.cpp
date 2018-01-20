/*
 * MotionMagic.cpp
 *
 *  Created on: Jan 18, 2018
 *      Author: Developer
 */

#include <MotionMagic.h>

/*!
 * Constructor for MotionMagic, takes four pointers for each of the Talons, the state of the
 * Talons does not matter
 */
MotionMagic::MotionMagic(WPI_TalonSRX *rightFront, WPI_TalonSRX *rightBack, WPI_TalonSRX *leftFront, WPI_TalonSRX *leftBack)
{
	rightFrontTalon = rightFront;
	rightBackTalon = rightBack;
	leftFrontTalon = leftFront;
	leftBackTalon = leftBack;

	//////////////////Not positive this works for motion magic trajectories//////////////////
	rightBackTalon->Follow(*rightFrontTalon);
	leftBackTalon->Follow(*leftFrontTalon);

}

/*!
 * Init's the MotionMagic, must be called at least once before loop
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
	leftFrontTalon->Config_kP(0,0.0,0);
	leftFrontTalon->Config_kI(0,0.0,0);
	leftFrontTalon->Config_kD(0,0.0,0);
	leftFrontTalon->Config_kF(0,0.0,0);

	frc::SmartDashboard::PutNumber("P",KP);
	frc::SmartDashboard::PutNumber("I",KI);
	frc::SmartDashboard::PutNumber("D",KD);
}

/*!
 * Needs to be called every time the roborio cycles.
 * TargetRotation is the target to approach in native units
 */
void MotionMagic::Loop(double targetRotation)
{
	rightFrontTalon->Set(targetRotation);
	leftFrontTalon->Set(-targetRotation);
}

/*!
 * Must be called before running testDrive
 */
void MotionMagic::testDriveInit()
{
	oi = new OperatorInputs();

	rightFrontTalon->Set(ControlMode::PercentOutput,0);
	rightBackTalon->Set(ControlMode::PercentOutput,0);
	leftFrontTalon->Set(ControlMode::PercentOutput,0);
	leftBackTalon->Set(ControlMode::PercentOutput,0);

	leftSide = new frc::SpeedControllerGroup(*leftFrontTalon, *leftBackTalon);
	rightSide = new frc::SpeedControllerGroup(*rightFrontTalon, *rightBackTalon);

	drive = new frc::DifferentialDrive(*leftSide, *rightSide);
}

/*!
 * Runs a basic Arcade Drive and outputs velocity and position to the SmartDashboard for recording purposes
 */
void MotionMagic::testDrive()
{

	drive->ArcadeDrive(-oi->xBoxLeftY(), -oi->xBoxLeftX(), false);
	frc::SmartDashboard::PutNumber("rightTalonVelocity", rightFrontTalon->GetSelectedSensorVelocity(0));
	frc::SmartDashboard::PutNumber("leftTalonVelocity", leftFrontTalon->GetSelectedSensorVelocity(0));

	frc::SmartDashboard::PutNumber("rightTalonPosition", rightFrontTalon->GetSelectedSensorPosition(0));
	frc::SmartDashboard::PutNumber("leftTalonPosition", leftFrontTalon->GetSelectedSensorPosition(0));
}

/*!
 * For testing PID values, sets the target to the xBoxLeftY * 5000 in native units
 */
void MotionMagic::testTarget()
{
	rightFrontTalon->Set(-oi->xBoxLeftY()*5000);
	leftFrontTalon->Set(oi->xBoxLeftY()*5000);

	frc::SmartDashboard::PutNumber("rightTalonVelocity", rightFrontTalon->GetSelectedSensorVelocity(0));
	frc::SmartDashboard::PutNumber("leftTalonVelocity", leftFrontTalon->GetSelectedSensorVelocity(0));

	frc::SmartDashboard::PutNumber("rightTalonPosition", rightFrontTalon->GetSelectedSensorPosition(0));
	frc::SmartDashboard::PutNumber("leftTalonPosition", leftFrontTalon->GetSelectedSensorPosition(0));
}

MotionMagic::~MotionMagic() {

}
