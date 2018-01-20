/*
 * MotionMagic.cpp
 *
 *  Created on: Jan 18, 2018
 *      Author: Developer
 */

#include <MotionMagic.h>

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
	rightFrontTalon->Config_kP(0,0.0,0);
	rightFrontTalon->Config_kI(0,0.0,0);
	rightFrontTalon->Config_kD(0,0.0,0);
	rightFrontTalon->Config_kF(0,0.0,0);

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
}

void MotionMagic::Loop(double targetRotation)
{
	rightFrontTalon->Set(targetRotation);
	leftFrontTalon->Set(targetRotation);
}

MotionMagic::~MotionMagic() {

}
