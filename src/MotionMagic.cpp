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
	//Not Sure what this needs to be
	//rightFrontTalon->SetFeedBackDevice(FeedbackDevice::QuadEncoder);
	//rightFrontTalon->configEncoderCodesPerRev(XXX);
	//rightFrontTalon->SetPIDProfile(0);
	//rightFrontTalon->SetF(0);
	leftFrontTalon->Set(ControlMode::MotionMagic, 0);
}

void MotionMagic::Loop(double targetRotation)
{
	rightFrontTalon->Set(targetRotation);
	leftFrontTalon->Set(targetRotation);
}

MotionMagic::~MotionMagic() {

}
