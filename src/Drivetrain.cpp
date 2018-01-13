/*
 * Drivetrain.cpp
 *
 *  Created on: Jan 11, 2018
 *      Author: Matt Wildman
 */

#include <Drivetrain.h>

Drivetrain::Drivetrain()
{
	oi = new OperatorInputs();
	input = new XboxController(1);

	xBoxY = 0;
	xBoxX = 0;

	rightFrontTalon = new WPI_TalonSRX(1);
	rightBackTalon = new WPI_TalonSRX(3);
	leftFrontTalon = new WPI_TalonSRX(0);
	leftBackTalon = new WPI_TalonSRX(2);

	leftSide = new frc::SpeedControllerGroup(*leftFrontTalon, *leftBackTalon);
	rightSide = new frc::SpeedControllerGroup(*rightFrontTalon, *rightBackTalon);

	drive = new frc::DifferentialDrive(*leftSide, *rightSide);

	shifter = new Solenoid(0);

}

void Drivetrain::init()
{
	xBoxX = 0;
	xBoxY = 0;

	shifter->Set(false);
}

void Drivetrain::loop()
{
	xBoxX = -input->GetX(GenericHID::kLeftHand);
	xBoxY = -input->GetY(GenericHID::kLeftHand);

	drive->ArcadeDrive(xBoxY, xBoxX, false);

	shift();
}

void Drivetrain::shift()
{
	if (input->GetBumperPressed(GenericHID::kLeftHand))
	{
		shifter->Set(!shifter->Get());
	}
}


Drivetrain::~Drivetrain()
{

}

