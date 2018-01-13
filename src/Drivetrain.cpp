/*
 * Drivetrain.cpp
 *
 *  Created on: Jan 11, 2018
 *      Author: Matt Wildman
 */

#include <Drivetrain.h>


Drivetrain::Drivetrain(OperatorInputs *operatorinputs)
{
	m_operatorinputs = operatorinputs;
	//input = new XboxController(1);

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


Drivetrain::~Drivetrain()
{
}


void Drivetrain::Init()
{
	xBoxX = 0;
	xBoxY = 0;

	shifter->Set(false);
}


void Drivetrain::Loop()
{
	xBoxX = -m_operatorinputs->xBoxLeftX();
	xBoxY = -m_operatorinputs->xBoxLeftY();

	drive->ArcadeDrive(xBoxY, xBoxX, false);

	Shift();
}


void Drivetrain::Shift()
{
	if (m_operatorinputs->xBoxLeftBumper())
	{
		shifter->Set(!shifter->Get());
	}
}
