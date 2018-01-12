/*
 * Drivetrain.cpp
 *
 *  Created on: Jan 11, 2018
 *      Author: Developer
 */

#include <Drivetrain.h>

Drivetrain::Drivetrain() {

	input = new XboxController(0);
}

void Drivetrain::loop()
{
	drive.CurvatureDrive(input->GetY(GenericHID::kLeftHand), input->GetX(GenericHID::kRightHand), false);
}
Drivetrain::~Drivetrain() {

}

