/*
 * Drivetrain.h
 *
 *  Created on: Jan 11, 2018
 *      Author: Matt Wildman
 */

#ifndef SRC_DRIVETRAIN_H_
#define SRC_DRIVETRAIN_H_
#include <RobotDrive.h>
#include <Drive\DifferentialDrive.h>
#include <ctre\Phoenix.h> //Includes all ctre classes. Do not include the individual header files, it won't work
#include <SpeedControllerGroup.h>
#include "OperatorInputs.h"
#include <DriverStation.h>
#include <XboxController.h>
#include <Solenoid.h>

class Drivetrain
{
public:
	Drivetrain();

	void init();

	//To be called every time the loop runs
	void loop();

	//Checks the controller and if necessary shifts the drivetrain
	void shift();
	virtual ~Drivetrain();
	double getXboxX() const { return xBoxX; }

private:
	OperatorInputs *oi;
	XboxController *input;

	//Stores the current x and y values of the joystick multiplied by -1
	double xBoxX;
	double xBoxY;

	//WPI_TalonSRX is a CTRE class that inherits from speedcontroller and works over the can bus
	WPI_TalonSRX *rightFrontTalon;
	WPI_TalonSRX *rightBackTalon;
	WPI_TalonSRX *leftFrontTalon;
	WPI_TalonSRX *leftBackTalon;

	//SpeedControllerGroup basically nests multiple speedcontroller classes into one side
	frc::SpeedControllerGroup *leftSide;
	frc::SpeedControllerGroup *rightSide;

	//A class that has functions for curvature drive, arcade drive and tank drive.
	frc::DifferentialDrive *drive;

	Solenoid *shifter;
};

#endif /* SRC_DRIVETRAIN_H_ */
