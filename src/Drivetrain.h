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
#include <SpeedControllerGroup.h>
#include <ctre\Phoenix.h>
#include <DriverStation.h>
#include <XboxController.h>
#include <Solenoid.h>
#include "OperatorInputs.h"


class Drivetrain
{
public:
	Drivetrain();

	//Checks the controller and if necessary shifts the drivetrain
	virtual ~Drivetrain();
	double getXboxX() const { return xBoxX; }

	Drivetrain(OperatorInputs *operatorinputs, WPI_TalonSRX *rightFront, WPI_TalonSRX *rightBack, WPI_TalonSRX *leftFront, WPI_TalonSRX *leftBack);

	void Init();
	void Loop();
	void Shift();


private:
	OperatorInputs *m_operatorinputs;

	//Stores the current x and y values of the joystick multiplied by -1
	double xBoxX;
	double xBoxY;

	//WPI_TalonSRX is a CTRE class that inherits from speedcontroller and works over the can bus
	WPI_TalonSRX *rightFrontTalon;
	WPI_TalonSRX *rightBackTalon;
	WPI_TalonSRX *leftFrontTalon;
	WPI_TalonSRX *leftBackTalon;

	//SpeedControllerGroup basically nests multiple speedcontroller classes into one side
	SpeedControllerGroup *leftSide;
	SpeedControllerGroup *rightSide;

	//A class that has functions for curvature drive, arcade drive and tank drive.
	DifferentialDrive *drive;

	Solenoid *shifter;
};


#endif /* SRC_DRIVETRAIN_H_ */
