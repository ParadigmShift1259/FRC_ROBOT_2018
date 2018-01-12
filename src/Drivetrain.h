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
#include <XboxController.h>

class Drivetrain {
public:
	Drivetrain();

	//To be called every time the loop runs
	void loop();

	virtual ~Drivetrain();

private:

	//WPI_TalonSRX is a ctre class that inherits from speedcontroller and works over the can bus
	WPI_TalonSRX rightFrontTalon{1};
	WPI_TalonSRX rightBackTalon{3};
	WPI_TalonSRX leftFrontTalon{0};
	WPI_TalonSRX leftBackTalon{2};

	//SpeedControllerGroup basically nests multiple speedcontroller classes into one side
	frc::SpeedControllerGroup leftSide{leftFrontTalon, leftBackTalon};
	frc::SpeedControllerGroup rightSide{rightFrontTalon, leftBackTalon};

	//A class that has functions for curvature drive, arcade drive and tank drive.
	frc::DifferentialDrive drive{leftSide, rightSide};

	//Not sure if this is better than our custom operator inputs but it was fast
	XboxController *input;

};

#endif /* SRC_DRIVETRAIN_H_ */
