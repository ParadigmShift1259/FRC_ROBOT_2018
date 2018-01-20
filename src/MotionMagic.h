/*
 * MotionMagic.h
 *
 *  Created on: Jan 18, 2018
 *      Author: Developer
 */

#ifndef SRC_MOTIONMAGIC_H_
#define SRC_MOTIONMAGIC_H_

#include <ctre\Phoenix.h>
#include <SpeedControllerGroup.h>
#include <drive/DifferentialDrive.h>
#include "OperatorInputs.h"
#include <SmartDashboard/SmartDashboard.h>
#include "Const.h"

class MotionMagic {
public:
	MotionMagic(WPI_TalonSRX *rightFront, WPI_TalonSRX *rightBack, WPI_TalonSRX *leftFront, WPI_TalonSRX *leftBack, OperatorInputs *input);

	void Init();						//!<Standard Init

	void Loop(double targetRotation);	//!<Main Loop Function

	void testDriveInit();				//!<For testing purposes

	void testDrive();					//!<For testing purposes

	void testTarget();					//!<For testing purposes
	virtual ~MotionMagic();

protected:
	SpeedControllerGroup *leftSide;		//!<Purely used for testDrive, configured in testDriveInit
	SpeedControllerGroup *rightSide;	//!<Purely used for testDrive, configured in testDriveInit

	DifferentialDrive *drive;			//!<Purely used for testDrive, configured in testDriveInit
	OperatorInputs *oi;					//!<Purely used for testDrive and testTarget, configured in the constructor
	WPI_TalonSRX *rightFrontTalon;		//!<Right Front Talon, executes motion magic functions
	WPI_TalonSRX *rightBackTalon;		//!<Right Back Talon, follows Right Front Talon
	WPI_TalonSRX *leftFrontTalon;		//!<Left Front Talon, executes motion magic functions
	WPI_TalonSRX *leftBackTalon;		//!<Left Back Talon, follows Left Front Talon

	double p;							//!<Used in testDrive to set PID values from SmartDashboard
	double i;							//!<Used in testDrive to set PID values from SmartDashboard
	double d;							//!<Used in testDrive to set PID values from SmartDashboard
	double maxVel;						//!<Used in testDrive to maintain the fastest velocity aquired

};

#endif /* SRC_MOTIONMAGIC_H_ */
