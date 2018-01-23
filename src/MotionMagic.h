/*
 * MotionMagic.h
 *
 *  Created on: Jan 18, 2018
 *      Author: Developer
 */

#ifndef SRC_MOTIONMAGIC_H_
#define SRC_MOTIONMAGIC_H_

#include <WPIlib.h>
#include <ctre\Phoenix.h>
#include "OperatorInputs.h"
#include "Const.h"

class MotionMagic
{
public:
	MotionMagic(WPI_TalonSRX *rightFront, WPI_TalonSRX *rightBack, WPI_TalonSRX *leftFront, WPI_TalonSRX *leftBack, OperatorInputs *input);
	virtual ~MotionMagic();
	void Init();						//!<Standard Init
	void Loop(double targetRotation);	//!<Main Loop Function
	void testDriveInit();				//!<For testing purposes
	void testDrive();					//!<For testing purposes
	void testTarget();					//!<For testing purposes

protected:
	SpeedControllerGroup *m_leftSide;		//!<Purely used for testDrive, configured in testDriveInit
	SpeedControllerGroup *m_rightSide;	//!<Purely used for testDrive, configured in testDriveInit

	DifferentialDrive *m_drive;			//!<Purely used for testDrive, configured in testDriveInit
	OperatorInputs *m_oi;					//!<Purely used for testDrive and testTarget, configured in the constructor
	WPI_TalonSRX *m_rightLeadTalon;		//!<Right Front Talon, executes motion magic functions
	WPI_TalonSRX *m_rightFollowTalon;		//!<Right Back Talon, follows Right Front Talon
	WPI_TalonSRX *m_leftLeadTalon;		//!<Left Front Talon, executes motion magic functions
	WPI_TalonSRX *m_leftFollowTalon;		//!<Left Back Talon, follows Left Front Talon

	double m_p;							//!<Used in testDrive to set PID values from SmartDashboard
	double m_i;							//!<Used in testDrive to set PID values from SmartDashboard
	double m_d;							//!<Used in testDrive to set PID values from SmartDashboard
	double m_maxVel;						//!<Used in testDrive to maintain the fastest velocity aquired
};

#endif /* SRC_MOTIONMAGIC_H_ */
