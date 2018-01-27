/*
 * Robot.h
 *
 *  Created on: Jan 11, 2018
 *      Author: Developer
 */

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_


#include <WPILib.h>
#include "Const.h"
#include "OperatorInputs.h"
#include "MotionProfiling.h"
#include "Drivetrain.h"
#include "Lifter.h"
#include "Grabber.h"
#include "Climber.h"


class Robot : public IterativeRobot
{
public:
	virtual void RobotInit();
	virtual void AutonomousInit();
	virtual void AutonomousPeriodic();
	virtual void TeleopInit();
	virtual void TeleopPeriodic();
	virtual void TestInit();
	virtual void TestPeriodic();
	virtual void DisabledInit();

protected:
	OperatorInputs *m_operatorinputs;
	DriveTrain *m_drivetrain;

	Compressor *m_compressor;
	MotionProfiling *m_motionprofile;
	DriverStation *m_driverstation;
	Lifter *m_lifter;
	Grabber *m_grabber;
	Climber *m_climber;


	WPI_TalonSRX *m_rightFrontTalon;
	WPI_TalonSRX *m_rightBackTalon;
	WPI_TalonSRX *m_leftFrontTalon;
	WPI_TalonSRX *m_leftBackTalon;
};


#endif /* SRC_ROBOT_H_ */
