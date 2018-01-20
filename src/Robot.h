/*
 * Robot.h
 *
 *  Created on: Jan 11, 2018
 *      Author: Developer
 */

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_


#include "Const.h"
#include "OperatorInputs.h"
#include <Compressor.h>
#include "MotionMagic.h"
#include <Drivetrain.h>


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
	MotionMagic *m_motionmagic;
	DriverStation *m_driverstation;

	WPI_TalonSRX *m_rightFrontTalon;
	WPI_TalonSRX *m_rightBackTalon;
	WPI_TalonSRX *m_leftFrontTalon;
	WPI_TalonSRX *m_leftBackTalon;
};


#endif /* SRC_ROBOT_H_ */
