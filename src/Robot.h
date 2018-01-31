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
#include "Drivetrain.h"
#include "Lifter.h"
#include "Grabber.h"
#include "Climber.h"
#include "DrivePID.h"


class Robot : public TimedRobot
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
	DriverStation *m_driverstation;
	Lifter *m_lifter;
	Grabber *m_grabber;
	Climber *m_climber;
	DrivePID *m_drivepid;

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};


#endif /* SRC_ROBOT_H_ */
