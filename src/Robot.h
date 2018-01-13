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
#include "Drivetrain.h"
#include <Compressor.h>


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
	Drivetrain *m_drivetrain;
	Compressor *m_compressor;

	WPI_TalonSRX *rightFrontTalon;
	WPI_TalonSRX *rightBackTalon;
	WPI_TalonSRX *leftFrontTalon;
	WPI_TalonSRX *leftBackTalon;

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};


#endif /* SRC_ROBOT_H_ */
