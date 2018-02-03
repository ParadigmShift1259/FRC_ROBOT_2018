#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#include <Drivetrain.h>
#include <Intake.h>
#include <WPILib.h>
#include "Autonomous.h"
#include "Const.h"
#include "OperatorInputs.h"
#include "Lifter.h"
#include "Climber.h"
#include "Autonomous.h"


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
	Intake *m_intake;
	Climber *m_climber;
	Autonomous *m_autonomous;

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};


#endif /* SRC_ROBOT_H_ */
