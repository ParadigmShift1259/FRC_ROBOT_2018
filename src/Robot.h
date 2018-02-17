#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_


#include <WPILib.h>
#include "Const.h"
#include "OperatorInputs.h"
#include "Drivetrain.h"
#include "Lifter.h"
#include "Intake.h"
#include "Climber.h"
#include "Autonomous.h"


class Robot : public TimedRobot
{
public:
	enum TurnState {kInit, kTurning, kIdle};
	virtual void RobotInit();
	virtual void RobotPeriodic();
	virtual void AutonomousInit();
	virtual void AutonomousPeriodic();
	virtual void TeleopInit();
	virtual void TeleopPeriodic();
	virtual void TestInit();
	virtual void TestPeriodic();
	virtual void DisabledInit();
	virtual void DisabledPeriodic();

protected:
	OperatorInputs *m_operatorinputs;
	DriveTrain *m_drivetrain;
	DrivePID *m_drivepid;
	Compressor *m_compressor;
	DriverStation *m_driverstation;
	Lifter *m_lifter;
	Intake *m_intake;
	Climber *m_climber;
	Autonomous *m_autonomous;
	PigeonIMU *m_pigeon;
	TurnState m_turn=kInit;

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoAutoMode = "Auto Mode";
	const std::string kAutoTestMode = "Test Mode";
	std::string m_autoSelected;
	double m_gyroval[3];
};


#endif /* SRC_ROBOT_H_ */
