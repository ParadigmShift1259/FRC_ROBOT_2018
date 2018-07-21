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
#include "Lidar.h"


class Robot : public TimedRobot
{
public:
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
	Lidar *lidar;

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kszAutoDefault = "NO AUTO";
	const std::string kszAutoCenterSwitch1 = "Center Switch 1 Cube";
	const std::string kszAutoCenterSwitch3 = "Center Switch 3 Cube";
	const std::string kszAutoLeftScale2X = "Left Scale 2 Cube Cross";
	const std::string kszAutoRightScale2X = "Right Scale 2 Cube Cross";
	const std::string kszAutoLeftScale1P = "Left Scale 1 Cube Pause";
	const std::string kszAutoRightScale1P = "Right Scale 1 Cube Pause";
	const std::string kszAutoLeftSwitch = "Left Switch Straight";
	const std::string kszAutoRightSwitch = "Right Switch Straight";
	const std::string kszAutoTestMode = "Test Mode";
	std::string m_autoSelected;

	void ReadChooser();
};


#endif /* SRC_ROBOT_H_ */
