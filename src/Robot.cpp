/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


#include <iostream>
#include <string>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "Robot.h"


AutoMode automode = kAutoAuto;


void Robot::RobotInit()
{
	m_chooser.AddDefault(kAutoAutoMode, kAutoAutoMode);
	m_chooser.AddObject(kAutoTestMode, kAutoTestMode);
	m_chooser.AddObject(kAutoStageMode, kAutoStageMode);
	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

	m_driverstation = &DriverStation::GetInstance();
	m_operatorinputs = new OperatorInputs();
	m_drivetrain = new DriveTrain(m_operatorinputs);
	m_compressor = nullptr;
	if (PCM_COMPRESSOR_SOLENOID != -1)
		m_compressor = new Compressor(PCM_COMPRESSOR_SOLENOID);
	m_lifter = new Lifter(m_driverstation, m_operatorinputs);
	m_intake = new Intake(m_operatorinputs, m_lifter);
	m_climber = new Climber(m_operatorinputs);
	m_autonomous = new Autonomous(m_operatorinputs, m_drivetrain);
}


void Robot::RobotPeriodic()
{

}


/*
 * This autonomous (along with the chooser code above) shows how to
 * select
 * between different autonomous modes using the dashboard. The sendable
 * chooser code works with the Java SmartDashboard. If you prefer the
 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
 * GetString line to get the auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to
 * the
 * if-else structure below with additional strings. If using the
 * SendableChooser make sure to add them to the chooser code above as
 * well.
 */
void Robot::AutonomousInit()
{
	DriverStation::ReportError("AutonomousInit");

	m_drivetrain->Init(DriveTrain::DriveMode::kFollower);
	m_autonomous->Init();

	if (m_compressor != nullptr)
		m_compressor->Start();

	m_lifter->Init();
	m_intake->Init();
	m_climber->Init();
}


void Robot::AutonomousPeriodic()
{
	m_autonomous->Loop();
}


void Robot::TestInit()
{
	DriverStation::ReportError("TestInit");
}


void Robot::TestPeriodic()
{
}


void Robot::TeleopInit()
{
	m_autoSelected = m_chooser.GetSelected();

	if (m_autoSelected == kAutoAutoMode)
		automode = kAutoAuto;
	else
	if (m_autoSelected == kAutoTestMode)
		automode = kAutoTest;
	else
	if (m_autoSelected == kAutoStageMode)
		automode = kAutoStage;

	if (automode == kAutoTest)
		DriverStation::ReportError("TeleopInit Test Mode");
	else
	if (automode == kAutoStage)
		DriverStation::ReportError("TeleopInit Stage Mode");
	else
		DriverStation::ReportError("TeleopInit");

	if (m_compressor != nullptr)
		m_compressor->Start();
	m_drivetrain->Init(DriveTrain::DriveMode::kFollower);
	m_lifter->Init();
	m_intake->Init();
	m_climber->Init();
}


void Robot::TeleopPeriodic()
{
	if (automode == kAutoTest)
	{
		m_drivetrain->Loop();
		m_lifter->TestLoop();
		m_intake->TestLoop();
		m_climber->TestLoop();
	}
	else
	{
		m_drivetrain->Loop();
		m_lifter->Loop();
		m_intake->Loop();
		m_climber->Loop();
	}
}


void Robot::DisabledInit()
{
	if (m_compressor != nullptr)
		m_compressor->Stop();
	m_drivetrain->Stop();
	m_lifter->Stop();
	m_intake->Stop();
	m_climber->Stop();
	m_autonomous->Stop();
}


void Robot::DisablePeriodic()
{
	m_autoSelected = m_chooser.GetSelected();
}


START_ROBOT_CLASS(Robot)
