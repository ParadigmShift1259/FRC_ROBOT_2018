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


void Robot::RobotInit()
{
	m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
	m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

	m_driverstation = &DriverStation::GetInstance();
	m_operatorinputs = new OperatorInputs();
	m_drivetrain = new DriveTrain(m_operatorinputs);
	m_compressor = nullptr;
	if (PCM_COMPRESSOR_SOLENOID != -1)
		m_compressor = new Compressor(PCM_COMPRESSOR_SOLENOID);
	m_lifter = new Lifter(m_operatorinputs);
	m_grabber = new Grabber(m_operatorinputs);
	m_climber = new Climber(m_operatorinputs);
	m_autonomous = new Autonomous(m_operatorinputs, m_drivetrain);
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
/*
	m_autoSelected = m_chooser.GetSelected();
	// m_autoSelected = SmartDashboard::GetString(
	// 		"Auto Selector", kAutoNameDefault);
	std::cout << "Auto selected: " << m_autoSelected << std::endl;

	if (m_autoSelected == kAutoNameCustom)
	{
		// Custom Auto goes here
	}
	else
	{
		// Default Auto goes here
	}
*/
	DriverStation::ReportError("AutonomousInit");
	if (m_compressor != nullptr)
		m_compressor->Start();
	m_drivetrain->Init(DriveTrain::DriveMode::kFollower);
	m_lifter->Init();
	m_grabber->Init();
	m_climber->Init();
	m_autonomous->Init();
}


void Robot::AutonomousPeriodic()
{
/*
	if (m_autoSelected == kAutoNameCustom)
	{
		// Custom Auto goes here
	}
	else
	{
		// Default Auto goes here
	}
*/
//	DriverStation::ReportError("AutonomousPeriodic");
	m_autonomous->Loop();
}


void Robot::TestInit()
{
	DriverStation::ReportError("TestInit");
	if (m_compressor != nullptr)
		m_compressor->Start();
	m_drivetrain->Init(DriveTrain::DriveMode::kFollower);
	m_lifter->Init();
	m_grabber->Init();
	m_climber->Init();
}


void Robot::TestPeriodic()
{
	m_drivetrain->Loop();
	m_lifter->Loop();
	m_grabber->Loop();
	m_climber->Loop();
}


void Robot::TeleopInit()
{
	DriverStation::ReportError("TeleopInit");
	if (m_compressor != nullptr)
		m_compressor->Start();
	m_drivetrain->Init(DriveTrain::DriveMode::kFollower);
	m_lifter->Init();
	m_grabber->Init();
	m_climber->Init();
}


void Robot::TeleopPeriodic()
{
	m_drivetrain->Loop();
	m_lifter->Loop();
	m_grabber->Loop();
	m_climber->Loop();
}


void Robot::DisabledInit()
{
	if (m_compressor != nullptr)
		m_compressor->Stop();
	m_drivetrain->Stop();
	m_lifter->Stop();
	m_grabber->Stop();
	m_climber->Stop();
	m_autonomous->Stop();
}


START_ROBOT_CLASS(Robot)
