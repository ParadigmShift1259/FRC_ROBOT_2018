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


AutoMode automode = kAutoStraight;


void Robot::RobotInit()
{
	m_chooser.AddDefault(kszAutoDefault, kszAutoDefault);
	m_chooser.AddObject(kszAutoCenterSwitch1, kszAutoCenterSwitch1);
	m_chooser.AddObject(kszAutoCenterSwitch3, kszAutoCenterSwitch3);
	m_chooser.AddObject(kszAutoLeftScale2X, kszAutoLeftScale2X);
	m_chooser.AddObject(kszAutoRightScale2X, kszAutoRightScale2X);
	m_chooser.AddObject(kszAutoRightScale2X, kszAutoRightScale2X);
	m_chooser.AddObject(kszAutoLeftScale1P, kszAutoLeftScale1P);
	m_chooser.AddObject(kszAutoRightScale1P, kszAutoRightScale1P);
	m_chooser.AddObject(kszAutoLeftSwitch, kszAutoLeftSwitch);
	m_chooser.AddObject(kszAutoRightSwitch, kszAutoRightSwitch);
	m_chooser.AddObject(kszAutoTestMode, kszAutoTestMode);
	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

	m_driverstation = &DriverStation::GetInstance();
	m_compressor = nullptr;
	if (PCM_COMPRESSOR_SOLENOID != -1)
		m_compressor = new Compressor(PCM_COMPRESSOR_SOLENOID);

	m_operatorinputs = new OperatorInputs();
	m_drivetrain = new DriveTrain(m_operatorinputs);
	m_drivepid = new DrivePID(m_drivetrain, m_operatorinputs);
	m_lifter = new Lifter(m_driverstation, m_operatorinputs);
	m_intake = new Intake(m_driverstation, m_operatorinputs, m_lifter, m_drivepid);
	m_climber = new Climber(m_operatorinputs);
	m_autonomous = new Autonomous(m_operatorinputs, m_drivetrain, m_drivepid, m_intake, m_lifter);
	lidar = new Lidar();
	SmartDashboard::PutNumber("P", 0.01);
	SmartDashboard::PutNumber("I", 0.0012);
	SmartDashboard::PutNumber("D", 0.07);
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
//	DriverStation::ReportError("AutonomousInit");
//
//	if (m_compressor != nullptr)
//		m_compressor->Stop();
//	ReadChooser();
//	m_drivetrain->Init(DriveTrain::DriveMode::kFollower);
//	m_lifter->Init();
//	m_intake->Init();
//	m_climber->Init();
//	m_autonomous->Init();
	lidar->start();
	lidar->SetScanPeriod(1000);
	lidar->SetMinMaxAngle(0, 360);
	lidar->SetParkTrim(0);
	lidar->SetSamplesPerScan(360);
	lidar->SetSampleRejectionMode(false);
}


void Robot::AutonomousPeriodic()
{
	lidar->Loop();
//	m_lifter->Loop();
//	m_intake->AutoLoop();
//	m_climber->Loop();
//	m_autonomous->Loop();
//	m_drivepid->Loop();
}


void Robot::TestInit()
{
	DriverStation::ReportError("TestInit");

}


void Robot::TestPeriodic()
{
//	lidar->stop();
}


void Robot::TeleopInit()
{
//	if (automode == kAutoTest)
//		DriverStation::ReportError("TeleopInit Test Mode");
//	else
//		DriverStation::ReportError("TeleopInit");
//
//	if (m_compressor != nullptr)
//		m_compressor->Start();
//	m_drivetrain->Init(DriveTrain::DriveMode::kFollower);
//	m_lifter->Init();
//	m_intake->Init();
//	m_climber->Init();
	lidar->stop();
}


void Robot::TeleopPeriodic()
{
//	if (automode == kAutoTest)
//	{
//		m_lifter->TestLoop();
//		m_intake->TestLoop();
//		m_climber->TestLoop();
//		m_drivepid->Loop();
//	}
//	else
//	{
//		m_lifter->Loop();
//		m_intake->Loop();
//		m_intake->VisionLoop();
//		if (!m_intake->IsVisioning())
//			m_drivetrain->Loop();
//		m_climber->Loop();
//		m_drivepid->Loop();
//	}

}


void Robot::DisabledInit()
{
	DriverStation::ReportError("DisabledInit");
	lidar->stop();
	if (m_compressor != nullptr)
		m_compressor->Stop();
	m_drivetrain->Stop();
	m_lifter->Stop();
	m_intake->Stop();
	m_climber->Stop();
	m_autonomous->Stop();
	m_drivepid->Stop();
}


void Robot::DisabledPeriodic()
{
//	ReadChooser();
//	m_drivepid->Loop();
//	m_intake->VisionLoop();
}


void Robot::ReadChooser()
{
	m_autoSelected = m_chooser.GetSelected();
	string gamedata = DriverStation::GetInstance().GetGameSpecificMessage();
	if (gamedata.length() < 2)
		gamedata = "   ";

	automode = kAutoDefault;
	if (m_autoSelected == kszAutoCenterSwitch1)
	{
		if (gamedata[0] == 'L')
			automode = kAutoCenterSwitchLeft1;
		else
		if (gamedata[0] == 'R')
			automode = kAutoCenterSwitchRight1;
	}
	else
	if (m_autoSelected == kszAutoCenterSwitch3)
	{
		if (gamedata[0] == 'L')
			automode = kAutoCenterSwitchLeft3;
		else
		if (gamedata[0] == 'R')
			automode = kAutoCenterSwitchRight3;
	}
	else
	if (m_autoSelected == kszAutoLeftScale2X)
	{
		if (gamedata [1] == 'L')
			automode = kAutoLeftScaleLeft2;
		else
		if (gamedata [1] == 'R')
			automode = kAutoLeftScaleRight1;
			//automode = kAutoStraight;
	}
	else
	if (m_autoSelected == kszAutoRightScale2X)
	{
		if (gamedata [1] == 'L')
			automode = kAutoRightScaleLeft1;
			//automode = kAutoStraight;
		else
		if (gamedata [1] == 'R')
			automode = kAutoRightScaleRight2;
	}
	else
	if (m_autoSelected == kszAutoLeftScale1P)
	{
		if (gamedata [1] == 'L')
			//automode = kAutoLeftScaleLeft1;
			automode = kAutoLeftScaleLeft2;
		else
		if (gamedata [1] == 'R')
			//automode = kAutoLeftScaleRight1;
			automode = kAutoStraight;
	}
	else
	if (m_autoSelected == kszAutoRightScale1P)
	{
		if (gamedata [1] == 'L')
			//automode = kAutoRightScaleLeft1;
			automode = kAutoStraight;
		else
		if (gamedata [1] == 'R')
			//automode = kAutoRightScaleRight1;
			automode = kAutoRightScaleRight2;
	}
	else
	if (m_autoSelected == kszAutoLeftSwitch)
	{
		if (gamedata[0] == 'L')
			automode = kAutoStraight;
		else
		if (gamedata[0] == 'R')
			automode = kAutoStraight;
	}
	else
	if (m_autoSelected == kszAutoRightSwitch)
	{
		if (gamedata[0] == 'L')
			automode = kAutoStraight;
		else
		if (gamedata[0] == 'R')
			automode = kAutoStraight;
	}
	else
	if (m_autoSelected == kszAutoTestMode)
		automode = kAutoTest;

	SmartDashboard::PutNumber("AU1_automode", automode);
}


START_ROBOT_CLASS(Robot)
