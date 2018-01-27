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
	m_operatorinputs = new OperatorInputs();

	m_driverstation = &DriverStation::GetInstance();

	m_drivetrain = new DriveTrain(DriveTrain::DriveMode::kFollower, m_operatorinputs);

	m_motionprofile = new MotionProfiling(m_drivetrain->RightTalonLead(), m_drivetrain->RightTalonFollow(), m_drivetrain->LeftTalonLead(), m_drivetrain->LeftTalonFollow());

	m_compressor = new Compressor(PCM_COMPRESSOR_SOLENOID);
	m_lifter = new Lifter(m_operatorinputs);
	m_grabber = new Grabber(m_operatorinputs);
	m_climber = new Climber(m_operatorinputs);
}


void Robot::AutonomousInit()
{
	m_motionprofile->Init();
}


void Robot::AutonomousPeriodic()
{
	m_motionprofile->Loop();
}


void Robot::TestInit()
{
/*
	DriverStation::ReportError("TestInit");
	m_compressor->Start();
	m_drivetrain->Init();
	m_lifter->Init();
	m_grabber->Init();
	m_climber->Init();
*/
}


void Robot::TestPeriodic()
{
/*
	m_drivetrain->Loop();
	m_lifter->Loop();
	m_grabber->Loop();
	m_climber->Loop();
*/
}


void Robot::TeleopInit()
{
	DriverStation::ReportError("TeleopInit");

	m_compressor->Start();
	m_drivetrain->Init();
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
	m_drivetrain->Stop();
	m_lifter->Stop();
	m_grabber->Stop();
	m_climber->Stop();
}


START_ROBOT_CLASS(Robot)
