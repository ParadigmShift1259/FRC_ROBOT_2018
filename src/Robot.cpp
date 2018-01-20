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

	m_rightFrontTalon = new WPI_TalonSRX(1);
	m_rightBackTalon = new WPI_TalonSRX(3);
	m_leftFrontTalon = new WPI_TalonSRX(0);
	m_leftBackTalon = new WPI_TalonSRX(2);

	m_operatorinputs = new OperatorInputs();

	m_motionmagic = new MotionMagic(m_rightFrontTalon, m_rightBackTalon, m_leftFrontTalon, m_leftBackTalon);
	m_driverstation = &DriverStation::GetInstance();
	m_drivetrain = new DriveTrain(DriveTrain::DriveMode::kTank, m_operatorinputs, m_driverstation);
	m_compressor = new Compressor(PCM_COMPRESSOR_SOLENOID);
}


void Robot::AutonomousInit()
{
	m_motionmagic->Init();
}


void Robot::AutonomousPeriodic()
{
	m_motionmagic->Loop(10000);//Arbitrary
}


void Robot::TestInit()
{
	DriverStation::ReportError("TestInit");
	m_compressor->Start();
	m_drivetrain->Init();
}


void Robot::TestPeriodic()
{
	m_drivetrain->Loop();
}


void Robot::TeleopInit()
{
	DriverStation::ReportError("TeleopInit");
	m_compressor->Start();
	m_drivetrain->Init();
}


void Robot::TeleopPeriodic()
{
	m_drivetrain->Loop();
}


void Robot::DisabledInit()
{
	m_drivetrain->Stop();
}


START_ROBOT_CLASS(Robot)
