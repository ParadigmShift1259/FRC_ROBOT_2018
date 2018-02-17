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
	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

	m_driverstation = &DriverStation::GetInstance();
	m_operatorinputs = new OperatorInputs();
	m_drivetrain = new DriveTrain(m_operatorinputs);
	m_compressor = nullptr;
	if (PCM_COMPRESSOR_SOLENOID != -1)
		m_compressor = new Compressor(PCM_COMPRESSOR_SOLENOID);
	m_lifter = new Lifter(m_driverstation, m_operatorinputs);
	m_intake = new Intake(m_driverstation, m_operatorinputs, m_lifter);
	m_climber = new Climber(m_operatorinputs);
	m_pigeon = new PigeonIMU(0);
	m_gyroval[0] = 0;
	m_gyroval[1] = 0;
	m_gyroval[2] = 0;
	m_drivepid = new DrivePID(m_drivetrain, m_pigeon, m_operatorinputs);
	m_autonomous = new Autonomous(m_operatorinputs, m_drivetrain, m_drivepid);
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

	if (m_compressor != nullptr)
		m_compressor->Start();

	m_drivetrain->Init(DriveTrain::DriveMode::kFollower);
	m_lifter->Init();
	m_intake->Init();
	m_climber->Init();
	m_autonomous->Init();
}


void Robot::AutonomousPeriodic()
{
//	m_drivetrain->Loop();
	m_lifter->Loop();
	m_intake->Loop();
	m_climber->Loop();
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
	if (automode == kAutoTest)
		DriverStation::ReportError("TeleopInit Test Mode");
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
	double ypr[3] = {0, 0, 0};
	if (automode == kAutoTest)
	{
		/*if(m_operatorinputs->xBoxRightTrigger(OperatorInputs::ToggleChoice::kToggle,0))
			m_turn = kInit;
		switch(m_turn)
		{
		case kInit:
			m_drivepid->Enable();
			m_drivepid->Init(0.009, 0.0005, 0.07, true);
			m_drivepid->SetRelativeAngle(90);
			m_turn = kTurning;
		case kTurning:
			m_drivepid->Drive(0,false);
			SmartDashboard::PutNumber("DriveAngle Setpoint",m_drivepid->GetSetpoint());
			if(m_drivepid->OnTarget())
				m_turn = kIdle;
			break;
		case kIdle:
		default:
			m_drivepid->Stop();
			m_drivetrain->Loop();
		}*/
		m_lifter->TestLoop();
		m_intake->TestLoop();
		m_climber->TestLoop();
		m_pigeon->GetAccumGyro(m_gyroval);
		m_pigeon->GetYawPitchRoll(ypr);
		SmartDashboard::PutNumber("Gyrox", m_gyroval[0]);
		SmartDashboard::PutNumber("Gyroy", m_gyroval[1]);
		SmartDashboard::PutNumber("Gyroz", m_gyroval[2]);
		SmartDashboard::PutNumber("GyroFused",m_pigeon->GetFusedHeading());
		SmartDashboard::PutNumber("GyroYaw",ypr[0]);
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
	DriverStation::ReportError("DisabledInit");

	if (m_compressor != nullptr)
		m_compressor->Stop();
	m_drivetrain->Stop();
	m_lifter->Stop();
	m_intake->Stop();
	m_climber->Stop();
	m_autonomous->Stop();
	m_gyroval[0] = 0;
	m_gyroval[1] = 0;
	m_gyroval[2] = 0;
	m_pigeon->SetFusedHeading(0,0);
	m_pigeon->SetYaw(0,0);

}


void Robot::DisabledPeriodic()
{
	//DriverStation::ReportError("DisabledPeriodic");

	m_autoSelected = m_chooser.GetSelected();

	if (m_autoSelected == kAutoAutoMode)
		automode = kAutoAuto;
	else
	if (m_autoSelected == kAutoTestMode)
		automode = kAutoTest;

	SmartDashboard::PutNumber("AU1_automode", automode);
}


START_ROBOT_CLASS(Robot)
