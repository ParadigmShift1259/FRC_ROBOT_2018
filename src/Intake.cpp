/*
 * Intake.cpp
 *
 *  Created on: Jan 20, 2018
 *      Author: Jival
 */

#include <Intake.h>
#include "Const.h"



Intake::Intake(OperatorInputs *inputs, Lifter *lifter)
{
	m_leftmotor = nullptr;
	m_rightmotor = nullptr;
	m_solenoid = nullptr;
	m_leftposition = 0;
	m_rightposition = 0;

	m_inputs = inputs;
	m_lifter = lifter;

	if ((CAN_INTAKE_LEFTMOTOR != -1) && (CAN_INTAKE_RIGHTMOTOR != -1))
	{
		m_leftmotor = new WPI_TalonSRX(CAN_INTAKE_LEFTMOTOR);
		m_leftmotor->Set(ControlMode::PercentOutput, 0);
		m_leftmotor->SetNeutralMode(NeutralMode::Brake);

		m_rightmotor = new WPI_TalonSRX(CAN_INTAKE_RIGHTMOTOR);
		m_rightmotor->Set(ControlMode::PercentOutput, 0);
		m_rightmotor->SetNeutralMode(NeutralMode::Brake);

	}

	if (PCM_INTAKE_SOLENOID != -1)
		m_solenoid = new Solenoid(PCM_INTAKE_MODULE, PCM_INTAKE_SOLENOID);

	m_cubesensor = new DigitalInput(DIO_INTAKE_CUBESENSOR);

	m_stage = kBottom;
}


Intake::~Intake()
{
	if (m_leftmotor != nullptr)
		delete m_leftmotor;
	if (m_rightmotor != nullptr)
		delete m_rightmotor;
	if (m_solenoid != nullptr)
		delete m_solenoid;
}


void Intake::Init()
{
	if ((m_leftmotor == nullptr) || (m_rightmotor == nullptr) || (m_solenoid == nullptr))
		return;

	DriverStation::ReportError("IntakeInit");

	m_leftmotor->StopMotor();
	m_rightmotor->StopMotor();
	m_solenoid->Set(false);
	m_timer.Reset();
	m_timer.Start();

}


void Intake::Loop()
{
	if ((m_leftmotor == nullptr) || (m_rightmotor == nullptr) || (m_solenoid == nullptr))
		return;

	const double MOTOR_SPEED = 0.5;

	switch (m_stage)
	{
	case kBottom:
		m_leftmotor->StopMotor();				/// motors are off by default
		m_rightmotor->StopMotor();
		if (m_lifter->IsBottom())				/// check for lifter to be on the bottom
		{
			m_solenoid->Set(true);					/// open intake arms
			m_stage = kIngest;						/// lifter is at bottom, go to ingest stage
		}
		break;

	case kIngest:
		if (m_cubesensor->Get() || m_inputs->xBoxBackButton())
		{
			m_solenoid->Set(false);				/// we have cube, close intake arms
			m_timer.Reset();
			m_leftmotor->Set(MOTOR_SPEED);
			m_rightmotor->Set(MOTOR_SPEED);
			m_stage = kIngestWait;
		}
		else
		if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kHold))
		{
			m_leftmotor->Set(MOTOR_SPEED);
			m_rightmotor->Set(MOTOR_SPEED);
		}
		else
		{
			m_leftmotor->StopMotor();
			m_rightmotor->StopMotor();
		}
		break;

	case kIngestWait:
		if (m_timer.HasPeriodPassed(0.2))
		{
			m_leftmotor->StopMotor();
			m_rightmotor->StopMotor();
			m_stage = kBox;
		}
		else
		{
			m_leftmotor->Set(MOTOR_SPEED);
			m_rightmotor->Set(MOTOR_SPEED);
		}
		break;

	case kBox:
		if (m_inputs->xBoxBButton())
		{
			m_leftmotor->Set(MOTOR_SPEED * -1);
			m_rightmotor->Set(MOTOR_SPEED * -1);
			m_timer.Reset();
			m_stage = kEject;
		}
		else
		{
			m_leftmotor->StopMotor();
			m_rightmotor->StopMotor();
		}
		break;

	case kEject:
		if (m_timer.HasPeriodPassed(1.0))
		{
			m_solenoid->Set(true);
			m_leftmotor->StopMotor();
			m_rightmotor->StopMotor();
			m_stage = kBottom;
		}
		else
		{
			m_leftmotor->Set(MOTOR_SPEED * -1);
			m_rightmotor->Set(MOTOR_SPEED * -1);
		}
		break;
	};

	SmartDashboard::PutNumber("LI_cubesensor", m_cubesensor->Get());
}


void Intake::TestLoop()
{
	if ((m_leftmotor == nullptr) || (m_rightmotor == nullptr) || (m_solenoid == nullptr))
		return;

	m_leftposition = m_leftmotor->GetSelectedSensorPosition(0);
	m_rightposition = m_rightmotor->GetSelectedSensorPosition(0);

	if (!m_cubesensor->Get() && m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kHold))		/// ingest cube - positive
	{
		m_leftmotor->Set(0.5);
		m_rightmotor->Set(0.5);
	}
	else
	if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kHold))		/// eject cube - negative
	{
		m_leftmotor->Set(-0.5);
		m_rightmotor->Set(-0.5);
	}
	else
	{
		m_leftmotor->StopMotor();
		m_rightmotor->StopMotor();
	}

	if (m_inputs->xBoxDPadLeft())		/// open intake - deploy - true
		m_solenoid->Set(true);
	else
	if (m_inputs->xBoxDPadRight())		/// close intake - retract - false (default)
		m_solenoid->Set(false);

	SmartDashboard::PutNumber("L1_left_position", m_leftposition);
	SmartDashboard::PutNumber("L2_right_position", m_rightposition);
	SmartDashboard::PutNumber("Li_cube_sensor", m_cubesensor->Get());
}


void Intake::Stop()
{
	if ((m_leftmotor == nullptr) || (m_rightmotor == nullptr) || (m_solenoid == nullptr))
		return;

	m_leftmotor->StopMotor();
	m_rightmotor->StopMotor();
	m_timer.Stop();
}


void Intake::ResetPosition()
{
	if ((m_leftmotor == nullptr) || (m_rightmotor == nullptr) || (m_solenoid == nullptr))
		return;

	m_leftposition = 0;
	m_rightposition = 0;
	m_leftmotor->SetSelectedSensorPosition(0, 0, 0);
	m_rightmotor->SetSelectedSensorPosition(0, 0, 0);
}
