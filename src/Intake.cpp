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
	m_ingestspeed = INT_INGESTSPEED;
	m_ejectspeed = INT_EJECTSPEED;
	m_allowingest = false;
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
			m_leftmotor->Set(m_ingestspeed);	/// turn on motors to ingest cube
			m_rightmotor->Set(m_ingestspeed * -1.0);
			m_allowingest = false;
			m_stage = kIngestWait;				/// wait for box to ingest
		}
		else
		if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kHold))
		{
			m_leftmotor->Set(m_ingestspeed);	/// turn on motors if button pressed
			m_rightmotor->Set(m_ingestspeed * -1.0);
		}
		else
		{
			m_leftmotor->StopMotor();			/// stop motors if button not pressed
			m_rightmotor->StopMotor();
		}
		break;

	case kIngestWait:
		if (m_timer.HasPeriodPassed(0.5))		/// wait for 500ms
		{
			m_leftmotor->StopMotor();				/// ingestion is complete stop motors
			m_rightmotor->StopMotor();
			m_stage = kBox;							/// we have the box
		}
		else
		{
			m_leftmotor->Set(m_ingestspeed);		/// run the motors to ensure we have the box
			m_rightmotor->Set(m_ingestspeed * -1.0);
		}
		break;

	case kBox:
		if (m_inputs->xBoxAButton())			/// allow ingest motor only when A button released and pressed again
		{
			m_allowingest = true;
			DriverStation::ReportError("A button pressed");
		}
		if (m_allowingest && m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kHold))
		{
			m_leftmotor->Set(m_ingestspeed);	/// turn on motors if button pressed
			m_rightmotor->Set(m_ingestspeed * -1.0);
		}
		else
		if (m_inputs->xBoxBButton())
		{
			m_leftmotor->Set(m_ejectspeed);			/// eject the box
			m_rightmotor->Set(m_ejectspeed * -1.0);
			m_timer.Reset();
			m_stage = kEject;
		}
		else
		{
			m_leftmotor->StopMotor();				/// stop motors until button is pressed
			m_rightmotor->StopMotor();
		}
		break;

	case kEject:
		if (m_timer.HasPeriodPassed(1.0))
		{
			m_solenoid->Set(true);					/// open arms
			m_leftmotor->StopMotor();
			m_rightmotor->StopMotor();
			m_stage = kBottom;						/// go back to beginning (reset loop)
		}
		else
		{
			m_leftmotor->Set(m_ejectspeed);			/// ensure the box is ejected
			m_rightmotor->Set(m_ejectspeed * -1.0);
		}
		break;
	};

	SmartDashboard::PutNumber("IN1_leftmotor", m_leftmotor->GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("IN2_rightmotor", m_rightmotor->GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("IN3_solenoid", m_solenoid->Get());
	SmartDashboard::PutNumber("IN4_cubesensor", m_cubesensor->Get());
	SmartDashboard::PutNumber("IN5_stage", m_stage);
}


void Intake::TestLoop()
{
	if ((m_leftmotor == nullptr) || (m_rightmotor == nullptr) || (m_solenoid == nullptr))
		return;

	if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kHold))		/// ingest cube - positive
	{
		m_leftmotor->Set(m_ingestspeed);
		m_rightmotor->Set(m_ingestspeed * -1.0);
	}
	else
	if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kHold))		/// eject cube - negative
	{
		m_leftmotor->Set(m_ejectspeed);
		m_rightmotor->Set(m_ejectspeed * -1.0);
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

	SmartDashboard::PutNumber("IN1_leftmotor", m_leftmotor->GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("IN2_rightmotor", m_rightmotor->GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("IN3_solenoid", m_solenoid->Get());
	SmartDashboard::PutNumber("IN4_cubesensor", m_cubesensor->Get());
	SmartDashboard::PutNumber("IN5_stage", m_stage);
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

	m_leftmotor->SetSelectedSensorPosition(0, 0, 0);
	m_rightmotor->SetSelectedSensorPosition(0, 0, 0);
}
