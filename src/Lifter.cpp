/*
 * Lifter.cpp
 *
 *  Created on: Jan 20, 2018
 *      Author: Yreffoeg
 */

#include "Lifter.h"
#include "Const.h"


Lifter::Lifter(OperatorInputs *inputs)
{
	m_inputs = inputs;

	m_motor = nullptr;
	m_solenoid = nullptr;
	m_position = 0;

	if (CAN_LIFTER_MOTOR != -1)
	{
		m_motor = new WPI_TalonSRX(CAN_LIFTER_MOTOR);
		m_motor->Set(ControlMode::PercentOutput, 0);
		m_motor->SetNeutralMode(NeutralMode::Brake);
	}

	if (PCM_LIFTER_SOLENOID != -1)
		m_solenoid = new Solenoid(PCM_LIFTER_MODULE, PCM_LIFTER_SOLENOID);
}


Lifter::~Lifter()
{
	if (m_motor != nullptr)
		delete m_motor;
	if (m_solenoid != nullptr)
		delete m_solenoid;
}


void Lifter::Init()
{
	if ((m_motor == nullptr) || (m_solenoid == nullptr))
		return;

	DriverStation::ReportError("LifterInit");

	m_motor->StopMotor();
	m_solenoid->Set(false);
}


void Lifter::Loop()
{
	if ((m_motor == nullptr) || (m_solenoid == nullptr))
		return;

	m_motor->StopMotor();
}


void Lifter::TestLoop()
{
	if ((m_motor == nullptr) || (m_solenoid == nullptr))
		return;

	m_position = m_motor->GetSelectedSensorPosition(0);

	if (m_inputs->xBoxYButton(OperatorInputs::ToggleChoice::kHold))		/// raise lifter - positive
	{
		if (m_position < LIFTER_MAX)
			m_motor->Set(0.5);
		else
			m_motor->StopMotor();
	}
	else
	if (m_inputs->xBoxXButton(OperatorInputs::ToggleChoice::kHold))		/// lower lifter - negative
	{
		if (m_position > LIFTER_MIN)
			m_motor->Set(-0.5);
		else
			m_motor->StopMotor();
	}
	else
	{
		m_motor->StopMotor();
	}

	if (m_inputs->xBoxDPadUp())		/// angle lifter - deploy - true
		m_solenoid->Set(true);
	else
	if (m_inputs->xBoxDPadDown())		/// straighten lifter - retract - false (default)
		m_solenoid->Set(false);

	SmartDashboard::PutNumber("L1_position", m_position);
}


void Lifter::Stop()
{
	if ((m_motor == nullptr) || (m_solenoid == nullptr))
		return;

	m_motor->StopMotor();
}


void Lifter::ResetPosition()
{
	if ((m_motor == nullptr) || (m_solenoid == nullptr))
		return;

	m_motor->SetSelectedSensorPosition(0, 0, 0);
	m_position = 0;
}
