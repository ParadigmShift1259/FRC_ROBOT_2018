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
	if (CAN_LIFTER_MOTOR == -1)
		m_motor = nullptr;
	else
	{
		m_motor = new WPI_TalonSRX(CAN_LIFTER_MOTOR);
		m_motor->Set(ControlMode::PercentOutput, 0);
		m_motor->SetNeutralMode(NeutralMode::Brake);
	}
	m_position = 0;
}


Lifter::~Lifter()
{
	if (m_motor != nullptr)
		delete m_motor;
}


void Lifter::Init()
{
	DriverStation::ReportError("LifterInit");
	if (m_motor != nullptr)
		m_motor->StopMotor();
}


void Lifter::Loop()
{
	if (m_motor == nullptr)
		return;

	m_motor->StopMotor();
}


void Lifter::TestLoop()
{
	if (m_motor == nullptr)
		return;

	m_position = m_motor->GetSelectedSensorPosition(0);

	if (m_inputs->xBoxYButton())
	{
		if (m_position < LIFTER_MAX)
			m_motor->Set(0.5);
		else
			m_motor->StopMotor();
	}
	else
	if (m_inputs->xBoxXButton())
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

	SmartDashboard::PutNumber("L1_position", m_position);
}


void Lifter::Stop()
{
	if (m_motor != nullptr)
		m_motor->StopMotor();
}


void Lifter::ResetPosition()
{
	m_motor->SetSelectedSensorPosition(0, 0, 0);
	m_position = 0;
}
