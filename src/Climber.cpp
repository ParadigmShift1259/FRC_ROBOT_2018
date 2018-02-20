/*
 * Climber.cpp
 *
 *  Created on: Jan 20, 2018
 *      Author: Jival
 */

#include "Climber.h"
#include "Const.h"


Climber::Climber(OperatorInputs *inputs)
{
	m_motor = nullptr;

	m_inputs = inputs;

	if (CAN_CLIMBER_MOTOR != -1)
	{
		m_motor = new WPI_TalonSRX(CAN_CLIMBER_MOTOR);
		m_motor->Set(ControlMode::PercentOutput, 0);
		m_motor->SetNeutralMode(NeutralMode::Brake);
	}
}


Climber::~Climber()
{
	if (m_motor != nullptr)
		delete m_motor;
}


void Climber::Init()
{
	if (m_motor == nullptr)
		return;

	DriverStation::ReportError("ClimberInit");

	m_motor->StopMotor();
}


void Climber::Loop()
{
	if (m_motor == nullptr)
		return;

	if (m_inputs->xBoxStartButton(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL) && m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL))		/// initiate climb - positive
		m_motor->Set(-1.0);
	else if (m_inputs->xBoxStartButton(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL))
		m_motor->Set(1.0);
	else
		m_motor->StopMotor();
}


void Climber::TestLoop()
{
	if (m_motor == nullptr)
		return;

	if (m_inputs->xBoxStartButton(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL) && m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL))		/// initiate climb - positive
		m_motor->Set(-0.5);
	else if (m_inputs->xBoxStartButton(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL))
		m_motor->Set(0.5);
	else
		m_motor->StopMotor();
}


void Climber::Stop()
{
	if (m_motor == nullptr)
		return;

	m_motor->StopMotor();
}
