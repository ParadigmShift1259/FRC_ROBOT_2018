/*
 * Grabber.cpp
 *
 *  Created on: Jan 20, 2018
 *      Author: Jival
 */

#include "Grabber.h"
#include "Const.h"



Grabber::Grabber(OperatorInputs *inputs)
{
	m_inputs = inputs;
	if ((CAN_GRABBER_LEFTMOTOR == -1) || (CAN_GRABBER_RIGHTMOTOR == -1))
	{
		m_leftmotor = nullptr;
		m_rightmotor = nullptr;
	}
	else
	{
		m_leftmotor = new WPI_TalonSRX(CAN_GRABBER_LEFTMOTOR);
		m_leftmotor->Set(ControlMode::PercentOutput, 0);
		m_leftmotor->SetNeutralMode(NeutralMode::Brake);

		m_rightmotor = new WPI_TalonSRX(CAN_GRABBER_RIGHTMOTOR);
		m_rightmotor->Set(ControlMode::PercentOutput, 0);
		m_rightmotor->SetNeutralMode(NeutralMode::Brake);
	}
	m_leftposition = 0;
	m_rightposition = 0;
}



Grabber::~Grabber()
{
	if (m_leftmotor != nullptr)
		delete m_leftmotor;

	if (m_rightmotor != nullptr)
		delete m_rightmotor;
}


void Grabber::Init()
{
	DriverStation::ReportError("GrabberInit");

	if (m_leftmotor != nullptr)
		m_leftmotor->StopMotor();
	if (m_rightmotor != nullptr)
		m_rightmotor->StopMotor();

}


void Grabber::Loop()
{
	if ((m_leftmotor == nullptr) || (m_rightmotor == nullptr))
		return;

	m_leftmotor->StopMotor();
	m_rightmotor->StopMotor();
}


void Grabber::TestLoop()
{
	if ((m_leftmotor == nullptr) || (m_rightmotor == nullptr))
		return;

	m_leftposition = m_leftmotor->GetSelectedSensorPosition(0);
	m_rightposition = m_rightmotor->GetSelectedSensorPosition(0);

	if (m_inputs->xBoxBButton())
	{
		m_leftmotor->Set(0.5);
		m_rightmotor->Set(0.5);
	}
	else
	if (m_inputs->xBoxAButton())
	{
			m_leftmotor->Set(-0.5);
			m_rightmotor->Set(-0.5);
	}
	else
	{
		m_leftmotor->StopMotor();
		m_rightmotor->StopMotor();
	}
	SmartDashboard::PutNumber("L1_left_position", m_leftposition);
	SmartDashboard::PutNumber("L2_right_position", m_rightposition);
}


void Grabber::Stop()
{
	if (m_leftmotor != nullptr)
		m_leftmotor->StopMotor();
	if (m_rightmotor != nullptr)
		m_rightmotor->StopMotor();
}


void Grabber::ResetPosition()
{
	m_leftposition = 0;
	m_rightposition = 0;
	m_leftmotor->SetSelectedSensorPosition(0, 0, 0);
	m_rightmotor->SetSelectedSensorPosition(0, 0, 0);
}
