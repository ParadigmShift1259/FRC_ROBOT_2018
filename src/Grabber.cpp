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

	m_leftmotor->StopMotor();
	m_rightmotor->StopMotor();
}


void Grabber::Stop()
{
	if (m_leftmotor != nullptr)
		m_leftmotor->StopMotor();
	if (m_rightmotor != nullptr)
		m_rightmotor->StopMotor();
}
