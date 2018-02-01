/*
 * Autonomous.cpp
 *
 *  Created on: Jan 20, 2018
 *      Author: Jival
 */

#include "Autonomous.h"
#include "Const.h"


Autonomous::Autonomous(OperatorInputs *inputs, DriveTrain *drivetrain)
{
	m_inputs = inputs;
	m_drivetrain = drivetrain;
	m_drivepid = new DrivePID(m_drivetrain, m_inputs);
}


Autonomous::~Autonomous()
{
}


void Autonomous::Init()
{
	m_drivepid->Init(0.0005, 0.0, 0.0, true);
	m_drivepid->SetRelativeAngle(90);
}


void Autonomous::Loop()
{
	m_drivepid->Drive(-0.5);
}


void Autonomous::Stop()
{
	m_drivepid->Stop();
}
