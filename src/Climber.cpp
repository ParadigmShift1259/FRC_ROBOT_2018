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
	m_inputs = inputs;

}


Climber::~Climber()
{
}


void Climber::Init()
{
	DriverStation::ReportError("ClimberInit");
}


void Climber::Loop()
{
}


void Climber::TestLoop()
{
}


void Climber::Stop()
{
}


