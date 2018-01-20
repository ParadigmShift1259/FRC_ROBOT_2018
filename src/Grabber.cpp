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
}


Grabber::~Grabber()
{
}


void Grabber::Init()
{
	DriverStation::ReportError("GrabberInit");
}


void Grabber::Loop()
{
}


void Grabber::TestLoop()
{
}


void Grabber::Stop()
{
}
