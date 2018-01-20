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
}


Lifter::~Lifter()
{
}


void Lifter::Init()
{
	DriverStation::ReportError("LifterInit");
}


void Lifter::Loop()
{
}


void Lifter::TestLoop()
{
}


void Lifter::Stop()
{
}
