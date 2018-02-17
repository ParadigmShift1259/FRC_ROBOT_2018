/*
 * VisionTarget.cpp
 *
 *  Created on: Feb 13, 2018
 *      Author: Developer
 */

#include <VisionTarget.h>

VisionTarget::VisionTarget()
{
	m_nettableinst = NetworkTableInstance::GetDefault();
	m_nettable = m_nettableinst.GetTable("OpenCV");
	m_robocounter=0;
}


void VisionTarget::Init()
{
	m_robocounter=0;
	m_nettable->Delete("visioncounter");
}


void VisionTarget::Loop()
{
	m_robocounter++;
	SmartDashboard::PutNumber("OpenCV Counter",m_nettable->GetNumber("visioncounter",0));
    SmartDashboard::PutNumber("RoboRIO Counter",m_robocounter);
    m_nettable->PutNumber("RoboCounter",m_robocounter);
}


void VisionTarget::Stop()
{

}

VisionTarget::~VisionTarget() {

}

