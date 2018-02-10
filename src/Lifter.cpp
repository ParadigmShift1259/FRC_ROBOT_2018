/*
 * Lifter.cpp
 *
 *  Created on: Jan 20, 2018
 *      Author: Yreffoeg
 */

#include "Lifter.h"
#include "Const.h"


Lifter::Lifter(DriverStation *ds, OperatorInputs *inputs)
{
	m_ds = ds;
	m_inputs = inputs;

	m_motor = nullptr;
	m_solenoid = nullptr;
	m_position = 0;
	m_raisespeed = LIF_RAISESPEED;
	m_lowerspeed = LIF_LOWERSPEED;
	m_liftermin = LIF_LIFTERMIN;
	m_liftermax = LIF_LIFTERMAX;
	m_lifterminspd = LIF_LIFTERMINSPD;
	m_liftermaxspd = LIF_LIFTERMAXSPD;
	m_stage = kIdle;

	if (CAN_LIFTER_MOTOR != -1)
	{
		m_motor = new WPI_TalonSRX(CAN_LIFTER_MOTOR);
		m_motor->Set(ControlMode::PercentOutput, 0);
		m_motor->SetNeutralMode(NeutralMode::Brake);
		m_motor->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
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

	// do initialization for auto mode
	if (m_ds->IsAutonomous())
	{
		m_motor->SetSelectedSensorPosition(LIF_LIFTERSTART, 0, 0);
		m_solenoid->Set(false);
	}

	// do initialization for any mode
	m_motor->StopMotor();
}


void Lifter::Loop()
{
	if ((m_motor == nullptr) || (m_solenoid == nullptr))
		return;

	m_position = m_motor->GetSelectedSensorPosition(0);

	/// if left bumper and Y override position sensor and raise lift
	if (m_inputs->xBoxYButton(OperatorInputs::ToggleChoice::kHold) && m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kHold))
	{
		m_motor->Set(m_raisespeed * 0.5);
	}
	else
	/// if Y raise list only if not at max position
	if (m_inputs->xBoxYButton(OperatorInputs::ToggleChoice::kHold) && (m_position < m_liftermax))		/// raise lifter - positive
	{
		if (m_position > m_liftermaxspd)
			m_motor->Set(m_raisespeed * 0.5);
		else
			m_motor->Set(m_raisespeed);
	}
	else
	/// if left bumper and X override position sensor and lower lift
	if (m_inputs->xBoxXButton(OperatorInputs::ToggleChoice::kHold) && m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kHold))
	{
		m_motor->Set(m_lowerspeed * 0.5);
		m_motor->SetSelectedSensorPosition(0, 0, 0);
	}
	else
	/// if X lower lift only if not at min position
	if (m_inputs->xBoxXButton(OperatorInputs::ToggleChoice::kHold) && (m_position > m_liftermin))		/// lower lifter - negative
	{
		if (m_position < m_lifterminspd)
			m_motor->Set(m_lowerspeed * 0.5);
		else
			m_motor->Set(m_lowerspeed);
	}
	else
	/// stop lift if less than or at min position
	if (m_inputs->xBoxXButton(OperatorInputs::ToggleChoice::kHold) && (m_position <= m_liftermin))
	{
		m_motor->StopMotor();
		m_motor->SetSelectedSensorPosition(0, 0, 0);
	}
	/// no buttons pressed so stop motors
	else
	{
		if (automode == kAutoStage)
			Staging();
		else
			m_motor->StopMotor();
	}

	if (m_inputs->xBoxDPadUp())			/// angle lifter - deploy - true
		m_solenoid->Set(true);
	else
	if (m_inputs->xBoxDPadDown())		/// straighten lifter - retract - false (default)
		m_solenoid->Set(false);

	SmartDashboard::PutNumber("LI1_liftermin", m_liftermin);
	SmartDashboard::PutNumber("LI2_liftermax", m_liftermax);
	SmartDashboard::PutNumber("LI3_position", m_position);
}


void Lifter::TestLoop()
{
	if ((m_motor == nullptr) || (m_solenoid == nullptr))
		return;

	m_position = m_motor->GetSelectedSensorPosition(0);

	if (m_inputs->xBoxYButton(OperatorInputs::ToggleChoice::kHold))		/// raise lifter - positive
	{
		m_motor->Set(m_raisespeed * 0.5);
	}
	else
	if (m_inputs->xBoxXButton(OperatorInputs::ToggleChoice::kHold))		/// lower lifter - negative
	{
		m_motor->Set(m_lowerspeed * 0.5);
	}
	else
	{
		m_motor->StopMotor();
	}

	if (m_inputs->xBoxDPadUp())			/// angle lifter - deploy - true
		m_solenoid->Set(true);
	else
	if (m_inputs->xBoxDPadDown())		/// straighten lifter - retract - false (default)
		m_solenoid->Set(false);

	SmartDashboard::PutNumber("LI1_liftermin", m_liftermin);
	SmartDashboard::PutNumber("LI2_liftermax", m_liftermax);
	SmartDashboard::PutNumber("LI3_position", m_position);
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


bool Lifter::IsBottom()
{
	if (m_position <= m_liftermin)
		return true;
	else
		return false;
}


void Lifter::Staging()
{
	m_position = m_motor->GetSelectedSensorPosition(0);

	switch (m_stage)
	{
	case kIdle:
		if (m_inputs->xBoxStartButton())
		{
			m_motor->SetSelectedSensorPosition(0, 0, 0);
			m_stage = kRaise;
		}
		else
			m_motor->StopMotor();
		break;

	case kRaise:
		if (m_position < LIF_LIFTERSTART)
			m_motor->Set(m_raisespeed * 0.5);
		else
		{
			m_motor->StopMotor();
			m_stage = kStop;
		}
		break;

	case kStop:
		m_motor->StopMotor();
		break;
	}

	SmartDashboard::PutNumber("LI1_liftermin", m_liftermin);
	SmartDashboard::PutNumber("LI2_liftermax", m_liftermax);
	SmartDashboard::PutNumber("LI3_position", m_position);
}
