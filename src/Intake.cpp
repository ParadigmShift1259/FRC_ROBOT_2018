/*
 * Intake.cpp
 *
 *  Created on: Jan 20, 2018
 *      Author: Jival
 */

#include <Intake.h>
#include "Const.h"


Intake::Intake(DriverStation *ds, OperatorInputs *inputs, Lifter *lifter, DrivePID *drivepid)
{
	m_leftmotor = nullptr;
	m_rightmotor = nullptr;
	m_solenoid = nullptr;

	m_ds = ds;
	m_inputs = inputs;
	m_lifter = lifter;
	m_drivepid = drivepid;

	if ((CAN_INTAKE_LEFTMOTOR != -1) && (CAN_INTAKE_RIGHTMOTOR != -1))
	{
		m_leftmotor = new WPI_TalonSRX(CAN_INTAKE_LEFTMOTOR);
		m_leftmotor->Set(ControlMode::PercentOutput, 0);
		m_leftmotor->SetNeutralMode(NeutralMode::Brake);

		m_rightmotor = new WPI_TalonSRX(CAN_INTAKE_RIGHTMOTOR);
		m_rightmotor->Set(ControlMode::PercentOutput, 0);
		m_rightmotor->SetNeutralMode(NeutralMode::Brake);
	}

	if (PCM_INTAKE_SOLENOID != -1)
		m_solenoid = new Solenoid(PCM_INTAKE_MODULE, PCM_INTAKE_SOLENOID);

	m_cubesensor = new DigitalInput(DIO_INTAKE_CUBESENSOR);

	m_stage = kIngest;
	m_ingestspeed = INT_INGESTSPEED;
	m_ejectspeed = INT_EJECTHIGH;
	m_allowingest = false;
	m_autoingest = false;
	m_visioning = kIdle;

	m_nettable = NetworkTableInstance::GetDefault().GetTable("OpenCV");
	m_counter = 0;
}


Intake::~Intake()
{
	if (m_leftmotor != nullptr)
		delete m_leftmotor;
	if (m_rightmotor != nullptr)
		delete m_rightmotor;
	if (m_solenoid != nullptr)
		delete m_solenoid;
}


void Intake::Init()
{
	if ((m_leftmotor == nullptr) || (m_rightmotor == nullptr) || (m_solenoid == nullptr))
		return;

	DriverStation::ReportError("IntakeInit");

	m_leftmotor->StopMotor();
	m_rightmotor->StopMotor();
	m_solenoid->Set(false);
	m_stage = kIngest;
	m_timer.Reset();
	m_timer.Start();
	m_allowingest = false;
	m_autoingest = false;
	m_counter = 0;
}


void Intake::Loop()
{
	if ((m_leftmotor == nullptr) || (m_rightmotor == nullptr) || (m_solenoid == nullptr))
		return;

	/// check A Button to record state of button toggle (used in kBox)
	bool xboxabuttontoggle = m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL);

	switch (m_stage)
	{
	case kBottom:
		m_leftmotor->StopMotor();				/// motors are off by default
		m_rightmotor->StopMotor();
		if (m_lifter->IsBottom())				/// check for lifter to be on the bottom
		{
			m_solenoid->Set(true);					/// open intake arms
			m_stage = kIngest;						/// lifter is at bottom, go to ingest stage
		}
		break;

	case kIngest:
		if (xboxabuttontoggle)
			m_autoingest = true;
		if (m_cubesensor->Get() || m_inputs->xBoxBackButton() || m_inputs->xBoxBackButton(OperatorInputs::kToggle, 1))
		{
			m_solenoid->Set(false);					/// we have cube, close intake arms
			m_timer.Reset();
			m_leftmotor->Set(m_ingestspeed);		/// turn on motors to ingest cube
			m_rightmotor->Set(m_ingestspeed * -1.0);
			m_allowingest = false;
			m_autoingest = false;
			m_visioning = kIdle;
			m_stage = kIngestWait;					/// wait for box to ingest
		}
		else
		if (m_autoingest /*m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kHold)*/)
		{
			m_solenoid->Set(true);					/// open intake arms
			m_leftmotor->Set(m_ingestspeed);		/// turn on motors if button pressed
			m_rightmotor->Set(m_ingestspeed * -1.0);
		}
		else
		{
			m_solenoid->Set(true);					/// open intake arms
			m_leftmotor->StopMotor();				/// stop motors if button not pressed
			m_rightmotor->StopMotor();
		}
		break;

	case kIngestWait:
		if (m_timer.Get() > 0.2)		/// wait for 200ms
		{
			m_leftmotor->StopMotor();				/// ingestion is complete stop motors
			m_rightmotor->StopMotor();
			if (m_lifter->MoveSmidgeUp())
				m_stage = kBox;							/// we have the box
		}
		else
		{
			m_leftmotor->Set(m_ingestspeed);		/// run the motors to ensure we have the box
			m_rightmotor->Set(m_ingestspeed * -1.0);
		}
		break;

	case kBox:
		if (xboxabuttontoggle)					/// allow ingest motor only when A button released and pressed again
			m_allowingest = true;
		if (m_allowingest && m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL))
		{
			m_leftmotor->Set(m_ingestspeed);		/// turn on motors if button pressed
			m_rightmotor->Set(m_ingestspeed * -1.0);
		}
		else
		if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL) && m_inputs->xBoxLeftBumper(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL))
		{
			m_ejectspeed = INT_EJECTLOW;			/// eject the box low speed mode
			m_leftmotor->Set(m_ejectspeed);
			m_rightmotor->Set(m_ejectspeed * -1.0);
			m_timer.Reset();
			m_stage = kEject;
		}
		else
		if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL))
		{
			m_leftmotor->Set(m_ejectspeed);			/// eject the box
			m_rightmotor->Set(m_ejectspeed * -1.0);
			m_timer.Reset();
			m_stage = kEject;
		}
		else
		{
			m_leftmotor->StopMotor();				/// stop motors until button is pressed
			m_rightmotor->StopMotor();
		}
		break;

	case kEject:
		if (m_timer.HasPeriodPassed(1.0))
		{
			m_solenoid->Set(true);					/// open arms
			m_leftmotor->StopMotor();
			m_rightmotor->StopMotor();
			m_ejectspeed = INT_EJECTHIGH;
			m_stage = kIngest;						/// go back to beginning (reset loop)
		}
		else
		{
			m_leftmotor->Set(m_ejectspeed);			/// ensure the box is ejected
			m_rightmotor->Set(m_ejectspeed * -1.0);
		}
		break;
	};

	SmartDashboard::PutNumber("IN1_leftmotor", m_leftmotor->GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("IN2_rightmotor", m_rightmotor->GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("IN3_solenoid", m_solenoid->Get());
	SmartDashboard::PutNumber("IN4_cubesensor", m_cubesensor->Get());
	SmartDashboard::PutNumber("IN5_stage", m_stage);
}


void Intake::VisionLoop()
{
	int counter = m_nettable->GetNumber("visioncounter", 0);
	double angle = m_nettable->GetNumber("XOffAngle", 0) * -1;
	double distance = m_nettable->GetNumber("Forward_Distance_Inch", 0);
	bool valid = false;

	double scale = distance / (96 * 2) + 0.25;
	if (counter > m_counter)
	{
		m_counter = counter;
		if (distance > 0.0)
			valid = true;
	}

	switch (m_visioning)
	{
	case kIdle:
		if (valid && m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kToggle, 0 * INP_DUAL))
		{
			m_drivepid->Init(m_pid[0], m_pid[1], m_pid[2], DrivePID::Feedback::kGyro);
			m_drivepid->EnablePID();
			m_visioning = kVision;
			m_autoingest = true;
		}
		else
			m_drivepid->DisablePID();
		m_counter = 0;
		break;

	case kVision:
		if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kToggle, 0 * INP_DUAL))
		{
			m_drivepid->DisablePID();
			m_autoingest = false;
			m_visioning = kIdle;
		}
		else
		{
			//double x = m_inputs->xBoxLeftX(0 * INP_DUAL) * 90;
			//m_drivepid->SetAbsoluteAngle(x);

			double y = m_inputs->xBoxLeftY(0 * INP_DUAL) * (scale > 1 ? 1 : scale);

			m_drivepid->Drive(y, true);
			m_drivepid->ResetGyro();
			m_drivepid->SetAbsoluteAngle(angle);
		}
		break;
	}
	SmartDashboard::PutNumber("IN999_scale", scale);
	SmartDashboard::PutNumber("IN6_visioncounter", counter);
	SmartDashboard::PutNumber("IN7_visionangle", angle);
	SmartDashboard::PutNumber("IN8_distance", distance);
}


void Intake::AutoLoop()
{
	switch (m_stage)
	{
	case kBottom:
	case kIngest:
	case kIngestWait:
		m_leftmotor->StopMotor();				/// stop motors until auto start
		m_rightmotor->StopMotor();
		break;

	case kBox:
		//m_ejectspeed = INT_EJECTLOW;			/// eject the box low speed mode
		m_leftmotor->Set(m_ejectspeed);
		m_rightmotor->Set(m_ejectspeed * -1.0);
		m_timer.Reset();
		m_stage = kEject;
		break;

	case kEject:
		if (m_timer.HasPeriodPassed(1.0))
		{
			m_solenoid->Set(true);					/// open arms
			m_leftmotor->StopMotor();
			m_rightmotor->StopMotor();
			//m_ejectspeed = INT_EJECTHIGH;
			m_stage = kIngest;						/// go back to beginning (reset loop)
		}
		else
		{
			m_leftmotor->Set(m_ejectspeed);			/// ensure the box is ejected
			m_rightmotor->Set(m_ejectspeed * -1.0);
		}
		break;
	};
}


void Intake::TestLoop()
{
	if ((m_leftmotor == nullptr) || (m_rightmotor == nullptr) || (m_solenoid == nullptr))
		return;

	if (m_inputs->xBoxAButton(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL))		/// ingest cube - positive
	{
		m_leftmotor->Set(m_ingestspeed);
		m_rightmotor->Set(m_ingestspeed * -1.0);
	}
	else
	if (m_inputs->xBoxBButton(OperatorInputs::ToggleChoice::kHold, 1 * INP_DUAL))		/// eject cube - negative
	{
		m_leftmotor->Set(m_ejectspeed);
		m_rightmotor->Set(m_ejectspeed * -1.0);
	}
	else
	{
		m_leftmotor->StopMotor();
		m_rightmotor->StopMotor();
	}

	if (m_inputs->xBoxDPadLeft(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))		/// open intake - deploy - true
		m_solenoid->Set(true);
	else
	if (m_inputs->xBoxDPadRight(OperatorInputs::ToggleChoice::kToggle, 1 * INP_DUAL))		/// close intake - retract - false (default)
		m_solenoid->Set(false);

	SmartDashboard::PutNumber("IN1_leftmotor", m_leftmotor->GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("IN2_rightmotor", m_rightmotor->GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("IN3_solenoid", m_solenoid->Get());
	SmartDashboard::PutNumber("IN4_cubesensor", m_cubesensor->Get());
	SmartDashboard::PutNumber("IN5_stage", m_stage);
}


void Intake::Stop()
{
	if ((m_leftmotor == nullptr) || (m_rightmotor == nullptr) || (m_solenoid == nullptr))
		return;

	m_leftmotor->StopMotor();
	m_rightmotor->StopMotor();
	m_timer.Stop();
}


void Intake::ResetPosition()
{
	if ((m_leftmotor == nullptr) || (m_rightmotor == nullptr) || (m_solenoid == nullptr))
		return;

	m_leftmotor->SetSelectedSensorPosition(0, 0, 0);
	m_rightmotor->SetSelectedSensorPosition(0, 0, 0);
}


void Intake::AutoEject()
{
	m_stage = kBox;
}


bool Intake::IsVisioning()
{
	if (m_visioning == kIdle)
		return false;
	return true;
}
