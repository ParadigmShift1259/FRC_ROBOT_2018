/*
 * Intake.h
 *
 *  Created on: Jan 20, 2018
 *      Author: Jival
 */

#ifndef SRC_Intake_H_
#define SRC_Intake_H_


#include <WPILib.h>
#include <networktables\NetworkTable.h>
#include <networktables\NetworkTableInstance.h>
#include <ctre\Phoenix.h>
#include "OperatorInputs.h"
#include "Lifter.h"
#include "DrivePID.h"


using namespace nt;


class Intake
{
public:
	enum Stage {kBottom, kIngest, kIngestWait, kBox, kFix, kEject};
	enum Vision {kIdle, kVision};

	Intake(DriverStation *ds, OperatorInputs *inputs, Lifter *lifter, DrivePID *drivepid);
	virtual ~Intake();
	void Init();
	void Loop();
	void VisionLoop();
	void AutoLoop();
	void TestLoop();
	void Stop();
	void ResetPosition();
	void AutoEject();
	void AutoIngest();
	bool IsVisioning();

protected:
	DriverStation *m_ds;
	OperatorInputs *m_inputs;
	Lifter *m_lifter;
	DrivePID *m_drivepid;
	WPI_TalonSRX *m_leftmotor;
	WPI_TalonSRX *m_rightmotor;
	Solenoid *m_solenoid;
	DigitalInput *m_cubesensor;
	Stage m_stage;
	Timer m_timer;

	double m_ingestspeed;
	double m_ejectspeed;
	bool m_allowingest;
	bool m_autoingest;
	int m_fixstage;
	Vision m_visioning;

	//double m_pid[3] = {0.02, 0.01, 0.1};
	//double m_pid[3] = {0.009, 0.0005, 0.07};
	double m_pid[3] = {0.015, 0.0, 0.0};
	shared_ptr<NetworkTable> m_nettable;
	int m_counter;
	Timer m_visiontimer;
	bool m_visionvalid;
	bool m_auto;
};


#endif /* SRC_Intake_H_ */
