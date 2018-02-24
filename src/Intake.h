/*
 * Intake.h
 *
 *  Created on: Jan 20, 2018
 *      Author: Jival
 */

#ifndef SRC_Intake_H_
#define SRC_Intake_H_


#include <WPILib.h>
#include <ctre\Phoenix.h>
#include "OperatorInputs.h"
#include "Lifter.h"


class Intake
{
public:
	enum Stage {kBottom, kIngest, kIngestWait, kBox, kEject};

	Intake(DriverStation *ds, OperatorInputs *inputs, Lifter *lifter);
	virtual ~Intake();
	void Init();
	void Loop();
	void AutoLoop();
	void TestLoop();
	void Stop();
	void ResetPosition();
	void AutoEject();

protected:
	DriverStation *m_ds;
	OperatorInputs *m_inputs;
	Lifter *m_lifter;
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
};


#endif /* SRC_Intake_H_ */
