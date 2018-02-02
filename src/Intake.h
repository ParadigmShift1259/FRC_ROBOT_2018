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


class Intake
{
public:
	Intake(OperatorInputs *inputs);
	virtual ~Intake();
	void Init();
	void Loop();
	void TestLoop();
	void Stop();
	void ResetPosition();

protected:
	OperatorInputs *m_inputs;
	WPI_TalonSRX *m_leftmotor;
	WPI_TalonSRX *m_rightmotor;
	Solenoid *m_solenoid;
	DigitalInput *m_cubesensor;
	int m_leftposition;
	int m_rightposition;
};


#endif /* SRC_Intake_H_ */
