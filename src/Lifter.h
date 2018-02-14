/*
 * Lifter.h
 *
 *  Created on: Jan 20, 2018
 *      Author: Yreffoeg
 */

#ifndef SRC_LIFTER_H_
#define SRC_LIFTER_H_


#include <WPILib.h>
#include <ctre\Phoenix.h>
#include "OperatorInputs.h"


class Lifter
{
public:
	enum Stage {kIdle, kRaise};

	Lifter(DriverStation *ds, OperatorInputs *inputs);
	virtual ~Lifter();
	void Init();
	void Loop();
	void TestLoop();
	void Stop();
	void ResetPosition();
	bool IsBottom();

protected:
	DriverStation *m_ds;
	OperatorInputs *m_inputs;
	WPI_TalonSRX *m_motor;
	Solenoid *m_solenoid;
	Stage m_stage;
	int m_position;
	double m_raisespeed;
	double m_lowerspeed;
	double m_liftermin;
	double m_liftermax;
	double m_lifterminspd;
	double m_liftermaxspd;

};


#endif /* SRC_LIFTER_H_ */
