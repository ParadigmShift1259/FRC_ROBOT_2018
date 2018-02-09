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
	Lifter(OperatorInputs *inputs);
	virtual ~Lifter();
	void Init();
	void Loop();
	void TestLoop();
	void Stop();
	void ResetPosition();
	bool IsBottom();

protected:
	OperatorInputs *m_inputs;
	WPI_TalonSRX *m_motor;
	Solenoid *m_solenoid;
	int m_position;
	double m_raisespeed;
	double m_lowerspeed;
	double m_liftermin;
	double m_liftermax;
};


#endif /* SRC_LIFTER_H_ */
