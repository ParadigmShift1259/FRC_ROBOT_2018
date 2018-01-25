/*
 * Grabber.h
 *
 *  Created on: Jan 20, 2018
 *      Author: Jival
 */

#ifndef SRC_Grabber_H_
#define SRC_Grabber_H_


#include <WPILib.h>
#include <ctre\Phoenix.h>
#include "OperatorInputs.h"


class Grabber
{
public:
	Grabber(OperatorInputs *inputs);
	virtual ~Grabber();
	void Init();
	void Loop();
	void TestLoop();
	void Stop();

protected:
	OperatorInputs *m_inputs;
	WPI_TalonSRX *m_leftmotor;
	WPI_TalonSRX *m_rightmotor;
	int m_leftposition;
	int m_rightposition;
};


#endif /* SRC_Grabber_H_ */
