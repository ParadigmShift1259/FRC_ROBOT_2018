/*
 * Climber.h
 *
 *  Created on: Jan 20, 2018
 *      Author: Jival
 */

#ifndef SRC_CLIMBER_H_
#define SRC_CLIMBER_H_


#include <WPILib.h>
#include <ctre\Phoenix.h>
#include "OperatorInputs.h"


class Climber
{
public:
	Climber(OperatorInputs *m_inputs);
	virtual ~Climber();
	void Init();
	void Loop();
	void TestLoop();
	void Stop();

protected:
	OperatorInputs *m_inputs;
	WPI_TalonSRX *m_motor;
};


#endif /* SRC_CLIMBER_H_ */
