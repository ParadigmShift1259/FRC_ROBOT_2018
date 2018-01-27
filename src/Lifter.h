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

protected:
	OperatorInputs *m_inputs;
	WPI_TalonSRX *m_motor;
	int m_position;
};


#endif /* SRC_LIFTER_H_ */
