/*
 * Autonomous.h
 *
 *  Created on: Jan 31, 2018
 *      Author: Developer
 */

#ifndef SRC_AUTONOMOUS_H_
#define SRC_AUTONOMOUS_H_


#include <WPILib.h>
#include "OperatorInputs.h"
#include "Drivetrain.h"
#include "DrivePID.h"


class Autonomous
{
public:
	Autonomous(OperatorInputs *inputs, DriveTrain *drivetrain);
	virtual ~Autonomous();
	void Init();
	void Loop();
	void Stop();

protected:
	OperatorInputs *m_inputs;
	DriveTrain *m_drivetrain;
	DrivePID *m_drivepid;
};


#endif /* SRC_AUTONOMOUS_H_ */
