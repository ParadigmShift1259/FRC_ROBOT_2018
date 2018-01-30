/*
 * DrivePID.h
 *
 *  Created on: Jan 27, 2018
 *      Author: Developer
 */


#ifndef SRC_DRIVEPID_H_
#define SRC_DRIVEPID_H_


#include <WPILib.h>
#include "DriveAnglePID.h"
#include "DriveTrain.h"
#include "OperatorInputs.h"


class DrivePID: public PIDSubsystem
{
public:
	DrivePID(DriveTrain *drivetrain, OperatorInputs *inputs);
	~DrivePID();
	void Init(double p = 0, double i = 0, double d = 0, bool enable = false);
	void Drive(double y);
	void Stop();

	void EnableAnglePID();
	void DisableAnglePID();

	void SetP(double p);
	void SetI(double i);
	void SetD(double d);
	void SetY(double y);
	void EnablePID();
	void DisablePID();
	double ReturnPIDInput();
	void UsePIDOutput(double output);

protected:
	DriveTrain *m_drivetrain;
	OperatorInputs *m_inputs;
	double m_p;
	double m_i;
	double m_d;
	double m_y;
};


#endif /* SRC_DRIVEPID_H_ */
