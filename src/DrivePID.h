/*
 * DrivePID.h
 *
 *  Created on: Jan 27, 2018
 *      Author: Developer
 */


#ifndef SRC_DRIVEPID_H_
#define SRC_DRIVEPID_H_


#include <Drivetrain.h>
#include <WPILib.h>
#include "DriveAnglePID.h"
#include "OperatorInputs.h"


class DrivePID: public PIDSubsystem
{
public:
	DrivePID(DriveTrain *drivetrain, PigeonIMU *pigeon, OperatorInputs *inputs);
	~DrivePID();
	void Init(double p = 0, double i = 0, double d = 0, bool enable = false);
	void Drive(double y, bool ramp = false);
	void Stop();

	void SetP(double p);
	void SetI(double i);
	void SetD(double d);
	void SetY(double y);
	void SetRelativeAngle(double angle);
	void SetAbsoluteAngle(double angle);

	void EnablePID();
	void DisablePID();
	double ReturnPIDInput();
	void UsePIDOutput(double output);

protected:
	DriveTrain *m_drivetrain;
	PigeonIMU *m_pigeon;
	OperatorInputs *m_inputs;
	double m_p;
	double m_i;
	double m_d;
	double m_y;
	double m_ramp;
	double m_angle;
};


#endif /* SRC_DRIVEPID_H_ */
