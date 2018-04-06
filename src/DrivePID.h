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
#include "OperatorInputs.h"


class DrivePID: public PIDSubsystem
{
public:
	enum Feedback { kDisabled, kEncoder, kGyro };

	DrivePID(DriveTrain *drivetrain, OperatorInputs *inputs);
	~DrivePID();
	void Init(double p = 0, double i = 0, double d = 0, Feedback feedback = kDisabled);
	void Loop();
	void Drive(double y, bool ramp = false);
	void Stop();

	bool GetEnabled();

	void SetP(double p);
	void SetI(double i);
	void SetD(double d);
	void SetY(double y);
	void SetRelativeAngle(double angle);
	void SetAbsoluteAngle(double angle);
	void ResetGyro();
	bool IsOnTarget(double count = 0);

	void EnablePID();
	void DisablePID();
	double ReturnPIDInput();
	void UsePIDOutput(double output);

protected:
	DriveTrain *m_drivetrain;
	OperatorInputs *m_inputs;
	PigeonIMU *m_pigeon;
	Feedback m_feedback;
	double m_gyroval[3];
	double m_p;
	double m_i;
	double m_d;
	double m_y;
	double m_ramp;
	int m_ontarget;
};


#endif /* SRC_DRIVEPID_H_ */
