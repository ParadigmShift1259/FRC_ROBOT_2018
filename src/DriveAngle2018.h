/**
 *  DriveAngle.h
 *  Date:
 *  Last Edited By:
 */


#ifndef SRC_DRIVE_ANGLE_H_
#define SRC_DRIVE_ANGLE_H_


#include <DriveAnglePID2018.h>
#include "DriveTrainWPI.h"
#include "OperatorInputs.h"
#include <SmartDashboard/SmartDashboard.h>


class DriveAngle2018
{
public:
	DriveAngle2018(DriveTrainWPI *DriveTrainWPI, OperatorInputs *inputs);
	virtual ~DriveAngle2018();
	void EnableAnglePID();
	void DisableAnglePID();
	void SetRelativeAngle(double target);
	double GetAngle();
	bool IsOnTarget();
	void Init(bool enable=false);
	void Loop();
	void Drive(double y, bool ramp = false);
	void Stop();
	void SetVisionAngle(double angle);
	void RunNormalDrive();
	bool IsEnabled();
	bool IsHighGear();
	void Shift();
	void SetToCurrentAngle();
	void SetP(double p) {m_driveAnglePID->SetP(p);};
	void SetI(double i) {m_driveAnglePID->SetI(i);};
	void SetD(double d) {m_driveAnglePID->SetD(d);};

protected:
	DriveTrainWPI *m_DriveTrainWPI;
	OperatorInputs *m_inputs;
	DriveAnglePID2018 *m_driveAnglePID;
	double m_angle;
};


#endif /* SRC_DRIVE_ANGLE_H_ */
