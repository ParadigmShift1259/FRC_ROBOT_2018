/**
 *  DriveAnglePID2018.h
 *  Date:
 *  Last Edited By:
 */


#ifndef SRC_DriveAnglePID2018_H_
#define SRC_DriveAnglePID2018_H_


#include "WPILib.h"
#include <Smartdashboard/Smartdashboard.h>
#include <Commands/PIDSubsystem.h>
#include <Const.h>
#include "DriveTrainWPI.h"
//#include <ntCore.h>
//#include <nt_Value.h>


class DriveAnglePID2018: public PIDSubsystem
{
public:
	DriveAnglePID2018(DriveTrainWPI *drive);
	virtual ~DriveAnglePID2018();
	double ReturnPIDInput();
	void ChangeActive(bool newState);
	bool IsDone();
	void SetSetpointRelativeToError(double newSetpoint);
	void UsePIDOutput(double output);
	double ReturnCurrentPosition();
	void CheckPIDValues();
	void SetY(double y);
	double GetY();
	void SetRamp(bool ramp);
	bool GetRamp();
	bool IsEnabled();
	void SetRelativeSetpoint(double setpoint);
	void SetP(double p) {m_P = p; SmartDashboard::PutNumber("DP00_P",m_P); CheckPIDValues();}
	void SetI(double i) {m_I = i; SmartDashboard::PutNumber("DP00_I",m_I); CheckPIDValues();}
	void SetD(double d) {m_D = d; SmartDashboard::PutNumber("DP00_D",m_D); CheckPIDValues();}

protected:
	DriveTrainWPI* m_DriveTrainWPI;
	//OperatorInputs *m_inputs;
	bool isInitialized;
	bool isActive;
	double m_y;
	bool m_ramp;
	double m_P;
	double m_I;
	double m_D;
};


#endif /* SRC_DriveAnglePID2018_H_ */
