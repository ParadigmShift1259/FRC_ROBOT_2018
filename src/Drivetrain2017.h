/**
 *  Drivetrain.h
 *  Date:
 *  Last Edited By:
 */


#ifndef SRC_DRIVETRAIN2017_H_
#define SRC_DRIVETRAIN2017_H_


#include "WPILib.h"
#include "OperatorInputs.h"
#include <driverstation.h>
#include <SpeedController.h>
#include <timer.h>
#include <ctre\Phoenix.h>
#include <iostream>
#include <fstream>


class OldDriveTrain
{
public:
	OldDriveTrain(OperatorInputs *inputs, DriverStation *ds);
	~OldDriveTrain();
	void Init();
	void Loop();
	void Stop();
	void Drive(double x, double y, bool ramp = false);
	void Shift();
	// change drivetrain direction and return true if going forward
	bool ChangeDirection();
	void LowSpeedDriving();

	double LeftMotor(double &invMaxValueXPlusY);
	double RightMotor(double &invMaxValueXPlusY);
	void SetRatioLR();
	void ResetEncoders();
	void CheckEncoderTimer();

	double Ramp(double previousPow, double desiredPow, double rampSpeedMin, double rampSpeedMax);
	//void rampRightPower(double desiredPow, double rampSpeedMin, double rampSpeedMax);
	void setCoasting(double newCoasting) {m_coasting = newCoasting;}
	double getLeftPow() {return m_leftpow;}
	double getRightPow() {return m_rightpow;}
	double getRatio() {return m_ratiolr;}
	bool getIsHighGear() {return m_ishighgear;}
	bool getIsLeftFaster() {return m_isleftfaster;}
	WPI_TalonSRX *LeftTalon() {return m_lefttalonlead;}
	WPI_TalonSRX *RightTalon() {return m_righttalonlead;}
	void setRamp(double newValue) {m_rampmax = newValue;}

	//double getRightEncoderPulses() {return m_rightencoder->GetRaw();}
	//double getLeftEncoderPulses() {return m_leftencoder->GetRaw();}
	//double getRightEncoderDistance() {return m_rightencoder->GetDistance();}
	//double getLeftEncoderDistance() {return m_leftencoder->GetDistance();}

	// moved these variables from protected to public
	bool m_ishighgear; //Robot starts in low gear
	double m_previousx;
	double m_previousy;
	bool m_isdownshifting;
	bool m_lowspeedmode;
	bool m_shift;

	Timer *dataTimer;
	fstream leftMotorFile;
	fstream rightMotorFile;
	void outputData();

protected:
	OperatorInputs *m_inputs;
	DriverStation *m_driverstation;
	WPI_TalonSRX *m_lefttalonlead;
	WPI_TalonSRX *m_lefttalonfollow;
	WPI_TalonSRX *m_righttalonlead;
	WPI_TalonSRX *m_righttalonfollow;
	Solenoid *m_shifter;
	//Encoder *m_leftencoder;
	//Encoder *m_rightencoder;
	Timer *m_timerencoder;
	Timer *m_timerramp;

	double m_leftpow;
	double m_rightpow;
	double m_leftencodermax;
	double m_rightencodermax;
	double m_ratiolr;
	double m_leftencoderfix;
	double m_rightencoderfix;
	bool m_isleftfaster;
	double m_leftspeed;
	double m_rightspeed;
	double m_leftposition;
	double m_rightposition;
	double m_coasting;
	double m_rampmax;

	double m_invertleft;
	double m_invertright;
	double m_direction;
};



#endif /* SRC_DRIVETRAIN2017_H_ */
