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

	double Ramp(double previousPow, double desiredPow, double rampSpeedMin, double rampSpeedMax);
	//void rampRightPower(double desiredPow, double rampSpeedMin, double rampSpeedMax);
	void setCoasting(double newCoasting) {m_coasting = newCoasting;}
	double getLeftPow() {return m_leftpow;}
	double getRightPow() {return m_rightpow;}
	bool getIsHighGear() {return m_ishighgear;}
	WPI_TalonSRX *LeftTalon() {return m_lefttalonlead;}
	WPI_TalonSRX *RightTalon() {return m_righttalonlead;}
	void setRamp(double newValue) {m_rampmax = newValue;}

protected:
	OperatorInputs *m_inputs;
	DriverStation *m_driverstation;
	WPI_TalonSRX *m_lefttalonlead;
	WPI_TalonSRX *m_lefttalonfollow;
	WPI_TalonSRX *m_righttalonlead;
	WPI_TalonSRX *m_righttalonfollow;
	Solenoid *m_shifter;
	Timer *m_timerramp;

	double m_leftpow;
	double m_rightpow;
	double m_leftspeed;
	double m_rightspeed;
	double m_leftposition;
	double m_rightposition;
	double m_coasting;
	double m_rampmax;

	double m_invertleft;
	double m_invertright;
	double m_direction;

	bool m_ishighgear;
	double m_previousx;
	double m_previousy;
	bool m_isdownshifting;
	bool m_lowspeedmode;
	bool m_shift;
};


#endif /* SRC_DRIVETRAIN2017_H_ */
