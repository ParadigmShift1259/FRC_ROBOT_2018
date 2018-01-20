/**
 *  DriveTrain.h
 *  Date:
 *  Last Edited By:
 */


#ifndef SRC_DriveTrain_H_
#define SRC_DriveTrain_H_


#include "WPILib.h"
#include "OperatorInputs.h"
#include <driverstation.h>
#include <SpeedController.h>
#include <timer.h>
#include <ctre\Phoenix.h>


class DriveTrain
{
public:
	// Drivetrain modes
	enum DriveMode { kFollower, kDiscrete, kTank, kArcade, kCurvature };

	DriveTrain(DriveMode mode, OperatorInputs *inputs, DriverStation *ds);
	~DriveTrain();
	void Init();
	void Loop();
	void Stop();
	void Drive(double x, double y, bool ramp = false);
	void Shift();
		// change DriveTrain direction and return true if going forward
	bool ChangeDirection();
	void LowSpeedDriving();
	double Ramp(double previousPow, double desiredPow, double rampSpeedMin, double rampSpeedMax);
	double LeftMotor(double &invMaxValueXPlusY);
	double RightMotor(double &invMaxValueXPlusY);

	void setCoasting(double newCoasting) {m_coasting = newCoasting;}
	void setRamp(double newValue) {m_rampmax = newValue;}
	bool getIsHighGear() {return m_ishighgear;}
	WPI_TalonSRX *LeftTalonLead() {return m_lefttalonlead;}
	WPI_TalonSRX *RightTalonLead() {return m_righttalonlead;}
	WPI_TalonSRX *LeftTalonFollow() {return m_lefttalonfollow;}
	WPI_TalonSRX *RightTalonFollow() {return m_righttalonfollow;}

protected:
	DriveMode m_mode;
	OperatorInputs *m_inputs;
	DriverStation *m_driverstation;
	WPI_TalonSRX *m_lefttalonlead;
	WPI_TalonSRX *m_lefttalonfollow;
	WPI_TalonSRX *m_righttalonlead;
	WPI_TalonSRX *m_righttalonfollow;
	SpeedControllerGroup *m_leftscgroup;
	SpeedControllerGroup *m_rightscgroup;
	DifferentialDrive *m_differentialdrive;
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


#endif /* SRC_DriveTrain_H_ */
