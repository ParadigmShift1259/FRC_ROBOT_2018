/**
 *  DriveTrain.h
 *  Date:
 *  Last Edited By:
 */


#ifndef SRC_DriveTrain_H_
#define SRC_DriveTrain_H_


#include <WPILib.h>
#include <ctre\Phoenix.h>
#include "OperatorInputs.h"


class DriveTrain
{
public:
	// Drivetrain modes
	enum DriveMode { kNone, kFollower, kDiscrete, kTank, kArcade, kCurvature };

	DriveTrain(OperatorInputs *inputs, WPI_TalonSRX *left1 = nullptr, WPI_TalonSRX *left2 = nullptr, WPI_TalonSRX *left3 = nullptr, WPI_TalonSRX *right1 = nullptr, WPI_TalonSRX *right2 = nullptr, WPI_TalonSRX *right3 = nullptr);
	~DriveTrain();
	void Init(DriveMode mode = kFollower);
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

	double GetLeftPosition(int encoder = 0);
	double GetRightPosition(int encoder = 0);
	double GetLeftVelocity(int encoder = 0);
	double GetRightVelocity(int encoder = 0);
	double GetMaxVelocity(int encoder = 0);
	double GetLeftDistance(int encoder = 0);
	double GetRightDistance(int encoder = 0);
	double GetMaxDistance(int encoder = 0);
	double GetAverageMaxDistance(int encoder = 0);
	void ResetDeltaDistance(int encoder = 0);
	double GetMaxDeltaDistance(int encoder = 0);

	WPI_TalonSRX *LeftTalon1() {return m_lefttalon1;}
	WPI_TalonSRX *RightTalon1() {return m_righttalon1;}
	WPI_TalonSRX *LeftTalon2() {return m_lefttalon2;}
	WPI_TalonSRX *RIghtTalon2() {return m_righttalon2;}
	WPI_TalonSRX *LeftTalon3() {return m_lefttalon3;}
	WPI_TalonSRX *RIghtTalon3() {return m_righttalon3;}

protected:
	DriveMode m_mode;
	OperatorInputs *m_inputs;
	WPI_TalonSRX *m_lefttalon1;
	WPI_TalonSRX *m_lefttalon2;
	WPI_TalonSRX *m_lefttalon3;
	WPI_TalonSRX *m_righttalon1;
	WPI_TalonSRX *m_righttalon2;
	WPI_TalonSRX *m_righttalon3;
	bool m_lefttalon1owner;
	bool m_lefttalon2owner;
	bool m_lefttalon3owner;
	bool m_righttalon1owner;
	bool m_righttalon2owner;
	bool m_righttalon3owner;
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

	double m_prevleftdistance;
	double m_prevrightdistance;

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
