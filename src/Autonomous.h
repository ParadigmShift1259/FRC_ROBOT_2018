#ifndef SRC_AUTONOMOUS_H_
#define SRC_AUTONOMOUS_H_


#include <WPILib.h>
#include "Drivetrain.h"
#include "OperatorInputs.h"
#include "DrivePID.h"
#include "Intake.h"
#include "Lifter.h"


class Autonomous
{
public:
	enum DriveStraightState {kStart, kAngle, kAccel, kMaintain, kDecel};
	enum TurnState {kInit, kTurning};

	Autonomous(OperatorInputs *inputs, DriveTrain *drivetrain, DrivePID *drivepid, Intake *intake, Lifter *lifter);
	virtual ~Autonomous();
	void Init();
	void Loop();
	void Stop();
	bool RampStraight(double targetdistance, double acceltime, double autopower, double deceldistance);
	bool TurnAngle(double angle, bool reset = true);
	bool MiniStraight(double targetdistance, double autopower, bool reset = true);
	bool AngleStraight(double angle, double targetdistance, double acceltime, double autopower, double deceldistance);
	void AutoCenterSwitchLeft1();
	void AutoCenterSwitchRight1();
	void AutoCenterSwitchLeft3();
	void AutoCenterSwitchRight3();
	void AutoLeftScaleLeft2();
	void AutoLeftScaleRight1();
	void AutoLeftScaleLeft1();
	void AutoRightScaleLeft1();
	void AutoRightScaleRight2();
	void AutoRightScaleRight1();
	void AutoStraight();
	void AutoTest();

protected:
	DriveTrain *m_drivetrain;
	OperatorInputs *m_inputs;
	DrivePID *m_drivepid;
	Intake *m_intake;
	Lifter *m_lifter;

	double m_pidstraight[3] = {0.04, 0.0012, 0.07};			// pid straight default values
	double m_pidangle[3] = {0.0, 0.0, 0.0};					// pid angle values

	double m_pidscale[3] = {0.013, 0.0002, 0.045};			// scale pid
//	double m_pidswitch[3] = {0.013, 0.0012, 0.045};
//	double m_pidswitch[3] = {0.04, 0.004, 0.15};			// switch pid
	double m_pidswitch[3] = {0.01, 0.0012, 0.07};			// switch pid
	double m_pidold[3] = {0.01, 0.0012, 0.07};				// Milwaukee Values

	Timer m_timer;
	DriveStraightState m_straightstate;
	TurnState m_turnstate;
	int m_autostage;

	double m_timermod;
	double m_distance;
};


#endif //SRC_AUTONOMOUS_H_

