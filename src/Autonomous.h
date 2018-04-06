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
	bool DriveStraight(double targetdistance, double acceltime, double autopower, double deceldistance);
	bool TurnAngle(double angle);
	bool MiniStraight(double targetdistance, double autopower);
	bool AngleStraight(double angle, double targetdistance, double acceltime, double autopower, double deceldistance);
	bool MiniAngleStraight(double angle, double targetdistance, double autopower);
	void AutoCenterSwitchRight();
	void AutoCenterSwitchLeft();
	void AutoStraight();
	void AutoRightScaleRight();
	void AutoRightScaleLeft();
	void AutoLeftScaleRight();
	void AutoLeftScaleLeft();
	void AutoTest();

protected:
	DriveTrain *m_drivetrain;
	OperatorInputs *m_inputs;
	DrivePID *m_drivepid;
	Intake *m_intake;
	Lifter *m_lifter;

	double m_pid[3] = {0.01, 0.0012, 0.07}; //0.035, 0.0008, 0.07

	Timer m_timer;
	DriveStraightState m_straightstate;
	TurnState m_turnstate;
	int m_autostage;

	double m_timermod;
	double m_distance;
};


#endif //SRC_AUTONOMOUS_H_

