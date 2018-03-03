#ifndef SRC_AUTONOMOUS_H_
#define SRC_AUTONOMOUS_H_


#include <WPILib.h>
#include "Drivetrain.h"
#include "OperatorInputs.h"
#include "DrivePID.h"
#include "Intake.h"


class Autonomous
{
public:
	enum DriveStraightState {kStart, kAccel, kMaintain, kDecel};
	enum TurnState {kInit, kTurning};

	Autonomous(OperatorInputs *inputs, DriveTrain *drivetrain, DrivePID *drivepid, Intake* intake);
	virtual ~Autonomous();
	void Init();
	void Loop();
	void Stop();
	bool DriveStraight(double targetdistance, double acceltime, double autopower, double deceldistance);
	bool TurnAngle(double angle);
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
	Intake* m_intake;

	double m_pid[3] = {0.02, 0.0005, 0.07};

	Timer m_timer;
	DriveStraightState m_straightstate;
	TurnState m_turnstate;
	int m_autostage;

	double m_acceldistance;
	double m_timermod;
	double m_distance;
	double m_target;
};


#endif //SRC_AUTONOMOUS_H_

