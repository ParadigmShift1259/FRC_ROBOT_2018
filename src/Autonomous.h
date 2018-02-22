#ifndef SRC_AUTONOMOUS_H_
#define SRC_AUTONOMOUS_H_

#include <WPILib.h>
#include "Drivetrain.h"
#include "OperatorInputs.h"
#include "DrivePID.h"


class Autonomous
{
public:
	enum DriveStraightState {kStart, kAccel, kMaintain, kDecel};
	enum TurnState {kInit, kTurning, kIdle};

	Autonomous(OperatorInputs *inputs, DriveTrain *drivetrain, DrivePID *drivepid);
	virtual ~Autonomous();
	void Init();
	void Loop();
	bool DriveStraight(double targetdistance, double acceltime, double autopower, double deceldistance);
	bool TurnAngle(double angle);
	static void VelocityAdjust(Autonomous *arg);
	void Stop();

protected:
	double pid[3] = {0.009, 0.0005, 0.07};
	DriveTrain *m_drivetrain;
	Timer *m_timerstraight;
	OperatorInputs *m_inputs;
	DrivePID *m_drivepid;
	Notifier* m_notifier;

	DriveStraightState m_straightstate;
	TurnState m_turn;
	double m_acceldistance;
	double m_timermod;
	double m_timervalue;
	double m_distance;
	double m_target;
	int stage;
};


#endif //SRC_AUTONOMOUS_H_
