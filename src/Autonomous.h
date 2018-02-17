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

	Autonomous(OperatorInputs *inputs, DriveTrain *drivetrain, DrivePID *drivepid);
	virtual ~Autonomous();
	void Init();
	void Loop();
	bool DriveStraight(double targetDistance);
	static void VelocityAdjust(Autonomous *arg);
	void Stop();

protected:
	DriveTrain *m_drivetrain;
	Timer *m_timerstraight;
	OperatorInputs *m_inputs;
	DrivePID *m_drivepid;
	Notifier* m_notifier;

	DriveStraightState m_straightstate;
	double m_acceldistance;
	double m_timermod;
	double m_timervalue;
	double m_distance;
	double m_target;
};


#endif //SRC_AUTONOMOUS_H_
