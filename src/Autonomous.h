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

	Autonomous(OperatorInputs *inputs, DriveTrain *drivetrain);
	virtual ~Autonomous();
	void Init();
	void Loop();
	bool DriveStraight(double targetDistance);
	void Stop();

protected:
	DriveTrain *m_drivetrain;
	Timer *m_timerstraight;
	OperatorInputs *m_inputs;
	DrivePID *m_drivepid;

	DriveStraightState m_straightstate;
	double m_acceldistance;
	double m_timermod;
};


#endif //SRC_AUTONOMOUS_H_
