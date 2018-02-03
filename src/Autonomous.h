#ifndef SRC_AUTONOMOUS_H_
#define SRC_AUTONOMOUS_H_

#include <Drivetrain.h>
#include <WPILib.h>
#include "Const.h"
#include "OperatorInputs.h"
#include "DrivePID.h"

class Autonomous {
public:
	Autonomous(OperatorInputs *inputs, DriveTrain *drivetrain);
	void Init();
	void Loop();
	bool DriveStraight(double targetDistance);
	void Stop();
	virtual ~Autonomous();
protected:
	DriveTrain *m_drivetrain;
	Timer *m_timerstraight;
	OperatorInputs *m_inputs;
	enum DriveStraightState {kStart, kAccel, kMaintain, kDecel};
	DriveStraightState m_straightstate;
	DrivePID *m_drivepid;
	double m_acceldistance;
	double m_timermod;
};
#endif //SRC_AUTONOMOUS_H_
