#ifndef SRC_AUTONOMOUS_H_
#define SRC_AUTONOMOUS_H_

#include <DriveTrain.h>
#include <WPILib.h>
#include "Const.h"

class Autonomous {
public:
	Autonomous(DriveTrain *drivetrain);
	void Init();
	void Loop();
	bool DriveStraight(double targetDistance);
	void Stop();
	virtual ~Autonomous();
protected:
	DriveTrain *m_drivetrain;
	Timer *m_timerstraight;

	enum DriveStraightState {kStart, kAccel, kMaintain, kDecel};
	DriveStraightState m_straightstate;

	double m_acceldistance;
	double m_timermod;
};

#endif /* SRC_AUTONOMOUS_H_ */
