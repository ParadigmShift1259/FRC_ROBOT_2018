#ifndef SRC_AUTONOMOUS_H_
#define SRC_AUTONOMOUS_H_

#include <WPILib.h>
#include "Drivetrain.h"
#include "OperatorInputs.h"
#include "DrivePID.h"


class Autonomous
{
public:
//	enum DriveStraightState {kStart, kAccel, kMaintain, kDecel};
	enum Stage {kIdle, kStraight, kTurn};

	Autonomous(OperatorInputs *inputs, DriveTrain *drivetrain);
	virtual ~Autonomous();
	void Init();
	void Loop();
	bool GoStraight(double inches, double power);
//	bool DriveStraight(double targetDistance);
	void Stop();

protected:
	DriveTrain *m_drivetrain;
	OperatorInputs *m_inputs;
	DrivePID *m_drivepid;
//	Timer *m_timerstraight;

	Stage m_stage;
//	DriveStraightState m_straightstate;
//	double m_distance;
//	double m_acceldistance;
//	double m_timermod;
};


#endif //SRC_AUTONOMOUS_H_
