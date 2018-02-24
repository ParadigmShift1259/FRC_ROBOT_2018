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
	bool DriveStraight(double targetdistance, double acceltime, double autopower, double deceldistance);
	bool TurnAngle(double angle);
	void Stop();

protected:
	DriveTrain *m_drivetrain;
	Intake* m_intake;
	Timer *m_timerstraight;
	OperatorInputs *m_inputs;
	DrivePID *m_drivepid;
	double pid[3] = {0.009, 0.0005, 0.07};

	DriveStraightState m_straightstate;
	TurnState m_turn;
	double m_acceldistance;
	double m_timermod;
	double m_timervalue;
	double m_distance;
	double m_target;
	int m_stage;
};


#endif //SRC_AUTONOMOUS_H_
