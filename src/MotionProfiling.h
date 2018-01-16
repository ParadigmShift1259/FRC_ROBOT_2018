/*
 * MotionProfiling.h
 *
 *  Created on: Jan 13, 2018
 *      Author: Matt Wildman
 */

#ifndef SRC_MOTIONPROFILING_H_
#define SRC_MOTIONPROFILING_H_
#include <ctre\Phoenix.h>
#include <Notifier.h>

class MotionProfiling {
public:
	MotionProfiling(WPI_TalonSRX *rightFront, WPI_TalonSRX *rightBack, WPI_TalonSRX *leftFront, WPI_TalonSRX *leftBack);
	void Init();
	virtual ~MotionProfiling();

protected:

	void PeriodicFeed();
	//For feeding points into the buffer
	Notifier m_notifier;



	WPI_TalonSRX *rightFrontTalon;
	WPI_TalonSRX *rightBackTalon;
	WPI_TalonSRX *leftFrontTalon;
	WPI_TalonSRX *leftBackTalon;
};

#endif /* SRC_MOTIONPROFILING_H_ */
