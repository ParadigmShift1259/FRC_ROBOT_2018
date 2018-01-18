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
#include "Profile.h"

class MotionProfiling {
public:
	MotionProfiling(WPI_TalonSRX *rightFront, WPI_TalonSRX *rightBack, WPI_TalonSRX *leftFront, WPI_TalonSRX *leftBack);


	//An init, to be called once before running motion profile
	void Init();

	void Loop();
	/*
	 * Every time this is called the talons will process their buffer. Currently
	 * called every 5ms which is twice as fast as the talons process the points
	 */
	static void PeriodicFeed(MotionProfiling *arg);

	virtual ~MotionProfiling();

protected:
	/*
	 * Feed our motion profile points from Profile.h into the Talon
	 *
	 * Assumes that the entire motion profile is contained within two
	 * two dimensional arrays. 1 is the left array and 2 is the
	 * right array. They also must be the same size
	 */
	void feedTopBuffer();

	Notifier *m_notifier;

	WPI_TalonSRX *rightFrontTalon;
	WPI_TalonSRX *rightBackTalon;
	WPI_TalonSRX *leftFrontTalon;
	WPI_TalonSRX *leftBackTalon;

	int stage;

	MotionProfileStatus leftStatus;
	MotionProfileStatus rightStatus;
};

#endif /* SRC_MOTIONPROFILING_H_ */
