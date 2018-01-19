/*
 * MotionMagic.h
 *
 *  Created on: Jan 18, 2018
 *      Author: Developer
 */

#ifndef SRC_MOTIONMAGIC_H_
#define SRC_MOTIONMAGIC_H_

#include <ctre\Phoenix.h>

class MotionMagic {
public:
	MotionMagic(WPI_TalonSRX *rightFront, WPI_TalonSRX *rightBack, WPI_TalonSRX *leftFront, WPI_TalonSRX *leftBack);


	//An init, to be called once before running motion profile
	void Init();

	void Loop(double targetRotation);

	virtual ~MotionMagic();

protected:

	WPI_TalonSRX *rightFrontTalon;
	WPI_TalonSRX *rightBackTalon;
	WPI_TalonSRX *leftFrontTalon;
	WPI_TalonSRX *leftBackTalon;

};

#endif /* SRC_MOTIONMAGIC_H_ */
