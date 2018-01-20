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

	void Init();

	void Loop(double targetRotation);

	virtual ~MotionMagic();

protected:

	WPI_TalonSRX *rightFrontTalon; //!<Right Front Talon, executes motion magic functions
	WPI_TalonSRX *rightBackTalon;  //!<Right Back Talon, follows Right Front Talon
	WPI_TalonSRX *leftFrontTalon;  //!<Left Front Talon, executes motion magic functions
	WPI_TalonSRX *leftBackTalon;   //!<Left Back Talon, follows Left Front Talon

};

#endif /* SRC_MOTIONMAGIC_H_ */
