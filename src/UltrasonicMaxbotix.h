/*
 * UltrasonicMaxbotix.h
 *
 *  Created on: Jan 23, 2018
 *      Author: Developer
 */

#ifndef SRC_ULTRASONICMAXBOTIX_H_
#define SRC_ULTRASONICMAXBOTIX_H_

#include <Ultrasonic.h>

/// Ultrasonic Maxbotix rangefinder MB1013
///
/// \detail The ultrasonic class was written for the Daventech SRF 04 which has a 5.8 us per mm
/// The Maxbotix sensor has a 1 us per mm factor, so this class wraps the base and provides the correct scaling.
class UltrasonicMaxbotix : public Ultrasonic
{
public:
	~UltrasonicMaxbotix();

	UltrasonicMaxbotix(DigitalOutput* pingChannel, DigitalInput* echoChannel,
	             DistanceUnit units = kInches);
	UltrasonicMaxbotix(DigitalOutput& pingChannel, DigitalInput& echoChannel,
	             DistanceUnit units = kInches);
	UltrasonicMaxbotix(std::shared_ptr<DigitalOutput> pingChannel,
	             std::shared_ptr<DigitalInput> echoChannel,
	             DistanceUnit units = kInches);
	UltrasonicMaxbotix(int pingChannel, int echoChannel, DistanceUnit units = kInches);

	double GetRangeInches() const;
	double GetRangeMM() const;


private:
	static constexpr double kValueToInches = 6.35; //!< Emperically determined conversion factor
};


#endif /* SRC_ULTRASONICMAXBOTIX_H_ */
