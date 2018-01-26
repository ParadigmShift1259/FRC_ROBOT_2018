#include <UltrasonicMaxbotix.h>

/// Create an instance of the Ultrasonic Maxbotix sensor.
///
/// See Ultrasonic class in the WPILib library for argument documentation
UltrasonicMaxbotix::UltrasonicMaxbotix(DigitalOutput* pingChannel, DigitalInput* echoChannel, DistanceUnit units /* = kInches */)
	: Ultrasonic(pingChannel, echoChannel, units)
{
}

/// Create an instance of the Ultrasonic Maxbotix sensor.
///
/// See Ultrasonic class in the WPILib library for argument documentation
UltrasonicMaxbotix::UltrasonicMaxbotix(DigitalOutput& pingChannel, DigitalInput& echoChannel, DistanceUnit units /* = kInches */)
	: Ultrasonic(pingChannel, echoChannel, units)
{
}

/// Create an instance of the Ultrasonic Maxbotix sensor.
///
/// See Ultrasonic class in the WPILib library for argument documentation
UltrasonicMaxbotix::UltrasonicMaxbotix(std::shared_ptr<DigitalOutput> pingChannel,std::shared_ptr<DigitalInput> echoChannel, DistanceUnit units /* = kInches */)
	: Ultrasonic(pingChannel, echoChannel, units)
{
}

/// Create an instance of the Ultrasonic Maxbotix sensor.
///
/// See Ultrasonic class in the WPILib library for argument documentation
UltrasonicMaxbotix::UltrasonicMaxbotix(int pingChannel, int echoChannel, DistanceUnit units /* = kInches */)
	: Ultrasonic(pingChannel, echoChannel, units)
{
}

UltrasonicMaxbotix::~UltrasonicMaxbotix()
{
}
	
/// Get the range in inches from the ultrasonic sensor
///
/// @return Range in inches to the target returned from the ultrasonic sensor
double UltrasonicMaxbotix::GetRangeInches() const
{
	return Ultrasonic::GetRangeInches() * kValueToInches;

}

/// Get the range in millimeters from the ultrasonic sensor
///
/// @return Range in millimeters to the target returned from the ultrasonic sensor
double UltrasonicMaxbotix::GetRangeMM() const
{
	return GetRangeInches() * 25.4;
}
