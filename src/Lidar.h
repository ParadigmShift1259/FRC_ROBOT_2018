#ifndef SRC_LIDAR_H_
#define SRC_LIDAR_H_
#include <WPILib.h>
#include <stdint.h>
#include <vector>

#define MAX_LIDAR_MESSAGE_LENGTH 2048
#define LIDAR_CMD_START 0
#define LIDAR_CMD_STOP 1
#define LIDAR_CMD_SET_SCAN_PERIOD 2
#define LIDAR_CMD_RESET 3
#define LIDAR_CMD_CLEAR_RESET 4
#define LIDAR_CMD_SET_SAMPLES_PER_SCAN 5
#define LIDAR_CMD_SET_SAMPLE_REJECTION 6
#define LIDAR_CMD_SET_PARK_TRIM 7
#define LIDAR_CMD_SET_MIN_MAX_ANGLE 8
#define LIDAR_CMD_MAX 9


class Lidar {
public:
	Lidar();
	virtual ~Lidar();

	void Loop();
	void start(); 													///Starts
	void stop();													///Stops
	void SetScanPeriod(const uint16_t period);						///In milliseconds, keep below 200 and keep samplesperscan/period < 1 (Maybe higher is ok?)
	void SetSamplesPerScan(const uint16_t samples_per_scan);		///How many samples per scan keep samplesperscan/period <1 (Maybe higher is ok?)
	void SetSampleRejectionMode(const bool enabled);				///Suggested to always be enabled
	void SetParkTrim(const int16_t trim);							///Changes where the zero angle is
	void SetMinMaxAngle(const uint16_t min, const uint16_t max);	///Sets what angles it scans in
	void ReadSensor();

private:
	SerialPort *serial;
	char CalcChecksum(const char *data, const uint16_t length);
	char command_buffer[MAX_LIDAR_MESSAGE_LENGTH];
	std::vector<uint16_t> samples;

};

#endif /* SRC_LIDAR_H_ */
