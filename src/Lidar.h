#ifndef SRC_LIDAR_H_
#define SRC_LIDAR_H_
#include <WPILib.h>
#include <stdint.h>


class Lidar {
public:
	Lidar();
	virtual ~Lidar();

	void start();
	void stop();

private:
	SerialPort *serial;
	char calcChecksum(const char *data, const uint16_t length);

};

#endif /* SRC_LIDAR_H_ */
