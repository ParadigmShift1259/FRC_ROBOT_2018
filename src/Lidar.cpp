/*
 * Lidar.cpp
 *
 *  Created on: Jul 21, 2018
 *      Author: Team1259
 */

#include <Lidar.h>

Lidar::Lidar()
{
	serial = new SerialPort(115200, SerialPort::Port::kUSB, 8, SerialPort::Parity::kParity_None, SerialPort::StopBits::kStopBits_One);
	serial->SetReadBufferSize(65535);
}


void Lidar::Loop()
{
	ReadSensor();

}


void Lidar::start(void)
{
  ssize_t msg_length = 5;

  command_buffer[0] = 0xFF;
  command_buffer[1] = 0xFF;
  command_buffer[2] = 0; // SCANNER2D_CMD_START
  command_buffer[3] = 0; // payload length
  command_buffer[4] = CalcChecksum(command_buffer, msg_length-1);
  serial->Write(command_buffer,msg_length);
}


void Lidar::stop(void)
{
  ssize_t msg_length = 5;

  command_buffer[0] = 0xFF;
  command_buffer[1] = 0xFF;
  command_buffer[2] = 1; // SCANNER2D_CMD_STOP
  command_buffer[3] = 0; // payload length
  command_buffer[4] = CalcChecksum(command_buffer, msg_length-1);
  serial->Write(command_buffer,msg_length);
}

char Lidar::CalcChecksum(const char *data, const uint16_t length)
{
  uint32_t i;
  uint8_t checksum = 0;

  for (i=0; i<length; i++)
  {
    checksum += (uint8_t)data[i];
  }

  return (char)~checksum;
}


void Lidar::ReadSensor()
{
	static std::vector<char> messageBuffer(500);
	messageBuffer.clear();
	bool endOfTransmission = false;
	int step = 0;
	uint16_t payloadLength = 0;
	uint16_t command = 0;
	uint16_t payloadBytesRead = 0;
	char checksum = 0;
	char msbSample = 0;
	char lsbSample = 0;
	while(!endOfTransmission)
	{
		// Read a byte
		while (!serial->Read(command_buffer, 1));
		char currentByte = command_buffer[0];
		messageBuffer.push_back(currentByte);
		switch (step)
		{
			// Synchronizing Bytes
			case 0:
			case 1:
				if (currentByte != 0xff)
				{
					endOfTransmission = true;
				}
				else
				{
					DriverStation::ReportError("Synchronization bits not found");
				}
				step++;
			break;
			// Message Type
			case 2:
				command = currentByte;
				switch (command)
				{
					case 0:
					case 1:
					break;
					default:
						DriverStation::ReportError("Invalid command");
						endOfTransmission = true;
				}
				step++;
			break;
			// Payload Length (MSB)
			case 3:
				payloadLength = ((uint16_t)currentByte) << 8;
				step++;
			break;
			// Payload Length (LSB)
			case 4:
				payloadLength |= currentByte;
				// Verify Payload Length isn't rediculously large
				if (payloadLength >= 2048)
				{
					DriverStation::ReportError("Payload too large");
					endOfTransmission = true;
				}
				else
				{
					samples.clear();
				}
				step++;
			break;
			// Payload Data
			case 5:
				if (command == 0)
				{
					if (!(payloadBytesRead & 1))
					{
						msbSample = currentByte;
					}
					else
					{
						lsbSample = currentByte;
						samples.push_back((((uint16_t)msbSample) << 8) | lsbSample);
					}
				}

				if (payloadLength <= payloadBytesRead)
				{
					step++;
				}
				payloadBytesRead++;
			break;
			// Checksum
			case 6:
				checksum = currentByte;
				SmartDashboard::PutNumber("Sample 0", samples[0]);
				File.open("/home/lvuser/distancelog.txt", std::fstream::app);
				File << samples[0]<<"\n";
				File.close();
				if (CalcChecksum(&messageBuffer[0],5+payloadLength) != checksum)
				{
					DriverStation::ReportError("Invalid checksum");
				}
				endOfTransmission = true;
				step++;
			break;
			default:
				DriverStation::ReportError("Invalid state");
				endOfTransmission = true;
		}
	}

	// Log Received Data
	File.open("/home/lvuser/inputpacket.txt", std::fstream::app);
	for (int i = 0; i < messageBuffer.size(); i++)
	{
		File << messageBuffer[i];
	}
	File.close();
}


void Lidar::SetScanPeriod(const uint16_t period)
{
  ssize_t msg_length = 7;

  command_buffer[0] = 0xFF;
  command_buffer[1] = 0xFF;
  command_buffer[2] = LIDAR_CMD_SET_SCAN_PERIOD;
  command_buffer[3] = 2; // payload length
  command_buffer[4] = (period & 0xFF);
  command_buffer[5] = (period & 0xFF00) >> 8;
  command_buffer[6] = CalcChecksum(command_buffer, msg_length-1);

  serial->Write(command_buffer,msg_length);
}


void Lidar::SetSamplesPerScan(const uint16_t samples_per_scan)
{
  ssize_t msg_length = 7;

  command_buffer[0] = 0xFF;
  command_buffer[1] = 0xFF;
  command_buffer[2] = LIDAR_CMD_SET_SAMPLES_PER_SCAN;
  command_buffer[3] = 2; // payload length
  command_buffer[4] = (samples_per_scan & 0xFF);
  command_buffer[5] = (samples_per_scan & 0xFF00) >> 8;
  command_buffer[6] = CalcChecksum(command_buffer, msg_length-1);

  serial->Write(command_buffer, msg_length);
}


void Lidar::SetSampleRejectionMode(const bool enabled)
{
  ssize_t msg_length = 6;

  command_buffer[0] = 0xFF;
  command_buffer[1] = 0xFF;
  command_buffer[2] =  LIDAR_CMD_SET_SAMPLE_REJECTION;
  command_buffer[3] = 1; // payload length
  command_buffer[4] = (enabled & 0x01);
  command_buffer[5] = CalcChecksum(command_buffer, msg_length-1);

  serial->Write(command_buffer, msg_length);
}


void Lidar::SetParkTrim(const int16_t trim)
{
  ssize_t msg_length = 7;

  command_buffer[0] = 0xFF;
  command_buffer[1] = 0xFF;
  command_buffer[2] = LIDAR_CMD_SET_PARK_TRIM;
  command_buffer[3] = 2; // payload length
  command_buffer[4] = (trim & 0xFF);
  command_buffer[5] = (trim & 0xFF00) >> 8;
  command_buffer[6] = CalcChecksum(command_buffer, msg_length-1);

  serial->Write(command_buffer, msg_length);
}


void Lidar::SetMinMaxAngle(const uint16_t min, const uint16_t max)
{
  ssize_t msg_length = 9;

  command_buffer[0] = 0xFF;
  command_buffer[1] = 0xFF;
  command_buffer[2] = LIDAR_CMD_SET_MIN_MAX_ANGLE;
  command_buffer[3] = 4; // payload length
  command_buffer[4] = (min & 0xFF);
  command_buffer[5] = (min & 0xFF00) >> 8;
  command_buffer[6] = (max & 0xFF);
  command_buffer[7] = (max & 0xFF00) >> 8;
  command_buffer[8] = CalcChecksum(command_buffer, msg_length-1);

  serial->Write(command_buffer, msg_length);
}


void Lidar::Read(const ssize_t bytesToRead)
{
    int bytesRead = 0;
    while (bytesRead < bytesToRead)
    {
        bytesRead += serial->Read(&command_buffer[bytesRead], bytesToRead-bytesRead);
    }
}


Lidar::~Lidar()
{
	delete serial;
}

