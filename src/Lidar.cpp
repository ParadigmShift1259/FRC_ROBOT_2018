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
  std::vector<char> messageBuffer;
  serial->Read(command_buffer, 1);
  // verify start of command
  if (command_buffer[0] != 0xFF)
  {
      return;
  }
  else
  {
	  DriverStation::ReportError("No First 0xFF");
  }
  serial->Read(command_buffer, 1);
  if (command_buffer[1] != 0xFF)
  {
      return;
  }
  else
  {
	  DriverStation::ReportError("No Second 0xFF");
  }
  DriverStation::ReportError("Both 0xFF");
  // store message for checksum
  messageBuffer.push_back(0xff);
  messageBuffer.push_back(0xff);

  // read rest of header
  serial->Read(command_buffer, 3);
  // store rest of header for checksum
  messageBuffer.insert(messageBuffer.end(), command_buffer, command_buffer + 3);

  // get command id
  char command = command_buffer[0];
  // get MSB of message length
  uint16_t payload_length = command_buffer[1]<<8;
  // get LSB of message length
  payload_length += command_buffer[2];

  // read in message data
  if (payload_length > 0)
  {
      serial->Read(command_buffer, payload_length);
      messageBuffer.insert(messageBuffer.end(), command_buffer, command_buffer + payload_length);
  }

  // read checksum
  char checksum[2];
  serial->Read(checksum, 1);
  // verify checksum matches calulated checksum for message
  if (checksum[0] == CalcChecksum(&messageBuffer[0],5+payload_length))
  {
      switch(command)
      {
          case 0:
            samples.clear();
            // scan data is in message buffer
            for (int i = 0; i < (payload_length/2); i++)
            {
                uint16_t range_reading = command_buffer[i*2] + (command_buffer[(i*2)+1]<<8);
                samples.push_back(range_reading);
            }
            DriverStation::ReportError("GotData");
        	SmartDashboard::PutNumber("Sample 0", samples[0]);
          break;
      }
  }
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


Lidar::~Lidar()
{
	delete serial;
}

