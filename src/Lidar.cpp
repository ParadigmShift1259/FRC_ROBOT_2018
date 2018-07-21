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

void Lidar::start(void)
{
  ssize_t msg_length = 5;
  char command_buffer[msg_length];

  command_buffer[0] = 0xFF;
  command_buffer[1] = 0xFF;
  command_buffer[2] = 0; // SCANNER2D_CMD_START
  command_buffer[3] = 0; // payload length
  command_buffer[4] = calcChecksum(command_buffer, msg_length-1);
  serial->Write(command_buffer,msg_length);
}


void Lidar::stop(void)
{
  ssize_t msg_length = 5;
  char command_buffer[msg_length];

  command_buffer[0] = 0xFF;
  command_buffer[1] = 0xFF;
  command_buffer[2] = 1; // SCANNER2D_CMD_STOP
  command_buffer[3] = 0; // payload length
  command_buffer[4] = calcChecksum(command_buffer, msg_length-1);
  serial->Write(command_buffer,msg_length);
}

char Lidar::calcChecksum(const char *data, const uint16_t length)
{
  uint32_t i;
  uint8_t checksum = 0;

  for (i=0; i<length; i++)
  {
    checksum += (uint8_t)data[i];
  }

  return (char)~checksum;
}


Lidar::~Lidar()
{
	delete serial;
}

