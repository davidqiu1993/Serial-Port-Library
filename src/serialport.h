/**
 * Serial port library. This library is for controlling serial 
 * port devices and communications.
 *
 * Author:  David Qiu
 * Email:   david@davidqiu.com
 * Website: http://www.davidqiu.com
 *
 * Copyright (c) 2014, David Qiu.
 *  
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef _SERIALPORT_H_
#define _SERIALPORT_H_

#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>


/**
 * @param device Device name of the serial port.
 * @return An int indicating the opened file number pointing to 
 *         the target serial port. If failed to open the serial 
 *         port device, a negative number will be returned.
 *
 * @brief
 *    Open a serial port device.
 */
int spOpen(const char* device)
{
  int fd = -1;

  // Open the target device
  fd = open(device, O_RDWR|O_NOCTTY|O_NDELAY);
  if (fd < 0)
  {
    printf("spOpen(const char*): Failed to open device %s.\n", device);
    return (-1);
  }

  // Lock the device access
  if (fcntl(fd, F_SETFL, 0) < 0)
  {
    printf("spOpen(const char*): Failed to lock the access of device %s.\n", device);
    return (-1);
  }

  /*
  if(!isatty(STDIN_FILENO))
  {
    printf("spOpen(const char*): Standard input is not a terminal device.\n");
    return (-1);
  }
  */

  return fd;
}


/**
 * @param fd Opened file number pointing to the serial port.
 * @param baudRate Target baud rate of the serial port. Valid 
 *                 baud rates are { 2400, 4800, 9600, 19200, 
 *                 38400, 57600, 115200 }.
 * @param dataSize Target data size of the serial port. Valid 
 *                 data sizes are { 7, 8 }.
 * @param checkParity Target parity of the parity check. 
 *                    Valid check parities are { 'N', 'O', 
 *                    'E' }.
 * @param stopBit Target stop bit of the serial port.
 * @return An int indicating the configuration result. 
 *         Positive number indicates success, others indicate 
 *         failure.
 *
 * @brief
 *    Configurate an opened serial port device.
 */
int spConfig(int fd, int baudRate, int dataSize, char checkParity, int stopBit)
{
  struct termios originConfig;
  struct termios targetConfig;

  // Read the original terminal configurations and check if failed
  if (tcgetattr(fd, &originConfig) != 0) // "0" indicates sucess
  {
    printf("configSerialPort(int, int, int, char, int): Failed to read terminal description.\n");
    return (-1);
  }

  // Initialize cflag
  bzero(&targetConfig, sizeof(targetConfig));
  targetConfig.c_cflag |= CLOCAL | CREAD;

  // Set target baud rate
  switch (baudRate)
  {
    // Baud rate 2400
    case 2400:
      cfsetispeed(&targetConfig, B2400);
      cfsetospeed(&targetConfig, B2400);
      break;

    // Baud rate 4800
    case 4800:
      cfsetispeed(&targetConfig, B4800);
      cfsetospeed(&targetConfig, B4800);
      break;

    // Baud rate 9600
    case 9600:
      cfsetispeed(&targetConfig, B9600);
      cfsetospeed(&targetConfig, B9600);
      break;

    // Baud rate 19200
    case 19200:
      cfsetispeed(&targetConfig, B19200);
      cfsetospeed(&targetConfig, B19200);
      break;

    // Baud rate 38400
    case 38400:
      cfsetispeed(&targetConfig, B38400);
      cfsetospeed(&targetConfig, B38400);
      break;

    // Baud rate 57600
    case 57600:
      cfsetispeed(&targetConfig, B57600);
      cfsetospeed(&targetConfig, B57600);
      break;

    // Baud rate 115200
    case 115200:
      cfsetispeed(&targetConfig, B115200);
      cfsetospeed(&targetConfig, B115200);
      break;

    // Invalid baud rate
    default:
      // Baud rate does not exist
      printf("configSerialPort(int, int, int, char, int): Baud rate %d does not exist.\n", baudRate);
      return (-1);
  }

  // Set target data size
  targetConfig.c_cflag &= (~CSIZE);
  switch (dataSize)
  {
    // Data size 7
    case 7:
      targetConfig.c_cflag |= CS7;
      break;

    // Data size 8
    case 8:
      targetConfig.c_cflag |= CS8;
      break;

    // Invalid data size
    default:
      printf("configSerialPort(int, int, int, char, int): Data size %d does not exist.\n", dataSize);
      return (-1);
  }

  // Set target check parity
  switch (checkParity)
  {
    // None
    case'N':
      targetConfig.c_cflag &= (~PARENB);
      break;

    // Odd
    case'O':
      targetConfig.c_cflag |= PARENB;
      targetConfig.c_cflag |= PARODD;
      targetConfig.c_iflag |= (INPCK | ISTRIP);
      break;

    // Even
    case'E':
      targetConfig.c_cflag |= PARENB;
      targetConfig.c_cflag &= (~PARODD);
      targetConfig.c_iflag |= (INPCK | ISTRIP);
      break;

    // Invalid check parity
    default:
      printf("configSerialPort(int, int, int, char, int): Check parity %c does not exist.\n", checkParity);
      return (-1);
  }

  // Set target stop bit
  switch (stopBit)
  {
    // 1 stop bit
    case 1:
      targetConfig.c_cflag &= (~CSTOPB);
      break;

    // 2 stop bits
    case 2:
      targetConfig.c_cflag |= CSTOPB;
      break;

    // Invalid stop bit
    default:
      printf("configSerialPort(int, int, int, char, int): Stop bit %d does not exist.\n", stopBit);
      return (-1);
  }

  // Set waiting time
  targetConfig.c_cc[VTIME] = 0;

  // Set minimum byte
  targetConfig.c_cc[VMIN] = 0;

  // Flush input and output queue
  tcflush(fd, TCIFLUSH);

  // Active the target configurations immediately and check the result
  if (tcsetattr(fd, TCSANOW, &targetConfig) != 0) // "0" indicates success
  {
    printf("configSerialPort(int, int, int, char, int): Failed to configurate terminal description.\n");
    return (-1);
  }

  // Return success
  return 1;
}


/**
 * @param fd Opened file number pointing to the serial port.
 * @param buf Pointer to the target buffer to put read bytes.
 * @param len Length of bytes to read.
 * @return An int indicating the length of bytes read. If failed to 
 *         read bytes, a negative number will be returned.
 *
 * @brief
 *    Read bytes from a serial port device.
 */
int spRead(int fd, void* buf, int len)
{
  // Check the parameters
  if(!len)
  {
    printf("spRead(int, void*, int): The length of bytes to read must be positive.");
    return (-1);
  }

  // Read bytes and return the result
  return read(fd, buf, len);
}


/**
 * @param fd Opened file number pointing to the serial port.
 * @param buf Pointer to the target buffer to write.
 * @param len Length of the bytes to write.
 * @return An int indicating the number of bytes written. If failed 
 *         to write bytes to the serial port device, a negative 
 *         number will be returned.
 *
 * @brief
 *    Write bytes to a serial port device.
 */
int spWrite(int fd, void* buf, int len)
{
  // Check the parameters
  if(!len)
  {
    printf("spWrite(int, void*, int): The length of buffer must be positive.");
    return (-1);
  }

  // Write bytes and return the result
  return write(fd, buf, len);
}


/**
 * @param fd Opened file number pointing to the serial port.
 * @return An int indicating if the action result.
 *
 * @brief
 *    Close a serial port device.
 */
int spClose(int fd)
{
  // Close the serial port and return the result
  return close(fd);
}


#endif // _SERIALPORT_H_

