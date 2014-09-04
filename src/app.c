/**
 * Serial port library.
 * This library is a test program of using the serial port library to control 
 * serial port devices and communications.
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


#include <stdio.h>
#include <stdlib.h>
#include "serialport.h"


int main()
{
  char buf[1024];
  int len;
  int i;

  // Open a serial port device and check the result
  //int fd = spOpen("COM1"); // Windows
  int fd = spOpen("/dev/ttyUSB0"); // Linux
  if (!fd) return (-1);

  // Configurate the serial port
  spConfig(fd, 115200, 8, 'N', 1);

  // Loop read data from serial port
  while (1)
  {
    len = spRead(fd, buf, 1024);
    for (i=0; i<len; ++i)
    {
      printf("%c", buf[i]);
    }
  }

  return 0;
}

