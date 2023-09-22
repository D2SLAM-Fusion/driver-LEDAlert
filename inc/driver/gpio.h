/*
  File name: inc/driver/gpio.h
  File description: This file defines the class JetsonGPIO, which is
                    used to control the GPIO of Jetson Orin.

  Author: JasonHsu
  Created: 2023/9/21

  This file is part of [LEDAlert].

  [LEDAlert] is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  [LEDAlert] is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with [LEDAlert]. If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef __GPIO_H__
#define __GPIO_H__

#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#define PIN_NUM (466)      // UART0_TXD
#define PIN_NAME ("PX.04") // cat /sys/kernel/debug/gpio | grep 466

class JetsonGPIO
{
  private:
    int gpioNumber;
    std::string gpioName;
    std::string getSysfsPath(const std::string &file);

  public:
    JetsonGPIO(int number, std::string name);
    ~JetsonGPIO();
    bool unexportGPIO();
    bool exportGPIO();
    bool setDirection(const std::string &direction);
    bool setValue(const std::string &value);
};

#endif // __GPIO_H__