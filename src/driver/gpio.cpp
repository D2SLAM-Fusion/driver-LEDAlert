/*
  File name: src/driver/gpio.cpp
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

#include "driver/gpio.h"

JetsonGPIO::JetsonGPIO(int number, std::string name) : gpioNumber(number), gpioName(name) {}

JetsonGPIO::~JetsonGPIO() {
    unexportGPIO();
}

std::string JetsonGPIO::getSysfsPath(const std::string& file) {
    return "/sys/class/gpio/" + gpioName + "/" + file;
}

bool JetsonGPIO::unexportGPIO() {
    std::ofstream unexportFile("/sys/class/gpio/unexport");
    if (!unexportFile) {
        std::cerr << "Error unexporting GPIO" << std::endl;
        return false;
    }
    unexportFile << gpioNumber;
    return true;
}

bool JetsonGPIO::exportGPIO() {
    std::ofstream exportFile("/sys/class/gpio/export");
    if (!exportFile) {
        std::cerr << "Error exporting GPIO" << std::endl;
        return false;
    }
    exportFile << gpioNumber;
    return true;
}

bool JetsonGPIO::setDirection(const std::string& direction) {
    std::ofstream directionFile(getSysfsPath("direction"));
    if (!directionFile) {
        std::cerr << "Error setting GPIO direction" << std::endl;
        unexportGPIO();
        return false;
    }
    directionFile << direction;
    return true;
}

bool JetsonGPIO::setValue(const std::string& value) {
    std::ofstream valueFile(getSysfsPath("value"));
    if (!valueFile) {
        std::cerr << "Error setting GPIO value" << std::endl;
        unexportGPIO();
        return false;
    }
    valueFile << value;
    return true;
}