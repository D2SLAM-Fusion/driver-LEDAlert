/*
  File name: src/driver/ledAlert.cpp
  File description: This file defines the class LEDAlert, which is
                    used to control the LED status of Jetson Orin.

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


#include "driver/ledAlert.h"

inline void delay(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

LEDAlert::LEDAlert() : led(PIN_NUM, PIN_NAME), alertLevel(ONLINE), LEDState(false), scanCount(0), scanStop(false) {
    try {
        led.unexportGPIO();
        led.exportGPIO();
        led.setDirection("out");
        led.setValue(std::to_string(LEDState));
    } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
}

LEDAlert::~LEDAlert() {
    led.unexportGPIO();
    scanStop = true;
    if (alertThread.joinable()) {
        alertThread.join();
    }
}

bool LEDAlert::LEDSwitch() {
    try{
        LEDState = !LEDState;
        return led.setValue(std::to_string(LEDState));
    } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
}

bool LEDAlert::onlineState(int count) {
    if(LEDState) {
        LEDSwitch();
    }
    return true;
}

bool LEDAlert::offlineState(int count) {
    if(!LEDState) {
        LEDSwitch();
    }
    return true;
}

bool LEDAlert::warningState(int count) {
    if(count % 10 == 0) {
        LEDSwitch();
    }
    return true;
}

bool LEDAlert::errorState(int count) {
    if(count % 1 == 0) {
        LEDSwitch();
    }
    return true;
}

void LEDAlert::alertThreadFunc() {
    scanCount = 1;
    while (!scanStop) {
        try{
            switch (alertLevel) {
                case ONLINE:
                    onlineState(scanCount);
                    break;
                case OFFLINE:
                    offlineState(scanCount);
                    break;
                case WARNING:
                    warningState(scanCount);
                    break;
                case ERROR:
                    errorState(scanCount);
                    break;
                default:
                    break;
            }

            if (scanCount++ > 10) {
                scanCount = 1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } catch (std::exception& e) {
            std::cerr << e.what() << std::endl;
        }
    }
}

bool LEDAlert::startAlertThread() {
    alertThread = std::thread(&LEDAlert::alertThreadFunc, this);
    return true;
}

bool LEDAlert::getAlertLevel(AlertState &level) {
    level = alertLevel;
    return true;
}

bool LEDAlert::setAlertLevel(AlertState level) {
    alertLevel = level;
    return true;
}