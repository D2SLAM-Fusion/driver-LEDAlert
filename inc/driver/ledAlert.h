/*
  File name: inc/driver/ledAlert.h
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

#ifndef __LED_ALERT_H__
#define __LED_ALERT_H__

#include "driver/gpio.h"

enum AlertState {
    ONLINE,
    OFFLINE,
    WARNING,
    ERROR
};

class LEDAlert {
private:
    JetsonGPIO led;
    AlertState alertLevel;
    bool LEDState;
    bool LEDSwitch();
    bool onlineState(int count);
    bool offlineState(int count);
    bool warningState(int count);
    bool errorState(int count);
    std::thread alertThread;
    void alertThreadFunc();
    int scanCount;
    bool scanStop;

public:
    LEDAlert();
    ~LEDAlert();
    bool startAlertThread();
    bool getAlertLevel(AlertState &level);
    bool setAlertLevel(AlertState level);
};

#endif // __LED_ALERT_H__