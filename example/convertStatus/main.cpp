/*
  File name: example/convertStatus/main.cpp
  File description: This file implements the main function of the example, which
                    is used to convert the alert level to the LED status.

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
#include <iostream>

int main(int argc, char** argv) {
    LEDAlert* ledAlert = new LEDAlert();
    AlertState alertLevel;
    ledAlert->startAlertThread();

    while (true) {
        std::cout << "Enter alert level (0-3) or 'q' to quit: ";
        std::string input;
        std::getline(std::cin, input);

        if (input == "q") {
            delete ledAlert;
            break;
        }

        alertLevel = static_cast<AlertState>(std::stoi(input));
        ledAlert->setAlertLevel(alertLevel);
    }

    return 0;
}