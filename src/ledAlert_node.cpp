#include "ledAlert.h"
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