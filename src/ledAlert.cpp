#include "ledAlert.h"

inline void delay(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

LEDAlert::LEDAlert() : led(PIN_NUM, PIN_NAME), alertLevel(ONLINE), LEDState(false) {
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
    while (true) {
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