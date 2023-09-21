#ifndef __LED_ALERT_H__
#define __LED_ALERT_H__

#include "gpio.h"

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

public:
    LEDAlert();
    ~LEDAlert();
    bool startAlertThread();
    bool getAlertLevel(AlertState &level);
    bool setAlertLevel(AlertState level);
};

#endif // __LED_ALERT_H__