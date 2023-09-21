#ifndef __GPIO_H__
#define __GPIO_H__

#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <csignal>

#define PIN_NUM (466) // UART0_TXD
#define PIN_NAME ("PX.04") // cat /sys/kernel/debug/gpio | grep 466

class JetsonGPIO {
private:
    int gpioNumber;
    std::string gpioName;
    std::string getSysfsPath(const std::string& file);

public:
    JetsonGPIO(int number, std::string name);
    ~JetsonGPIO();
    bool unexportGPIO();
    bool exportGPIO();
    bool setDirection(const std::string& direction);
    bool setValue(const std::string& value);
};

#endif // __GPIO_H__