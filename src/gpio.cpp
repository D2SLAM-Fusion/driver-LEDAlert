#include "gpio.h"

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