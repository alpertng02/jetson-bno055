#include <iostream>
#include <string>
#include <thread>
#include "bno055.hpp"

void sleep_ms(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main(int argc, char **argv) {
    imu::BNO055 bno("/dev/i2c-1", 0x28);
    sleep_ms(10);

    if (!bno.isOpen()) {
        std::cout << "Error! Device could not be opened!\n";
        return -1;
    } 
    std::cout << "BNO055 is open!\n";

    if (!bno.setPowerMode(imu::BNO055::PowerMode::Normal)) {
        std::cout << "Error! BNO055 power mode could not be set!\n";
        return -1;
    }
    std::cout << "BNO055 power mode is set to normal.\n";
    sleep_ms(100);

    if (!bno.setOperationMode(imu::BNO055::OperationMode::NDOF)) {
        std::cout << "Error! BNO055 opearation mode could not be set!\n";
        return -1;
    }
    std::cout << "BNO055 operation mode is set to NDOF\n";
    sleep_ms(100);

    while (true) {
        auto [eulerAngles, res] = bno.getEulerAngles();
        if (res) {
            printf("h: %3.2f,   p: %3.2f,   r: %3.2f\n\n", eulerAngles.h, eulerAngles.p, eulerAngles.r);
        } else {
            printf("Error! Could not read from BNO055\n");
        }
        sleep_ms(10);
    }
    return 0;
}