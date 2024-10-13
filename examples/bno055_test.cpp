#include <iostream>
#include <string>
#include <thread>
#include "bno055.hpp"

int main() {

    imu::BNO055 bno("/dev/i2c-0", 0x28);

    if (!bno.isOpen()) {
        std::cout << "Error! Device could not be opened!\n";
        return -1;
    } 
    std::cout << "BNO055 is open!\n";

    while (true) {
        auto [eulerAngles, res] = bno.getEulerAngles();
        if (res) {
            printf("h: %3.2f,   p: %3.2f,   r: %3.2f\n\n", eulerAngles.h, eulerAngles.p, eulerAngles.r);
        } else {
            printf("Error! Could not read from BNO055\n");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}