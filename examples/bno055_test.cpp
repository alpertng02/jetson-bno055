#include <iostream>
#include <string>
#include <thread>
#include <limits>
#include "bno055.hpp"

void sleep_ms(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main(int argc, char** argv) {

    if (argc != 3) {
        std::cout << "Error! Invalid command line arguments!\n";
        return -1;
    }

    std::string_view deviceDirectory { argv[1] };

    uint8_t deviceAddress {};
    try {
        auto deviceAdressInput = std::stoul(argv[2]);
        if (deviceAdressInput < 0 || deviceAdressInput > std::numeric_limits<uint8_t>().max()) {
            std::cout << "Error! Device I2C adress is not valid!\n";
            return -1;
        } else {
            deviceAddress = static_cast<uint8_t>(deviceAdressInput);
        }

    } catch (std::invalid_argument& err) {
        std::cout << "Error! Argument \"" << argv[2] << "\" is not a proper I2C adress!\n";
        return -1;
    }

    imu::BNO055 bno(deviceDirectory, deviceAddress);
    std::cout << "BNO055 is open!\n";

    sleep_ms(100);

    std::cout << std::endl;
    while (true) {

        try {
            auto calibStatus = bno.getSystemCalibrationStatus();
            printf("System Calibration Status: %u\n", calibStatus);

            auto accel = bno.getAccelMsq();
            printf("Acceleration => x: %3.2f,  y: %3.2f,  z: %3.2f\n", accel.x, accel.y, accel.z);

            auto gyro = bno.getGyroDps();
            printf("Gyro Dps     => x: %3.2f,  y: %3.2f,  z: %3.2f\n", gyro.x, gyro.y, gyro.z);

            auto mag = bno.getMagUT();
            printf("Mag uT      => x: %3.2f,  y: %3.2f,  z: %3.2f\n", mag.x, mag.y, mag.z);

            auto eulerAngles = bno.getEulerAngles();
            printf("Euler Angles => h: %3.2f,   p: %3.2f,   r: %3.2f\n\n", eulerAngles.h, eulerAngles.p, eulerAngles.r);

        } catch (std::runtime_error& err) {
            std::cout << err.what();
        }
        sleep_ms(10);

    }
    return 0;
}