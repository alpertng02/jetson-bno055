#include <iostream>
#include <string>
#include <thread>
#include "bno055.hpp"

void sleep_ms(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main(int argc, char** argv) {
    
    imu::BNO055 bno;

    if (!bno.init("/dev/i2c-1", 0x28)) {
        std::cout << "Error! Device could not be opened!\n";
        return -1;
    } 
    std::cout << "BNO055 is open!\n";
    sleep_ms(100);

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

    std::cout << std::endl;
    while (true) {

        auto [calibStatus, calibRes] = bno.getSystemCalibrationStatus();
        if (calibRes) {
            printf("System Calibration Status: %u\n", calibStatus);
        } else {
            printf("Error! Could not read calibration status from BNO055.\n");
        }

        auto [accel, accelRes] = bno.getAccelMsq();
        if (accelRes) {
            printf("Acceleration => x: %3.2f,  y: %3.2f,  z: %3.2f\n", accel.x, accel.y, accel.z);
        } else {
            printf("Error! Could not read accel from BNO055.\n");
        }

        auto [gyro, gyroRes] = bno.getGyroDps();
        if (gyroRes) {
            printf("Gyro Dps     => x: %3.2f,  y: %3.2f,  z: %3.2f\n", gyro.x, gyro.y, gyro.z);
        } else {
            printf("Error! Could not read gyro from BNO055.\n");
        }

        auto [mag, magRes] = bno.getMagUT();
        if (magRes) {
             printf("Mag uT      => x: %3.2f,  y: %3.2f,  z: %3.2f\n", mag.x, mag.y, mag.z);
        } else {
            printf("Error! Could not read mag from BNO055.\n");
        }

        auto [eulerAngles, eulerRes] = bno.getEulerAngles();
        if (eulerRes) {
            printf("Euler Angles => h: %3.2f,   p: %3.2f,   r: %3.2f\n\n", eulerAngles.h, eulerAngles.p, eulerAngles.r);
        } else {
            printf("Error! Could not read euler angles from BNO055.\n");
        }

        sleep_ms(10);
    }
    return 0;
}