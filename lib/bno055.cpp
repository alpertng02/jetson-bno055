#include "bno055.hpp"
#include "i2c/i2c.h"
#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <exception>

/***************************************************************/
/**\name    BUS READ AND WRITE FUNCTIONS           */
/***************************************************************/
static I2CDevice i2cdev {};
static constexpr size_t I2C_BUFFER_LEN = 16;

static int8_t jetson_i2c_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t wr_len) {
    if (reg_data == nullptr) {
        return BNO055_ERROR;
    }
    // Write data to register(s) over I2C
    const auto ret = i2c_write(&i2cdev, reg_addr, reg_data, wr_len);
    if (ret == -1) {
        return BNO055_ERROR;
    }

    return BNO055_SUCCESS;
}

static int8_t jetson_i2c_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t r_len) {
    if (reg_data == nullptr) {
        return BNO055_ERROR;
    }
    const int readRes = i2c_read(&i2cdev, reg_addr, reg_data, r_len);
    if (readRes == -1) {
        return BNO055_ERROR;
    }
    return BNO055_SUCCESS;
}

static void jetson_delay_ms(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

/***************************************************************/
/**\name    BNO055 WRAPPER FUNCTIONS           */
/***************************************************************/

imu::BNO055::BNO055(std::string_view busName, uint8_t devAddr, imu::BNO055::PowerMode powerMode, imu::BNO055::OperationMode operationMode) {
    if (!init(busName, devAddr)) {
        throw std::runtime_error("Connection to the BNO055 device could not be accomplished!\n");
    }
    constexpr uint32_t initSleepDur = 10;
    jetson_delay_ms(initSleepDur);

    bool res = setPowerMode(powerMode);
    jetson_delay_ms(initSleepDur);

    res &= setOperationMode(operationMode);
    jetson_delay_ms(initSleepDur);
    if (!res) {
        throw std::runtime_error("BNO055 device was connected but could not be configured!\n");
    }
}

bool imu::BNO055::init(std::string_view busName, uint8_t devAddr) {
    int bus = i2c_open(busName.data());
    if (bus == -1) {
        std::cout << "Error! I2C device " << busName << " could not be opened!\n";
        return false;
    }

    i2c_init_device(&i2cdev);
    i2cdev.bus = bus;
    i2cdev.addr = devAddr;
    i2cdev.iaddr_bytes = 1;
    i2cdev.page_bytes = I2C_BUFFER_LEN;
    jetson_delay_ms(100);

    bno_.bus_read = jetson_i2c_bus_read;
    bno_.bus_write = jetson_i2c_bus_write;
    bno_.delay_msec = jetson_delay_ms;

    bno_.dev_addr = devAddr;

    if (bno055_init(&bno_) == BNO055_SUCCESS) {
        return false;
    }
    return true;
}

uint8_t imu::BNO055::getSystemCalibrationStatus() {
    uint8_t data {};
    auto res = checkResult(bno055_get_sys_calib_stat(&data));
    if (res != BNO055_SUCCESS) {
        throw std::runtime_error("Communication to the device was failed\n");
    }
    return data;
}

imu::BNO055::Accel imu::BNO055::getAccelMsq() {
    imu::BNO055::Accel data {};
    auto res = checkResult(bno055_convert_double_accel_xyz_msq(&data));
    if (res != BNO055_SUCCESS) {
        throw std::runtime_error("Communication to the device was failed\n");
    }
    return data;
}

imu::BNO055::Accel imu::BNO055::getAccelMg() {
    imu::BNO055::Accel data {};
    auto res = checkResult(bno055_convert_double_accel_xyz_mg(&data));
    if (res != BNO055_SUCCESS) {
        throw std::runtime_error("Communication to the device was failed\n");
    }
    return data;
}

imu::BNO055::LinearAccel imu::BNO055::getLinearAccelMsq() {
    imu::BNO055::LinearAccel data {};
    auto res = checkResult(bno055_convert_double_linear_accel_xyz_msq(&data));
    if (res != BNO055_SUCCESS) {
        throw std::runtime_error("Communication to the device was failed\n");
    }
    return data;
}

imu::BNO055::Gyro imu::BNO055::getGyroDps() {
    imu::BNO055::Gyro data {};
    auto res = checkResult(bno055_convert_double_gyro_xyz_dps(&data));
    if (res != BNO055_SUCCESS) {
        throw std::runtime_error("Communication to the device was failed\n");
    }
    return data;
}

imu::BNO055::Gyro imu::BNO055::getGyroRps() {
    imu::BNO055::Gyro data {};
    auto res = checkResult(bno055_convert_double_gyro_xyz_rps(&data));
    if (res != BNO055_SUCCESS) {
        throw std::runtime_error("Communication to the device was failed\n");
    }
    return data;
}

imu::BNO055::Mag imu::BNO055::getMagUT() {
    imu::BNO055::Mag data {};
    auto res = checkResult(bno055_convert_double_mag_xyz_uT(&data));
    if (res != BNO055_SUCCESS) {
        throw std::runtime_error("Communication to the device was failed\n");
    }
    return data;
}

imu::BNO055::Quaternion imu::BNO055::getQuaternion() {
    imu::BNO055::Quaternion data {};
    auto res = checkResult(bno055_read_quaternion_wxyz(&data));
    if (res != BNO055_SUCCESS) {
        throw std::runtime_error("Communication to the device was failed\n");
    }
    return data;
}

imu::BNO055::EulerAngles imu::BNO055::getEulerAngles() {
    imu::BNO055::EulerAngles data {};
    auto res = checkResult(bno055_convert_double_euler_hpr_deg(&data));
    if (res != BNO055_SUCCESS) {
        throw std::runtime_error("Communication to the device was failed\n");
    }
    return data;
}

imu::BNO055::EulerRads imu::BNO055::getEulerRads() {
    imu::BNO055::EulerRads data {};
    auto res = checkResult(bno055_convert_double_euler_hpr_rad(&data));
    if (res != BNO055_SUCCESS) {
        throw std::runtime_error("Communication to the device was failed\n");
    }
    return data;
}


imu::BNO055::~BNO055() {
    i2c_close(i2cdev.bus);
}