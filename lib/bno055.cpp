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

imu::BNO055::BNO055(std::string_view busName, uint8_t devAddr) {
    if (!init(busName, devAddr)) {
        throw std::invalid_argument("Connection to the BNO055 device could not be accomplished!\n");
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

std::pair<uint8_t, bool> imu::BNO055::getSystemCalibrationStatus() {
    std::pair<uint8_t, bool> data {};
    data.second = checkResult(bno055_get_sys_calib_stat(&data.first));
    return data;
}

std::pair<imu::BNO055::Accel, bool> imu::BNO055::getAccelMsq() {
    std::pair<imu::BNO055::Accel, bool> data {};
    data.second = checkResult(bno055_convert_double_accel_xyz_msq(&data.first));
    return data;
}

std::pair<imu::BNO055::Accel, bool> imu::BNO055::getAccelMg() {
    std::pair<imu::BNO055::Accel, bool> data {};
    data.second = checkResult(bno055_convert_double_accel_xyz_mg(&data.first));
    return data;
}

std::pair<imu::BNO055::LinearAccel, bool> imu::BNO055::getLinearAccelMsq() {
    std::pair<imu::BNO055::LinearAccel, bool> data {};
    data.second = checkResult(bno055_convert_double_linear_accel_xyz_msq(&data.first));
    return data;
}

std::pair<imu::BNO055::Gyro, bool> imu::BNO055::getGyroDps() {
    std::pair<imu::BNO055::Gyro, bool> data {};
    data.second = checkResult(bno055_convert_double_gyro_xyz_dps(&data.first));
    return data;
}

std::pair<imu::BNO055::Gyro, bool> imu::BNO055::getGyroRps() {
    std::pair<imu::BNO055::Gyro, bool> data {};
    data.second = checkResult(bno055_convert_double_gyro_xyz_rps(&data.first));
    return data;
}

std::pair<imu::BNO055::Mag, bool> imu::BNO055::getMagUT() {
    std::pair<imu::BNO055::Mag, bool> data {};
    data.second = checkResult(bno055_convert_double_mag_xyz_uT(&data.first));
    return data;
}

std::pair<imu::BNO055::Quaternion, bool> imu::BNO055::getQuaternion() {
    std::pair<imu::BNO055::Quaternion, bool> data {};
    data.second = checkResult(bno055_read_quaternion_wxyz(&data.first));
    return data;
}

std::pair<imu::BNO055::EulerAngles, bool> imu::BNO055::getEulerAngles() {
    std::pair<imu::BNO055::EulerAngles, bool> data {};
    data.second = checkResult(bno055_convert_double_euler_hpr_deg(&data.first));
    return data;
}

std::pair<imu::BNO055::EulerRads, bool> imu::BNO055::getEulerRads() {
    std::pair<imu::BNO055::EulerRads, bool> data {};
    data.second = checkResult(bno055_convert_double_euler_hpr_rad(&data.first));
    return data;
}


imu::BNO055::~BNO055() {
    i2c_close(i2cdev.bus);
}