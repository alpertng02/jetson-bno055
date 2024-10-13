#include "bno055.hpp"
#include "i2c/i2c.h"
#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

/***************************************************************/
/**\name    BUS READ AND WRITE FUNCTIONS           */
/***************************************************************/
static I2CDevice i2cdev {};
static constexpr size_t I2C_BUFFER_LEN = 16;

static int8_t jetson_i2c_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t wr_len) {
    if (reg_data == nullptr) {
        return BNO055_ERROR;
    }

    uint8_t msg[I2C_BUFFER_LEN] { reg_addr };
    std::memcpy(msg + 1, reg_data, wr_len);

    // Write data to register(s) over I2C

    const auto ret = i2c_write(&i2cdev, dev_addr, msg, (wr_len + 1));
    if (ret == -1) {
        return BNO055_ERROR;
    }

    return BNO055_SUCCESS;
}

static int8_t jetson_i2c_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t r_len) {
    if (reg_data == nullptr) {
        return BNO055_ERROR;
    }
    const auto writeRes = i2c_write(&i2cdev, dev_addr, &reg_addr, 1);
    if (writeRes == -1) {
        return BNO055_ERROR;
    }

    const int readRes = i2c_read(&i2cdev, dev_addr, reg_data, r_len);
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

imu::BNO055::BNO055(const std::string& busName, uint8_t dev_addr) {
    int bus = i2c_open(busName.c_str());
    if (bus == -1) {
        std::cout << "Error! I2C device " << busName << " could not be opened!\n";
        return;
    }

    i2c_init_device(&i2cdev);
    i2cdev.bus = bus;
    i2cdev.addr = dev_addr;
    i2cdev.iaddr_bytes = 1;
    i2cdev.page_bytes = I2C_BUFFER_LEN;
    jetson_delay_ms(100);

    bno_.bus_read = jetson_i2c_bus_read;
    bno_.bus_write = jetson_i2c_bus_write;
    bno_.delay_msec = jetson_delay_ms;

    bno_.dev_addr = dev_addr;

    if (bno055_init(&bno_) == BNO055_SUCCESS) {
        isOpen_ = true;
    }
}