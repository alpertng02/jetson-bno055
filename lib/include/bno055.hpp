#ifndef BNO055_HPP
#define BNO055_HPP
extern "C" {   
#include "BNO055_SensorAPI/bno055.h"
}
#include <cstdint>
#include <string>
#include <optional>

// Your code here

namespace imu {

class BNO055 {
public:
    BNO055(const std::string& busName, uint8_t dev_addr);

    enum PowerMode {
        Normal = BNO055_POWER_MODE_NORMAL,
        LowPower = BNO055_POWER_MODE_LOWPOWER,
        Suspend = BNO055_POWER_MODE_SUSPEND,
    };

    bool setPowerMode(BNO055::PowerMode powerMode) {
        return checkResult(bno055_set_power_mode(powerMode));
    }

    enum OperationMode {
        Config = BNO055_OPERATION_MODE_CONFIG,
        AccelOnly = BNO055_OPERATION_MODE_ACCONLY,
        MagOnly = BNO055_OPERATION_MODE_MAGONLY,
        GyroOnly = BNO055_OPERATION_MODE_GYRONLY,
        AccelMag = BNO055_OPERATION_MODE_ACCMAG,
        AccelGyro = BNO055_OPERATION_MODE_ACCGYRO,
        MagGyro = BNO055_OPERATION_MODE_MAGGYRO,
        ImuPlus = BNO055_OPERATION_MODE_IMUPLUS,
        Compass = BNO055_OPERATION_MODE_COMPASS,
        M4G = BNO055_OPERATION_MODE_M4G,
        NDOF_FMC_OFF = BNO055_OPERATION_MODE_NDOF_FMC_OFF,
        NDOF = BNO055_OPERATION_MODE_NDOF,

    };
    bool setOperationMode(BNO055::OperationMode operationMode) {
        return checkResult(bno055_set_operation_mode(operationMode));
    }

    std::pair<uint8_t, bool> getSystemCalibrationStatus() {
        uint8_t calibStatus {};
        bool res = checkResult(bno055_get_sys_calib_stat(&calibStatus));
        return { calibStatus, res };
    }

    using Accel = bno055_accel_double_t;
    using LinearAccel = bno055_linear_accel_double_t;
    using Gyro = bno055_gyro_double_t;
    using Mag = bno055_mag_double_t;

    using EulerAngles = bno055_euler_double_t;
    using EulerRads = bno055_euler_double_t;
    using Quaternion = bno055_quaternion_t;

    std::pair<Accel, bool> getAccelMsq() {
        std::pair<Accel, bool> data {};
        data.second = bno055_convert_double_accel_xyz_msq(&data.first);
        return data;
    }

    std::pair<Accel, bool> getAccelMg() {
        std::pair<Accel, bool> data {};
        data.second = bno055_convert_double_accel_xyz_mg(&data.first);
        return data;
    }

    std::pair<LinearAccel, bool> getLinearAccelMsq() {
        std::pair<LinearAccel, bool> data {};
        data.second = bno055_convert_double_linear_accel_xyz_msq(&data.first);
        return data;
    }

    std::pair<Quaternion, bool> getQuaternion() {
        std::pair<Quaternion, bool> data {};
        data.second = bno055_read_quaternion_wxyz(&data.first);
        return data;
    }

    std::pair<EulerAngles, bool> getEulerAngles() {
        std::pair<EulerAngles, bool> data {};
        data.second = bno055_convert_double_euler_hpr_deg(&data.first);
        return data;
    }

    std::pair<EulerRads, bool> getEulerRads() {
        std::pair<EulerRads, bool> data {};
        data.second = bno055_convert_double_euler_hpr_rad(&data.first);
        return data;
    }

    bool isOpen() {
        return isOpen_;
    }

private:
    bno055_t bno_ {};
    bool isOpen_ = false;

    bool checkResult(int8_t res) {
        return res == BNO055_SUCCESS ? true : false;
    }

};

} // namespace imu
#endif // BNO055_HPP