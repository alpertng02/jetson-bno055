#ifndef BNO055_HPP
#define BNO055_HPP
extern "C" {
#include "BNO055_SensorAPI/bno055.h"
}
#include <cstdint>
#include <string_view>
#include <optional>

// Your code here

namespace imu {

class BNO055 {
public:
    BNO055() = default;
    BNO055(std::string_view busName, uint8_t dev_addr);

    bool init(std::string_view busName, uint8_t devAddr);

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


    using Accel = bno055_accel_double_t;
    using LinearAccel = bno055_linear_accel_double_t;
    using Gyro = bno055_gyro_double_t;
    using Mag = bno055_mag_double_t;

    using EulerAngles = bno055_euler_double_t;
    using EulerRads = bno055_euler_double_t;
    using Quaternion = bno055_quaternion_t;

    std::pair<uint8_t, bool> getSystemCalibrationStatus();

    std::pair<Accel, bool> getAccelMsq();
    std::pair<Accel, bool> getAccelMg();

    std::pair<LinearAccel, bool> getLinearAccelMsq();

    std::pair<Gyro, bool> getGyroDps();
    std::pair<Gyro, bool> getGyroRps();

    std::pair<Mag, bool> getMagUT();

    std::pair<Quaternion, bool> getQuaternion();

    std::pair<EulerAngles, bool> getEulerAngles();
    std::pair<EulerRads, bool> getEulerRads();

    ~BNO055();
private:
    bno055_t bno_ {};

    bool checkResult(int8_t res) {
        return res == BNO055_SUCCESS ? true : false;
    }

};

} // namespace imu
#endif // BNO055_HPP