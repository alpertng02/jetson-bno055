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
    enum PowerMode {
        Normal = BNO055_POWER_MODE_NORMAL,
        LowPower = BNO055_POWER_MODE_LOWPOWER,
        Suspend = BNO055_POWER_MODE_SUSPEND,
    };
    
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

    BNO055() = default;
    BNO055(std::string_view busName, uint8_t dev_addr, PowerMode powerMode = Normal, OperationMode operationMode = NDOF);

    bool init(std::string_view busName, uint8_t devAddr);

    bool reconnect();

    bool setPowerMode(BNO055::PowerMode powerMode) {
        return checkResult(bno055_set_power_mode(powerMode));
    }

    bool setOperationMode(BNO055::OperationMode operationMode) {
        return checkResult(bno055_set_operation_mode(operationMode));
    }

    using Accel = bno055_accel_double_t;
    using LinearAccel = bno055_linear_accel_double_t;
    using Gyro = bno055_gyro_double_t;
    using Mag = bno055_mag_double_t;

    using EulerAngles = bno055_euler_double_t;
    using EulerRads = bno055_euler_double_t;

    struct Quaternion {
        double w {};
        double x {};
        double y {};
        double z {};
    };

    uint8_t getSystemCalibrationStatus();

    Accel getAccelMsq();
    Accel getAccelMg();

    LinearAccel getLinearAccelMsq();

    Gyro getGyroDps();
    Gyro getGyroRps();

    Mag getMagUT();

    Quaternion getQuaternion();

    EulerAngles getEulerAngles();
    EulerRads getEulerRads();

    ~BNO055();
private:
    bno055_t bno_ {};
    std::string busName_ {};
    uint8_t devAddr_ {};

    bool checkResult(int8_t res) {
        return res == BNO055_SUCCESS ? true : false;
    }

};

} // namespace imu
#endif // BNO055_HPP