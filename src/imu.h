#include <wiringPi.h>
#include <limits>
#include <cstdint>

struct ImuData{
  double AccelX;
  double AccelY;
  double AccelZ;

  double Temp;

  double GyroX;
  double GyroY;
  double GyroZ;
};

class Imu
{
  public:
    static const char *const I2C_DEV;
    static const int IMU_DEVID = 0x68;

    static const int INTERRUPT_PIN = 14;

    void enable();
    void reset();
    double setSampleRate(double hz, uint8_t dlpfMode=0x01);

    Imu();

    inline double readAccelX() { return readAccelValue(REG_ACCEL_XOUT_H); }
    inline double readAccelY() { return readAccelValue(REG_ACCEL_YOUT_H); }
    inline double readAccelZ() { return readAccelValue(REG_ACCEL_ZOUT_H); }

    inline double readGyroX() { return readGyroValue(REG_GYRO_XOUT_H); }
    inline double readGyroY() { return readGyroValue(REG_GYRO_YOUT_H); }
    inline double readGyroZ() { return readGyroValue(REG_GYRO_ZOUT_H); }

    ImuData readAll();

    void enableDataReadyInterrupt(void (*dataReady)(void));
    
  private:
    constexpr static const double ACCEL_SCALE = 2.0 / std::numeric_limits<int16_t>::max();
    constexpr static const double GYRO_SCALE = 250.0 / std::numeric_limits<int16_t>::max();
    constexpr static const double TEMP_SCALE = 1.0 / 340.0;
    constexpr static const double TEMP_OFFSET = 36.53;

    static const uint8_t
        REG_SMPRT_DIV = 0x19,
        REG_CONFIG = 0x1A,
        REG_INT_ENABLE = 0x38,
        REG_ACCEL_XOUT_H = 0x3B,
        REG_ACCEL_YOUT_H = 0x3D,
        REG_ACCEL_ZOUT_H = 0x3F,
        REG_GYRO_XOUT_H = 0x43,
        REG_GYRO_YOUT_H = 0x45,
        REG_GYRO_ZOUT_H = 0x47,
        REG_PWR_MGMT_1 = 0x6B;

    int i2c_fd;
    double readScaledInt16(uint8_t reg, double scale);

    inline double readAccelValue(uint8_t reg) { return readScaledInt16(reg, ACCEL_SCALE); }
    inline double readGyroValue(uint8_t reg) { return readScaledInt16(reg, GYRO_SCALE); }
    double processRawData(uint8_t h, uint8_t l, double scale, double offset = 0);
};
