#include <linux/i2c-dev.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <system_error>
#include "imu.h"
#include "gpio.h"

using namespace std;

const char* const Imu::I2C_DEV="/dev/i2c-1";

inline int i2c_smbus_read_block_data(int file, uint8_t command,
                                     uint8_t size ,uint8_t *values)
{
    union i2c_smbus_data data;
    data.block[0] = size;
    int result;
    if ((result = i2c_smbus_access(file, I2C_SMBUS_READ, command,
                         I2C_SMBUS_I2C_BLOCK_DATA, &data)))
        return result;
    else
    {
        for (int i = 1; i <= size; i++)
            values[i - 1] = data.block[i];
        return result;
    }
}

Imu::Imu(int interruptPin) : intPin(interruptPin, Direction::IN)
{
    i2c_fd = open(I2C_DEV, O_RDWR);
    if (i2c_fd < 0) {
        throw runtime_error("failed to open i2c device");
    }
    if (ioctl(i2c_fd, I2C_SLAVE, IMU_DEVID) < 0) {
        throw runtime_error("failed to select imu address");
    }
}

Imu::~Imu()
{
    reset();
}

void Imu::enable()
{
    i2c_smbus_write_byte_data(i2c_fd, REG_PWR_MGMT_1, 0x01); //Using Clock Source: PLL with X axis gyroscope reference
}

void Imu::reset()
{
    i2c_smbus_write_byte_data(i2c_fd, REG_PWR_MGMT_1, 0x40+0x80); //Sleep and Device_Reset
}

double Imu::setSampleRate(double hz, uint8_t dlpfMode)
{
    uint8_t dlpf_cfg = dlpfMode & 0x07;
    i2c_smbus_write_byte_data(i2c_fd, REG_CONFIG, dlpf_cfg);

    double baseRate = dlpf_cfg == 0x00 || dlpf_cfg == 0x07 ? 8000.0 : 1000.0;
    uint8_t smprt_div = baseRate / hz - 1;
    i2c_smbus_write_byte_data(i2c_fd, REG_SMPRT_DIV, smprt_div);

    return baseRate / (smprt_div + 1);
}

double Imu::readScaledInt16(uint8_t reg, double scale)
{
    int rawData = i2c_smbus_read_word_data(i2c_fd, reg);
    int16_t data = rawData >> 8 | rawData << 8;
    return data * scale;
}

struct GyroRawData
{
    uint8_t GYRO_XOUT_H;
    uint8_t GYRO_XOUT_L;
    uint8_t GYRO_YOUT_H;
    uint8_t GYRO_YOUT_L;
    uint8_t GYRO_ZOUT_H;
    uint8_t GYRO_ZOUT_L;
};

struct ImuRawData {
    uint8_t ACCEL_XOUT_H;
    uint8_t ACCEL_XOUT_L;
    uint8_t ACCEL_YOUT_H;
    uint8_t ACCEL_YOUT_L;
    uint8_t ACCEL_ZOUT_H;
    uint8_t ACCEL_ZOUT_L;

    uint8_t TEMP_OUT_H;
    uint8_t TEMP_OUT_L;

    GyroRawData GyroData;
};
static_assert(sizeof(ImuRawData) == 14, "Imu raw data should be 14 byte long");

double Imu::processRawData(uint8_t h, uint8_t l, double scale, double offset)
{
    int16_t data = h << 8 | l;
    return data * scale + offset;
}

void Imu::processGyroRawData(const GyroRawData &rawData, GyroData &data)
{
    data.GyroX = processRawData(rawData.GYRO_XOUT_H, rawData.GYRO_XOUT_L, GYRO_SCALE);
    data.GyroY = processRawData(rawData.GYRO_YOUT_H, rawData.GYRO_YOUT_L, GYRO_SCALE);
    data.GyroZ = processRawData(rawData.GYRO_ZOUT_H, rawData.GYRO_ZOUT_L, GYRO_SCALE);
}

GyroData Imu::readGyro()
{
    GyroRawData rawData;
    i2c_smbus_read_block_data(i2c_fd, REG_GYRO_XOUT_H, sizeof(GyroRawData), (uint8_t *)&rawData);
    GyroData data;
    processGyroRawData(rawData, data);
    return data;
}

ImuData Imu::readAll()
{
    ImuRawData rawData;
    i2c_smbus_read_block_data(i2c_fd, REG_ACCEL_XOUT_H, sizeof(rawData), (uint8_t *)&rawData);
    ImuData data;
    data.AccelX = processRawData(rawData.ACCEL_XOUT_H, rawData.ACCEL_XOUT_L, ACCEL_SCALE);
    data.AccelY = processRawData(rawData.ACCEL_YOUT_H, rawData.ACCEL_YOUT_L, ACCEL_SCALE);
    data.AccelZ = processRawData(rawData.ACCEL_ZOUT_H, rawData.ACCEL_ZOUT_L, ACCEL_SCALE);
    data.Temp = processRawData(rawData.TEMP_OUT_H, rawData.TEMP_OUT_L, TEMP_SCALE, TEMP_OFFSET);
    processGyroRawData(rawData.GyroData, data);
    return data;
}

void Imu::enableDataReadyInterrupt(function<void()> dataReady)
{
    uint8_t int_enable = 0x01;
    i2c_smbus_write_byte_data(i2c_fd, REG_INT_ENABLE, int_enable);

    intPin.EnableISR(Edge::RISING, [dataReady](const DigitalValue &) { dataReady(); });
}
