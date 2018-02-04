#include "imu.h"
#include <linux/i2c-dev.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <system_error>

using namespace std;

const char* const Imu::I2C_DEV="/dev/i2c-1";

Imu::Imu()
{
    wiringPiSetupSys();

    i2c_fd = open(I2C_DEV, O_RDWR);
    if (i2c_fd < 0) {
        throw runtime_error("failed to open i2c device");
    }
    if (ioctl(i2c_fd, I2C_SLAVE, IMU_DEVID) < 0) {
        throw runtime_error("failed to select imu address");
    }
}

void Imu::enable()
{
    i2c_smbus_write_byte_data(i2c_fd, REG_PWR_MGMT_1, 0x00);
}

void Imu::reset()
{
    i2c_smbus_write_byte_data(i2c_fd, REG_PWR_MGMT_1, 0x40+0x80);
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

void Imu::enableDataReadyInterrupt(void (*dataReady)(void))
{
    uint8_t int_enable = 0x01;
    i2c_smbus_write_byte_data(i2c_fd, REG_INT_ENABLE, int_enable);

    wiringPiISR(INTERRUPT_PIN, INT_EDGE_RISING, dataReady);
}
