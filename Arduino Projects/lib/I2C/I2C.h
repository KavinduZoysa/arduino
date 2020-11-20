#ifndef _I2C_h_
#define _I2C_h_
#include <stdint.h>

class I2C {
private:
    uint8_t IMUAddress;
    uint16_t I2C_TIMEOUT; // TODO : Define as const
public:
    I2C();

    uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);

    uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop);

    uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes);
};

#endif