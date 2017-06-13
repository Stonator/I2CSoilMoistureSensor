/*----------------------------------------------------------------------*
 * SlowSoftI2CSoilMoistureSensor.h                                      *
 * Arduino library using software I2C bus for the Sensor version        *
 * of I2C Soil Moisture Sensor version from Chirp                       *
 * (https://github.com/Miceuz/i2c-moisture-sensor).                     *
 *                                                                      *
 * https://github.com/Stonator/I2CSoilMoistureSensor                    *
 *                                                                      *
 *----------------------------------------------------------------------*/

#ifndef SLOWSOFTI2CSOILMOISTURESENSOR_H
#define SLOWSOFTI2CSOILMOISTURESENSOR_H

#include "I2CSoilMoistureSensorBase.h"
#include <SlowSoftI2CMaster.h>


// Implementation using SlowSoftI2CMaster found on https://github.com/Stonator/SlowSoftI2CMaster
// The whole implementation is done in the header file to ensure minimal footprint.
// The class and depending libraries are only compiled and linked when included.
class SlowSoftI2CSoilMoistureSensor : public I2CSoilMoistureSensorBase
{
public:
  // @param sda Arduino Pin used for I2C SDA
  // @param scl Arduino Pin used for I2C SCL
  // @param internal_pullup Use internal pull-up resistors, be careful when using it and consider it as a potential source of errors.
  // @param usec_delay Configure the speed of the I2C bus. Rough numbers: 4usec -> ~60kHz, 10usec -> ~25kHz, 25usec -> ~10kHz.
  // @param address Sensor I2C address
  SlowSoftI2CSoilMoistureSensor(uint8_t sda, uint8_t scl, bool internal_pullup, unsigned int usec_delay, uint8_t address = SOILMOISTURESENSOR_DEFAULT_ADDR)
    : I2CSoilMoistureSensorBase(address)
    , m_sda(sda)
    , m_scl(scl)
    , m_internal_pullup(internal_pullup)
    , m_usec_delay(usec_delay)
    , m_i2cMaster(nullptr)
  {
  }

public:
  // Initialize software I2C bus. Needs to be called before any other sensor methods.
  // Returns true if successful.
  // Returns false if SDA or SCL are low, which probably means a I2C bus lockup or that the lines are not pulled up.
  bool i2cBegin()
  {
    if(m_i2cMaster != nullptr)
      return false;
    m_i2cMaster = new SlowSoftI2CMaster(m_sda, m_scl, m_internal_pullup, m_usec_delay);

    if(!m_i2cMaster->i2c_init())
    {
      i2cEnd();
      return false;
    }
    return true;
  }

  // Shutdown I2C bus and end communication
  void i2cEnd()
  {
    if(m_i2cMaster != nullptr)
    {
      delete(m_i2cMaster);
      m_i2cMaster = nullptr;
    }
  }

  // Returns true if I2C bus is up and running, false otherwise
  bool i2cInitialized()
  {
    return(m_i2cMaster != nullptr);
  }


  // Abstraction layer to read and write sensor registers
protected:

  // Helper method to write an 8 bit value to the sensor via I2C
  virtual bool writeI2CRegister8bit(uint8_t address, uint8_t value)
  {
    if(m_i2cMaster == nullptr)
      return(false);
    if(!m_i2cMaster->i2c_start((address << 1) | I2C_WRITE))
      return(false);
    if(!m_i2cMaster->i2c_write(value))
      return(false);
    m_i2cMaster->i2c_stop();
    return(true);
  }

  // Helper method to write an 8 bit value to the sensor via I2C to the given register
  virtual bool writeI2CRegister8bit(uint8_t address, uint8_t reg, uint8_t value)
  {
    if(m_i2cMaster == nullptr)
      return(false);
    if(!m_i2cMaster->i2c_start((address << 1) | I2C_WRITE))
      return(false);
    if(!m_i2cMaster->i2c_write(reg))
      return(false);
    if(!m_i2cMaster->i2c_write(value))
      return(false);
    m_i2cMaster->i2c_stop();
    return(true);
  }

  // Helper method to read a 8 bit value from the given register
  virtual uint8_t readI2CRegister8bit(uint8_t address, uint8_t reg)
  {
    if(m_i2cMaster == nullptr)
      return(0xFF);

    // write
    if(!m_i2cMaster->i2c_start((address << 1) | I2C_WRITE))
      return(0xFF);
    if(!m_i2cMaster->i2c_write(reg))
      return(0xFF);

    // read 1 byte
    delay(20);
    if(!m_i2cMaster->i2c_rep_start((address << 1) | I2C_READ))
      return(0xFF);
    uint8_t value = (uint16_t)m_i2cMaster->i2c_read(true);
    m_i2cMaster->i2c_stop();
    return(value);
  }

  // Helper method to read a 16 bit unsigned value from the given register
  virtual uint16_t readI2CRegister16bitUnsigned(uint8_t address, uint8_t reg)
  {
    if(m_i2cMaster == nullptr)
      return(0xFFFF);

    // write
    if(!m_i2cMaster->i2c_start((address << 1) | I2C_WRITE))
      return(0xFFFF);
    if(!m_i2cMaster->i2c_write(reg))
      return(0xFFFF);

    // read 2 bytes
    delay(20);
    if(!m_i2cMaster->i2c_rep_start((address << 1) | I2C_READ))
      return(0xFFFF);
    uint16_t value = ((uint16_t)m_i2cMaster->i2c_read(false) << 8) | (uint16_t)m_i2cMaster->i2c_read(true);
    m_i2cMaster->i2c_stop();
    return(value);
  }


protected:
  uint8_t m_sda;
  uint8_t m_scl;
  bool m_internal_pullup;
  unsigned int m_usec_delay;
  SlowSoftI2CMaster *m_i2cMaster;
};

#endif
