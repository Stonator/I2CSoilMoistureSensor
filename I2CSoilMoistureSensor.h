/*----------------------------------------------------------------------*
 * I2CSoilMoistureSensor.h - Arduino library for the Sensor version of  *
 * I2C Soil Moisture Sensor version from Chirp                          *
 * (https://github.com/Miceuz/i2c-moisture-sensor).                     *
 *                                                                      *
 * Ingo Fischer 11Nov2015                                               *
 * https://github.com/Apollon77/I2CSoilMoistureSensor                   *
 *                                                                      *
 * MIT license                                                          *
 *----------------------------------------------------------------------*/

#ifndef I2CSOILMOISTURESENSOR_H
#define I2CSOILMOISTURESENSOR_H

#include "I2CSoilMoistureSensorBase.h"

// define release-independent I2C functions
#if defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#include <TinyWireM.h>
#define sms_i2cBegin TinyWireM.begin
#define sms_i2cBeginTransmission TinyWireM.beginTransmission
#define sms_i2cEndTransmission TinyWireM.endTransmission
#define sms_i2cRequestFrom TinyWireM.requestFrom
#define sms_i2cRead TinyWireM.receive
#define sms_i2cWrite TinyWireM.send
#elif ARDUINO >= 100
#include <Wire.h>
#define sms_i2cBegin Wire.begin
#define sms_i2cBeginTransmission Wire.beginTransmission
#define sms_i2cEndTransmission Wire.endTransmission
#define sms_i2cRequestFrom Wire.requestFrom
#define sms_i2cRead Wire.read
#define sms_i2cWrite Wire.write
#else
#include <Wire.h>
#define sms_i2cBegin Wire.begin
#define sms_i2cBeginTransmission Wire.beginTransmission
#define sms_i2cEndTransmission Wire.endTransmission
#define sms_i2cRequestFrom Wire.requestFrom
#define sms_i2cRead Wire.receive
#define sms_i2cWrite Wire.send
#endif


 // Implementation using hardware Wire library based implementation
 // The whole implementation is done in the header file to ensure minimal footprint.
 // The class and depending libraries are only compiled and linked when included.
class I2CSoilMoistureSensor : public I2CSoilMoistureSensorBase
{
public:
  // Optionally set sensor I2C address if different from default
  I2CSoilMoistureSensor(uint8_t address = SOILMOISTURESENSOR_DEFAULT_ADDR) : I2CSoilMoistureSensorBase(address)
  {
    // nothing to do ... Wire.begin needs to be put outside of class
  }


  // Abstraction layer to read and write sensor registers
protected:

  // Helper method to write an 8 bit value to the sensor via I2C
  virtual bool writeI2CRegister8bit(uint8_t address, uint8_t value)
  {
    sms_i2cBeginTransmission(address);
    sms_i2cWrite(value);
    sms_i2cEndTransmission();
    return(true);
  }

  // Helper method to write an 8 bit value to the sensor via I2C to the given register
  virtual bool writeI2CRegister8bit(uint8_t address, uint8_t reg, uint8_t value)
  {
    sms_i2cBeginTransmission(address);
    sms_i2cWrite(reg);
    sms_i2cWrite(value);
    sms_i2cEndTransmission();
    return(true);
  }

  // Helper method to read a 8 bit value from the given register
  virtual uint8_t readI2CRegister8bit(uint8_t address, uint8_t reg)
  {
    sms_i2cBeginTransmission(address);
    sms_i2cWrite(reg);
    sms_i2cEndTransmission();
    delay(20);
    sms_i2cRequestFrom(address, (uint8_t)1);
    return sms_i2cRead();
  }

  // Helper method to read a 16 bit unsigned value from the given register
  virtual uint16_t readI2CRegister16bitUnsigned(uint8_t address, uint8_t reg)
  {
    uint16_t value;

    sms_i2cBeginTransmission(address);
    sms_i2cWrite(reg);
    sms_i2cEndTransmission();
    delay(20);
    sms_i2cRequestFrom(address, (uint8_t)2);
    value = (sms_i2cRead() << 8) | sms_i2cRead();
    sms_i2cEndTransmission();

    return value;
  }
};

#endif
