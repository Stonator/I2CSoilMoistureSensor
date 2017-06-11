/*----------------------------------------------------------------------*
 * I2CSoilMoistureSensor.cpp - Arduino library for the Sensor version of*
 * I2C Soil Moisture Sensor version from Chirp                          *
 * (https://github.com/Miceuz/i2c-moisture-sensor).                     *
 *                                                                      *
 * Ingo Fischer 11Nov2015                                               *
 * https://github.com/Apollon77/I2CSoilMoistureSensor                   *
 *                                                                      *
 * MIT license                                                          *
 *----------------------------------------------------------------------*/

#include "I2CSoilMoistureSensor.h"

// define release-independent I2C functions
#if defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#include <TinyWireM.h>
#define i2cBegin TinyWireM.begin
#define i2cBeginTransmission TinyWireM.beginTransmission
#define i2cEndTransmission TinyWireM.endTransmission
#define i2cRequestFrom TinyWireM.requestFrom
#define i2cRead TinyWireM.receive
#define i2cWrite TinyWireM.send
#elif ARDUINO >= 100
#include <Wire.h>
#define i2cBegin Wire.begin
#define i2cBeginTransmission Wire.beginTransmission
#define i2cEndTransmission Wire.endTransmission
#define i2cRequestFrom Wire.requestFrom
#define i2cRead Wire.read
#define i2cWrite Wire.write
#else
#include <Wire.h>
#define i2cBegin Wire.begin
#define i2cBeginTransmission Wire.beginTransmission
#define i2cEndTransmission Wire.endTransmission
#define i2cRequestFrom Wire.requestFrom
#define i2cRead Wire.receive
#define i2cWrite Wire.send
#endif

I2CSoilMoistureSensor::I2CSoilMoistureSensor(uint8_t address) : sensorAddress(address) {
  // nothing to do ... Wire.begin needs to be put outside of class
}

bool I2CSoilMoistureSensor::begin(bool wait)
{
  if(!resetSensor())
    return false;
  if(wait)
    delay(1000);
  return true;
}

bool I2CSoilMoistureSensor::setAddress(uint8_t address, bool reset)
{
  if(!writeI2CRegister8bit(sensorAddress, SOILMOISTURESENSOR_SET_ADDRESS, address))
    return false;
  if(reset)
  {
    if(!resetSensor())
      return false;
    delay(1000);
  }
  sensorAddress = address;
  return (readI2CRegister8bit(sensorAddress, SOILMOISTURESENSOR_GET_ADDRESS) == address);
}

bool I2CSoilMoistureSensor::changeSensor(uint8_t address, bool wait)
{
  sensorAddress = address;
  return begin(wait);
}

uint8_t I2CSoilMoistureSensor::getAddress()
{
  return sensorAddress;
}

bool I2CSoilMoistureSensor::resetSensor()
{
  return writeI2CRegister8bit(sensorAddress, SOILMOISTURESENSOR_RESET);
}

uint8_t I2CSoilMoistureSensor::getVersion()
{
  return readI2CRegister8bit(sensorAddress, SOILMOISTURESENSOR_GET_VERSION);
}

bool I2CSoilMoistureSensor::sleep()
{
  return writeI2CRegister8bit(sensorAddress, SOILMOISTURESENSOR_SLEEP);
}

bool I2CSoilMoistureSensor::isBusy()
{
  return (readI2CRegister8bit(sensorAddress, SOILMOISTURESENSOR_GET_BUSY) == 1);
}

unsigned int I2CSoilMoistureSensor::getCapacitance()
{
  return readI2CRegister16bitUnsigned(sensorAddress, SOILMOISTURESENSOR_GET_CAPACITANCE);
}

bool I2CSoilMoistureSensor::startMeasureLight()
{
  return writeI2CRegister8bit(sensorAddress, SOILMOISTURESENSOR_MEASURE_LIGHT);
}

unsigned int I2CSoilMoistureSensor::getLight(bool wait)
{
  if(wait)
  {
    if(!startMeasureLight())
      return(0xFFFF);
    delay(3000);
  }
  return readI2CRegister16bitUnsigned(sensorAddress, SOILMOISTURESENSOR_GET_LIGHT);
}

int I2CSoilMoistureSensor::getTemperature()
{
  return readI2CRegister16bitSigned(sensorAddress, SOILMOISTURESENSOR_GET_TEMPERATURE);
}


//----------------------------------------------------------------------------------------------------
// Abstraction layer to read and write sensor registers
//----------------------------------------------------------------------------------------------------

bool I2CSoilMoistureSensor::writeI2CRegister8bit(uint8_t address, uint8_t value)
{
  i2cBeginTransmission(address);
  i2cWrite(value);
  i2cEndTransmission();
  return(true);
}

bool I2CSoilMoistureSensor::writeI2CRegister8bit(uint8_t address, uint8_t reg, uint8_t value)
{
  i2cBeginTransmission(address);
  i2cWrite(reg);
  i2cWrite(value);
  i2cEndTransmission();
  return(true);
}

uint16_t I2CSoilMoistureSensor::readI2CRegister16bitUnsigned(uint8_t address, uint8_t reg)
{
  uint16_t value;

  i2cBeginTransmission(address);
  i2cWrite(reg);
  i2cEndTransmission();
  delay(20);
  i2cRequestFrom(address, (uint8_t)2);
  value = (i2cRead() << 8) | i2cRead();
  i2cEndTransmission();

  return value;
}

int16_t I2CSoilMoistureSensor::readI2CRegister16bitSigned(uint8_t address, uint8_t reg)
{
  return (int16_t)readI2CRegister16bitUnsigned(address, reg);
}

uint8_t I2CSoilMoistureSensor::readI2CRegister8bit(uint8_t address, uint8_t reg)
{
  i2cBeginTransmission(address);
  i2cWrite(reg);
  i2cEndTransmission();
  delay(20);
  i2cRequestFrom(address, (uint8_t)1);
  return i2cRead();
}

