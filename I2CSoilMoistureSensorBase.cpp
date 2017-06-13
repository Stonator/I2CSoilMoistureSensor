/*----------------------------------------------------------------------*
 * I2CSoilMoistureSensorBase.cpp - Arduino library for the Sensor version of*
 * I2C Soil Moisture Sensor version from Chirp                          *
 * (https://github.com/Miceuz/i2c-moisture-sensor).                     *
 *                                                                      *
 * Ingo Fischer 11Nov2015                                               *
 * https://github.com/Apollon77/I2CSoilMoistureSensor                   *
 *                                                                      *
 * MIT license                                                          *
 *----------------------------------------------------------------------*/

#include "I2CSoilMoistureSensorBase.h"


I2CSoilMoistureSensorBase::I2CSoilMoistureSensorBase(uint8_t address) : sensorAddress(address) {
  // nothing to do ... Wire.begin needs to be put outside of class
}

bool I2CSoilMoistureSensorBase::begin(bool wait)
{
  if(!resetSensor())
    return false;
  if(wait)
    delay(1000);
  return true;
}

bool I2CSoilMoistureSensorBase::setAddress(uint8_t address, bool reset)
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

bool I2CSoilMoistureSensorBase::changeSensor(uint8_t address, bool wait)
{
  sensorAddress = address;
  return begin(wait);
}

uint8_t I2CSoilMoistureSensorBase::getAddress()
{
  return sensorAddress;
}

bool I2CSoilMoistureSensorBase::resetSensor()
{
  return writeI2CRegister8bit(sensorAddress, SOILMOISTURESENSOR_RESET);
}

uint8_t I2CSoilMoistureSensorBase::getVersion()
{
  return readI2CRegister8bit(sensorAddress, SOILMOISTURESENSOR_GET_VERSION);
}

bool I2CSoilMoistureSensorBase::sleep()
{
  return writeI2CRegister8bit(sensorAddress, SOILMOISTURESENSOR_SLEEP);
}

bool I2CSoilMoistureSensorBase::isBusy()
{
  return (readI2CRegister8bit(sensorAddress, SOILMOISTURESENSOR_GET_BUSY) == 1);
}

unsigned int I2CSoilMoistureSensorBase::getCapacitance()
{
  return readI2CRegister16bitUnsigned(sensorAddress, SOILMOISTURESENSOR_GET_CAPACITANCE);
}

bool I2CSoilMoistureSensorBase::startMeasureLight()
{
  return writeI2CRegister8bit(sensorAddress, SOILMOISTURESENSOR_MEASURE_LIGHT);
}

unsigned int I2CSoilMoistureSensorBase::getLight(bool wait)
{
  if(wait)
  {
    if(!startMeasureLight())
      return(0xFFFF);
    delay(3000);
  }
  return readI2CRegister16bitUnsigned(sensorAddress, SOILMOISTURESENSOR_GET_LIGHT);
}

int I2CSoilMoistureSensorBase::getTemperature()
{
  return readI2CRegister16bitSigned(sensorAddress, SOILMOISTURESENSOR_GET_TEMPERATURE);
}


int16_t I2CSoilMoistureSensorBase::readI2CRegister16bitSigned(uint8_t address, uint8_t reg)
{
  return (int16_t)readI2CRegister16bitUnsigned(address, reg);
}

