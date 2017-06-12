/*----------------------------------------------------------------------*
 * I2CSoilMoistureSensor.h - Arduino library for the Sensor version of  *
 * I2C Soil Moisture Sensor version from Chrirp                         *
 * (https://github.com/Miceuz/i2c-moisture-sensor).                     *
 *                                                                      *
 * Ingo Fischer 11Nov2015                                               *
 * https://github.com/Apollon77/I2CSoilMoistureSensor                   *
 *                                                                      *
 * MIT license                                                          *
 *----------------------------------------------------------------------*/

#ifndef I2CSOILMOISTURESENSOR_H
#define I2CSOILMOISTURESENSOR_H

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h> 
#else
#include <WProgram.h> 
#endif

//Default I2C Address of the sensor
#define SOILMOISTURESENSOR_DEFAULT_ADDR 0x20

//Soil Moisture Sensor Register Addresses
#define SOILMOISTURESENSOR_GET_CAPACITANCE 	0x00 // (r) 	2 bytes
#define SOILMOISTURESENSOR_SET_ADDRESS 		0x01 //	(w) 	1 byte
#define SOILMOISTURESENSOR_GET_ADDRESS 		0x02 // (r) 	1 byte
#define SOILMOISTURESENSOR_MEASURE_LIGHT 	0x03 //	(w) 	n/a
#define SOILMOISTURESENSOR_GET_LIGHT 		0x04 //	(r) 	2 bytes
#define SOILMOISTURESENSOR_GET_TEMPERATURE	0x05 //	(r) 	2 bytes
#define SOILMOISTURESENSOR_RESET 			0x06 //	(w) 	n/a
#define SOILMOISTURESENSOR_GET_VERSION 		0x07 //	(r) 	1 bytes
#define SOILMOISTURESENSOR_SLEEP	        0x08 // (w)     n/a
#define SOILMOISTURESENSOR_GET_BUSY	        0x09 // (r)	    1 bytes


class I2CSoilMoistureSensor
{
public:
  // Optionally set sensor I2C address if different from default
  // Wire.begin needs to be called from outside of the class
  I2CSoilMoistureSensor(uint8_t address = SOILMOISTURESENSOR_DEFAULT_ADDR);


  // Sensor management
public:

  // Initializes anything ... it does a reset.
  // When used without parameter or parameter value is false then a
  // waiting time of at least 1 second is expected to give the sensor some time to boot up.
  // Alternatively use true as parameter and the method waits for a second and returns after that.
  // Returns true if successful, false if communication failed.
  bool begin(bool wait = false);

  // Change I2C address of the sensor to the provided address (1..127)
  // and do a reset after it in order for the new address to become effective if second parameter is true.
  // Method returns true if the new address is set successfully on sensor.
  // Returns true if successful, false if communication failed.
  bool setAddress(uint8_t address, bool reset);

  // Change the address (1..127) this instance is trying to read from and do a reset after to initialize.
  // Returns true if successful, false if communication failed.
  bool changeSensor(uint8_t address, bool wait = false);

  // Return current Address of the Sensor
  uint8_t getAddress();

  // Resets sensor. Give the sensor 0.5-1 second time to boot up after reset.
  // Returns true if successful, false if communication failed.
  bool resetSensor();

  // Get Firmware Version. 0x22 means 2.2
  // Returns 0xFF if communication failed.
  uint8_t getVersion();

  // Sleep sensor. Initiates SLEEP_MODE_PWR_DOWN in the sensor's MCU.
  // Returns true if successful, false if communication failed.
  bool sleep();

  // Check if sensor is busy. Returns true if a measurement is running.
  bool isBusy();


  // Sensor data
public:

  // Return measured Soil Moisture Capacitance
  // Moisture is somewhat linear. More moisture will give you higher reading.
  // Normally all sensors give about 290 - 310 as value in free air at 5V supply.
  // Returns 0xFFFF if communication failed.
  unsigned int getCapacitance();

  // Starts the measurement for the Light sensor. Wait at least 3 seconds till you call method getLight to get the Light value.
  // Returns true if successful, false if communication failed.
  bool startMeasureLight();

  // Read the Light Measurement from the sensor. When used without parameter or parameter value is false then a former call to
  // method startMeasureLight and a waiting time of at least 3 seconds is expected.
  // Alternatively use true as parameter and the method does the call to startMeasureLight and a 3 seconds delay automatically and no former call is needed.                                                      *
  // The measurement gives 65535 in a dark room away form desk lamp - so more light, lower reading.
  // When it's dark, it takes longer to measure light, reading the light register while measurement is in
  // progress (e.g. wait time too short) will return the previous reading. Be aware, light sensor is pretty noisy.
  // Returns 0xFFFF if communication failed.
  unsigned int getLight(bool wait = false);

  // Read the Temperature Measurement. Temperature is measured by the thermistor on the tip of the sensor.
  // Calculated absolute measurement accuracy is better than 2%.
  // The returned value is in degrees Celsius with factor 10, so need to divide by 10 to get real value.
  // Returns -1 if communication failed.
  int getTemperature();


  // Abstraction layer to read and write sensor registers
protected:

  // Helper method to write an 8 bit value to the sensor via I2C
  virtual bool writeI2CRegister8bit(uint8_t address, uint8_t value);

  // Helper method to write an 8 bit value to the sensor via I2C to the given register
  virtual bool writeI2CRegister8bit(uint8_t address, uint8_t reg, uint8_t value);

  // Helper method to read a 16 bit unsigned value from the given register
  virtual uint16_t readI2CRegister16bitUnsigned(uint8_t address, uint8_t reg);

  // Helper method to read a 16 bit signed value from the given register
  virtual int16_t readI2CRegister16bitSigned(uint8_t address, uint8_t reg);

  // Helper method to read a 8 bit value from the given register
  virtual uint8_t readI2CRegister8bit(uint8_t address, uint8_t reg);


protected:
  uint8_t sensorAddress;
};

#endif
