#include "AllSensors_AUAV.h"

#include <math.h>

AllSensors_AUAV::AllSensors_AUAV(TwoWire *bus, SensorPressureRange pressureRange) :
  pressure_diff_unit(PressureUnit::IN_H2O),
  temperature_unit(TemperatureUnit::CELCIUS)
{
  this->bus = bus;

  // Scaling factor for differential pressure sensor
  pressure_range = pressureRange * 2;
}

void AllSensors_AUAV::startMeasurement(SensorType type, MeasurementType measurement_type) {
  switch (type) {
    case SensorType::DIFFERENTIAL:
      bus->beginTransmission(I2C_ADDRESS_DIFF);
      break;
    case SensorType::ABSOLUTE:
      bus->beginTransmission(I2C_ADDRESS_ABS);
      break;
    default:
    return; // or handle error
  } 
  bus->write((uint8_t) measurement_type);
  // bus->write(0x00); // reserved
  // bus->write(0x00); // reserved
  bus->endTransmission();
}

uint8_t AllSensors_AUAV::readStatus(SensorType type) {
  uint8_t address;
  switch (type) {
    case SensorType::DIFFERENTIAL:
      address = I2C_ADDRESS_DIFF;
      break;
    case SensorType::ABSOLUTE:
      address = I2C_ADDRESS_ABS;
      break;
    default:
      return 0; // or handle error
  }
  bus->beginTransmission(address);
  bus->requestFrom(address, READ_STATUS_LENGTH);
  status = bus->read();
  bus->endTransmission();
  return status;
}

bool AllSensors_AUAV::readData(SensorType type) {
  uint8_t address;
  switch (type) {
    case SensorType::DIFFERENTIAL:
      address = I2C_ADDRESS_DIFF;
      pressure_d = NAN;
      temperature_d = NAN;
      break;
    case SensorType::ABSOLUTE:
      address = I2C_ADDRESS_ABS;
      pressure_a = NAN;
      temperature_a = NAN;
      break;
    default:
      return false; // or handle error
  }
  bus->beginTransmission(address);
  bus->requestFrom(address, (uint8_t) (READ_STATUS_LENGTH + READ_PRESSURE_LENGTH + READ_TEMPERATURE_LENGTH));

  // Read the 8-bit status data.
  status = bus->read();

  if (isError(status)) {
    // An ALU or memory error occurred.
    Serial.println("AUAV Error: ALU or memory error occurred");
    bus->endTransmission();
    return false;
  }

  if (isBusy(status)) {
    // The sensor is still busy; either retry or fail.
    bus->endTransmission();
    return false;
  }

  // Read the 24-bit of raw pressure data.
  *((uint8_t *)(&raw_p)+2) = bus->read();
  *((uint8_t *)(&raw_p)+1) = bus->read();
  *((uint8_t *)(&raw_p)+0) = bus->read();

  // Read the 24-bit of raw temperature data.
  *((uint8_t *)(&raw_t)+2) = bus->read();
  *((uint8_t *)(&raw_t)+1) = bus->read();
  *((uint8_t *)(&raw_t)+0) = bus->read();

  switch (type) {
    case SensorType::DIFFERENTIAL:
      pressure_d = convertPressure(transferDifferentialPressure(raw_p));
      temperature_d = convertTemperature(transferTemperature(raw_t));
      break;
    case SensorType::ABSOLUTE:
      pressure_a = convertPressure(transferAbsolutePressure(raw_p));
      temperature_a = convertTemperature(transferTemperature(raw_t));
      break;
    default:
      break;
  }
  bus->endTransmission();
  return true;
}