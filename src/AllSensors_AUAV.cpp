#include "AllSensors_AUAV.h"

#include <math.h>

AllSensors_AUAV::AllSensors_AUAV(TwoWire *bus, SensorPressureRange pressureRange) :
  pressure_diff_unit(PressureDifferentialUnit::IN_H2O),
  temperature_unit(TemperatureUnit::CELCIUS)
{
  this->bus = bus;

  // Scaling factor for differential pressure sensor
  pressure_range = pressureRange * 2;
}

void AllSensors_AUAV::startMeasurement(MeasurementType measurement_type) {
  bus->beginTransmission(I2C_ADDRESS_DIFF);
  bus->write((uint8_t) measurement_type);
  bus->write(0x00);
  bus->write(0x00);
  bus->endTransmission();

  bus->beginTransmission(I2C_ADDRESS_ABS);
  bus->write((uint8_t) measurement_type);
  bus->write(0x00);
  bus->write(0x00);
  bus->endTransmission();
}

uint8_t AllSensors_AUAV::readStatus(SensorType type) {
  switch (type) {
    case SensorType::DIFFERENTIAL:
      bus->requestFrom(I2C_ADDRESS_DIFF, READ_STATUS_LENGTH);
      break;
    case SensorType::ABSOLUTE:
      bus->requestFrom(I2C_ADDRESS_ABS, READ_STATUS_LENGTH);
      break;
    default:
      return 0; // or handle error
  }

  status = bus->read();
  bus->endTransmission();

  return status;
}

bool AllSensors_AUAV::readData(bool wait, SensorType type) {
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
      return true; // or handle error
  }

try_again:
  bus->requestFrom(address, (uint8_t) (READ_STATUS_LENGTH + READ_PRESSURE_LENGTH + READ_TEMPERATURE_LENGTH));

  // Read the 8-bit status data.
  status = bus->read();

  if (isError(status)) {
    // An ALU or memory error occurred.
    bus->endTransmission();
    goto error;
  }

  if (isBusy(status)) {
    // The sensor is still busy; either retry or fail.
    bus->endTransmission();

    if (wait) {
      // Wait just a bit so that we don't completely hammer the bus with retries.
      goto try_again;

    }

    goto error;
  }

  // Read the 24-bit (high 16-18 bits defined) of raw pressure data.
  *((uint8_t *)(&raw_p)+2) = bus->read();
  *((uint8_t *)(&raw_p)+1) = bus->read();
  *((uint8_t *)(&raw_p)+0) = bus->read();

  // Read the 24-bit (high 16 bits defined) of raw temperature data.
  *((uint8_t *)(&raw_t)+2) = bus->read();
  *((uint8_t *)(&raw_t)+1) = bus->read();
  *((uint8_t *)(&raw_t)+0) = bus->read();
  
  bus->endTransmission();

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

  return isError(status);

error:

  switch (type) {
    case SensorType::DIFFERENTIAL:
      pressure_d = NAN;
      temperature_d = NAN;
      break;
    case SensorType::ABSOLUTE:
      pressure_a = NAN;
      temperature_a = NAN;
      break;
    default:
      break;
  }

  return true;
}