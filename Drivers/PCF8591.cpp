#include "PCF8591.h"
#include "I2Cdev.h"
#include <math.h>

PCF8591::PCF8591() {
  addr = PCF8591_DEFAULT_ADDRESS;
  adc_mode = 0;
  adc_channel = 0;
  dac_on = false;
}

PCF8591::PCF8591(uint8_t address) {
  addr = address;
  adc_mode = 0;
  adc_channel = 0;
  dac_on = false;
}

void PCF8591::initialize () {
  I2Cdev::initialize();
}

bool PCF8591::testConnection() {
  char buf[8];
  return bcm2835_i2c_read (buf, 1) == BCM2835_I2C_REASON_OK;
}

bool PCF8591::setInputMode (uint8_t mode_number, uint8_t channel_number) {
  if (mode_number > 3 || channel_number > 3) return -1;
  adc_mode = mode_number;
  adc_channel = channel_number;
  bcm2835_i2c_setSlaveAddress(addr);
  char buf = channel_number & (mode_number << 4) & ((dac_on ? 0 : 1) << 6);
  bcm2835_i2c_write (&buf, 1);
  return 1;
}

float PCF8591::ADConversion () {
  char buf[2];
  bcm2835_i2c_setSlaveAddress(addr);
  bcm2835_i2c_read (buf, 2);
  return buf[1] / 256.0;
}

void PCF8591::DAConversion (float voltage_ratio) {
  char buf[2];
  buf[1] = adc_channel & (adc_mode << 4) & (1 << 6);
  buf[2] = (int) round(256 * voltage_ratio);
  bcm2835_i2c_write (buf, 2);
}

void PCF8591::setDACOff () {
  char buf = adc_channel & (adc_mode << 4) & (0 << 6);
  bcm2835_i2c_write (&buf, 1);
}
