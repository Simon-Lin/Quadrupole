#ifndef _PCF6591_
#define _PCF8591_

#include "I2Cdev.h"

#define PCF8591_DEFAULT_ADDRESS 0x48

#define PCF8591_MODE_FOUR_SINGLE_ENDED 0
#define PCF8591_MODE_THREE_DIFFERENTIAL 1
#define PCF8591_MODE_MIXED 2
#define PCF8591_MODE_TWO_DIFFERENTIAL 3

class PCF8591 {
 public:
  PCF8591();
  PCF8591(uint8_t address);
  void initialize();
  bool testConnection();

  bool  setInputMode (uint8_t mode_number, uint8_t channel_number);
  float ADConversion ();
  void  DAConversion (float voltage_ratio);
  void  setDACOff ();

 private:
  uint8_t addr;
  int adc_mode;
  bool dac_on;
};

#endif
