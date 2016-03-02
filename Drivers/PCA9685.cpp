#include "PCA9685.h"
#include <math.h>

PCA9685::PCA9685() {
	devAddr = PCA9685_DEFAULT_ADDRESS;
}

PCA9685::PCA9685(uint8_t address) {
	devAddr = address;
}

void PCA9685::initialize() {
	//enable register Auto-increment
	I2Cdev::writeBit(devAddr, PCA9685_MODE1, 5, 1);
	//initialize PWM values
	I2Cdev::writeByte(devAddr, PCA9685_PRESCALE, 30);
	for (int i = 0; i <16; i++) {
		LED_delay[i] = 0;
		LED_duty_cycle[i] = 0;
	}
	freq = 200;
	for (int i = 0; i < 48; i += 4) {
		buf[i] = 0x00;
		buf[i+1] = 0x00;
		buf[i+2] = 0x00;
		buf[i+3] = 0x20;
	}
	I2Cdev::writeBytes(devAddr, PCA9685_LEDX_ON_L, 48, buf);
	//wake chip from sleeping 
	I2Cdev::writeBit(devAddr, PCA9685_MODE1, 4, 0);
	bcm2835_st_delay (0, 500);
}

void PCA9685::sleep() {
	I2Cdev::writeBit(devAddr, PCA9685_MODE1, 4, 1);
	bcm2835_st_delay (0, 500);
}

void PCA9685::restart() {
	I2Cdev::readBit(devAddr, PCA9685_MODE1, 7, buf);
	if (buf[0] == 1) {
		I2Cdev::writeBit(devAddr, PCA9685_MODE1, 4, 0);
		bcm2835_st_delay (0, 500);
		I2Cdev::writeBit(devAddr, PCA9685_MODE1, 7, 1);
	}
}

void PCA9685::setPWM(int LED_num, float duty_cycle) {
	if (duty_cycle >= 1) {
		I2Cdev::writeByte(devAddr, LED_N_ON_H(LED_num), LED_FULL);
		I2Cdev::writeByte(devAddr, LED_N_OFF_H(LED_num), 0);
	} else if (duty_cycle <= 0) {
		I2Cdev::writeByte(devAddr, LED_N_ON_H(LED_num), 0);
		I2Cdev::writeByte(devAddr, LED_N_OFF_H(LED_num), LED_FULL);
	} else {
		uint16_t on = 0;
		uint16_t off = round(4095 * duty_cycle);
		uint8_t *ind = (uint8_t*) &on;
		I2Cdev::writeBytes(devAddr, LED_N_ON_L(LED_num), 2, ind);
		ind = (uint8_t*) &off;
		I2Cdev::writeBytes(devAddr, LED_N_OFF_L(LED_num), 2, ind);
	}
	LED_delay[LED_num] = 0;
	LED_duty_cycle[LED_num] = duty_cycle;
}

void PCA9685::setPWM(int LED_num, float duty_cycle, float delay) {
	uint8_t *ind;
	if (duty_cycle >= 1) {
		I2Cdev::writeByte(devAddr, LED_N_ON_H(LED_num), LED_FULL);
		I2Cdev::writeByte(devAddr, LED_N_OFF_H(LED_num), 0);
	} else if (duty_cycle <= 0) {
		I2Cdev::writeByte(devAddr, LED_N_ON_H(LED_num), 0);
		I2Cdev::writeByte(devAddr, LED_N_OFF_H(LED_num), LED_FULL);
	} else {
		uint16_t off = round(4095 * (delay+duty_cycle));
		if (off > 4095) {
			off -= 4096;
		}
		ind = (uint8_t*) &off;
		I2Cdev::writeBytes(devAddr, LED_N_OFF_L(LED_num), 2, ind);
	}
	uint16_t on = round(4095 * delay);
	ind = (uint8_t*) &on;
	I2Cdev::writeBytes(devAddr, LED_N_ON_L(LED_num), 2, ind);	
	LED_delay[LED_num] = delay;
	LED_duty_cycle[LED_num] = duty_cycle;
}

void PCA9685::getPWM(int LED_num, float *duty_cycle, float *delay) {
	*delay = LED_delay[LED_num];
	*duty_cycle = LED_duty_cycle[LED_num];
}
		
bool PCA9685::set_PWM_Frequency(uint32_t frequency) {
	int prescale = round(double(PCA9685_OSC_CLOCK_MHZ*1E6)/(4096*frequency)) - 1;
	if (prescale < PCA9685_PRESCALE_MIN || prescale > PCA9685_PRESCALE_MAX) {
		return false;
	} else {
		I2Cdev::writeByte(devAddr, PCA9685_PRESCALE, prescale);
		freq = frequency;
		return true;
	}
}

uint32_t PCA9685::get_PWM_Frequency() {
	return freq;
}
