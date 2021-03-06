#ifndef _PCA9685_
#define _PCA9685_

#include "I2Cdev.h"

#define PCA9685_DEFAULT_ADDRESS 0x40

#define PCA9685_MODE1           0x00
#define PCA9685_MODE2           0x01
#define PCA9685_SUBADDR1        0x02
#define PCA9685_SUBADDR2        0x03
#define PCA9685_SUBADDR3        0x04
#define PCA9685_ALLCALLADDR     0x05
#define PCA9685_LEDX_ON_L       0x06
#define PCA9685_LEDX_ON_H       0x07
#define PCA9685_LEDX_OFF_L      0x08
#define PCA9685_LEDX_OFF_H      0x09
 
#define PCA9685_ALL_LED_ON_L    0xFA
#define PCA9685_ALL_LED_ON_H    0xFB
#define PCA9685_ALL_LED_OFF_L   0xFC
#define PCA9685_ALL_LED_OFF_H   0xFD
#define PCA9685_PRESCALE        0xFE

#define PCA9685_PRESCALE_MIN    0x03    /* => max. frequency of 1526 Hz */
#define PCA9685_PRESCALE_MAX    0xFF    /* => min. frequency of 24 Hz */
 
#define PCA9685_COUNTER_RANGE   4096
#define PCA9685_DEFAULT_PERIOD  5000000 /* Default period_ns = 1/200 Hz */
#define PCA9685_OSC_CLOCK_MHZ   25      /* Internal oscillator with 25 MHz */
 
#define PCA9685_NUMREGS         0xFF
#define PCA9685_MAXCHAN         0x10
 
#define LED_FULL                (1 << 4)
#define MODE1_RESTART           (1 << 7)
#define MODE1_SLEEP             (1 << 4)
#define MODE2_INVRT             (1 << 4)
#define MODE2_OUTDRV            (1 << 2)
 
#define LED_N_ON_H(N)   (PCA9685_LEDX_ON_H + (4 * (N)))
#define LED_N_ON_L(N)   (PCA9685_LEDX_ON_L + (4 * (N)))
#define LED_N_OFF_H(N)  (PCA9685_LEDX_OFF_H + (4 * (N)))
#define LED_N_OFF_L(N)  (PCA9685_LEDX_OFF_L + (4 * (N)))


class PCA9685 {
	public:
		PCA9685();
		PCA9685(uint8_t address);
		void initialize();
	        bool testConnection();
		
		// Mode Register Methods
		void sleep();
		void restart();

		// PWM Output Control Register Methods
		void setPWM(int LED_num, float duty_cycle);
		void setPWM(int LED_num, float duty_cycle, float delay);
		void setAllPWM(float duty_cycle);
		void setAllPWM(float duty_cycle, float delay);
		void getPWM(int LED_num, float *duty_cycle, float *delay);
		
		bool set_PWM_Frequency(uint32_t frequency);
		uint32_t get_PWM_Frequency();
		
	private:
		uint8_t devAddr;
		float LED_delay[16];
		float LED_duty_cycle[16];
		double freq;
		
		uint8_t buf[48];
};
 

#endif
