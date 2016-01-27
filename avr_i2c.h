#ifndef _AVR_I2C_H_
#define _AVR_I2C_H_

#include <Arduino.h>
#include "utility/twi.h"

void avr_i2c_begin(void); // Master only
void avr_i2c_end(void);

int avr_i2c_read_byte(void);
size_t avr_i2c_write_byte(uint8_t data);

/*
int  avr_i2c_read( unsigned char slave_addr, unsigned char reg_addr,
                   unsigned char length,     unsigned char *data);
int  avr_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                   unsigned char length,     unsigned char const * data);
*/

// Static these later
void avr_i2c_beginTransmission(uint8_t slave_addr);
uint8_t avr_i2c_endTransmission(bool sendStop);
uint8_t avr_i2c_requestFrom(uint8_t slave_addr,uint8_t length,uint8_t sendStop);

#endif
