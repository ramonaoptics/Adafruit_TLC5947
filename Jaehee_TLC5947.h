/*!
 *  @file Adafruit_TLC5947.h
 *
 * 	Adafruit 24-channel PWM/LED driver
 *
 * 	This is a library for the Adafruit 24-channel PWM/LED driver:
 * 	http://www.adafruit.com/products/1429
 *
 *  These drivers uses SPI to communicate, 3 pins are required to
 *  interface: Data, Clock and Latch. The boards are chainable
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_TLC5947_H
#define _ADAFRUIT_TLC5947_H

#include <Arduino.h>

#define NUM_LEDS_PER_DRIVER 24
#define BITS_PER_CHANNEL 12
/*!
 *    @brief  Class that stores state and functions for interacting with
 *            TLC5947 24-channel PWM/LED driver
 */
class Adafruit_TLC5947 {
public:
  Adafruit_TLC5947(uint16_t n_tlc5947, uint8_t clock, uint8_t data, uint8_t latch0, uint8_t latch1, uint8_t latch2, uint8_t latch3, uint8_t latch4, uint8_t latch5, uint8_t latch6, uint8_t latch7, uint8_t latch8);

  boolean begin(void);

  void setPWM(uint16_t chan, uint16_t pwm);
  void setLED(uint16_t lednum, uint16_t r, uint16_t g, uint16_t b);
  void write(uint8_t latch);

private:
  uint16_t *pwmbuffer;

  uint16_t numdrivers;
  uint8_t _clk, _dat, _lat0, _lat1, _lat2, _lat3, _lat4, _lat5, _lat6, _lat7, _lat8;
};

#endif
