/*!
 *  @file Adafruit_TLC5947.cpp
 *
 *  @mainpage Adafruit 24-channel PWM/LED driver
 *
 *  @section intro_sec Introduction
 *
 * 	Driver for Microchip's TLC5947
 *
 * 	This is a library for the Adafruit TLC5947
 * 	http://www.adafruit.com/products/
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section author Author
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include "Jaehee_TLC5947.h"
#include <SPI.h>
/*!
 *    @brief  Instantiates a new TLC5947 class
 *    @param  n
 *            num of drivers (boards)
 *    @param  c
 *            Arduino pin connected to TLC5947 clock pin
 *    @param  d
 *            Arduino pin connected to TLC5947 data pin
 *    @param  l
 *            Arduino pin connected to TLC5947 latch pin
 */
Adafruit_TLC5947::Adafruit_TLC5947(uint16_t n_tlc5947, uint8_t c, uint8_t d,
                                   uint8_t l0, uint8_t l1, uint8_t l2, uint8_t l3, uint8_t l4, uint8_t l5, uint8_t l6, uint8_t l7, uint8_t l8) {
  numdrivers = n_tlc5947;
  _clk = c;
  _dat = d;
  _lat0 = l0;
  _lat1 = l1;
  _lat2 = l2;
  _lat3 = l3;
  _lat4 = l4;
  _lat5 = l5;
  _lat6 = l6;
  _lat7 = l7;
  _lat8 = l8;
  // malloc allocates space. each led is controlled by 12 bit number.
  // memory defined in unsigned 16 bit integer. we need 2 bytes (8*2) to store
  // 12 bits.
  pwmbuffer = (uint16_t *)malloc(2 * NUM_LEDS_PER_DRIVER * n_tlc5947);
  memset(pwmbuffer, 0, 2 *  NUM_LEDS_PER_DRIVER * n_tlc5947);

}


/*!
 *    @brief  Writes PWM data to the all connected TLC5947 boards
 */
 void Adafruit_TLC5947::write(uint8_t _lat) {


   uint8_t buffer[3];
   uint16_t pwm[2];
   digitalWrite(_lat, LOW);
   // 24 channels per TLC5974
   SPI.beginTransaction(SPISettings());
   for (int16_t channel = NUM_LEDS_PER_DRIVER * numdrivers - 2; channel >= 0; channel=channel-2) {
     pwm[0] = pwmbuffer[channel];
     pwm[1] = pwmbuffer[channel + 1];
     // 12 bits per channel, send MSB first
     buffer[0] = (pwm[1] >> 4) & 0x00FF;
     buffer[1] = ((pwm[0] & 0x0F00) >> 8) + ((pwm[1] & 0x000F) << 4);
     buffer[2] = (pwm[0] >> 0 ) & 0x00FF;
     SPI.transfer(buffer[0]);
     SPI.transfer(buffer[1]);
     SPI.transfer(buffer[2]);
   }
   SPI.endTransaction();

   digitalWrite(_lat, HIGH);
   digitalWrite(_lat, LOW);
 }


/*!
 *    @brief  Set the PWM channel / value
 *    @param  chan
 *            channel number ([0 - 23] on each board, so chanel 2 for second board will be 25)
 *    @param  pwm
 *            pwm value [0-4095]
 */
void Adafruit_TLC5947::setPWM(uint16_t chan, uint16_t pwm) {
  if (pwm > 4095)
    pwm = 4095;
  if (chan >= NUM_LEDS_PER_DRIVER * numdrivers )
    return;

  pwmbuffer[chan] = pwm;
}



/*!
 *    @brief  Set LED
 *    @param  lednum
 *            led number
 *    @param  r
 *            red value [0-255]
 *    @param  g
 *            green value [0-255]
 *    @param  b
 *            blue value [0-255]
 */
void Adafruit_TLC5947::setLED(uint16_t lednum, uint16_t r, uint16_t g,
                              uint16_t b) {
  // In python, and numpy, PWM buffer would be a 2D array with the first dimension
  // begin the tripplet of the RGB led yo uwant to control, and gthe last dimension
  // corresponding to 0 for red, 1 for blue, 2 for green
  // [lednum, 0] for red color is the same as
  // C index lednum * 3 + 0
  setPWM(lednum * 3 + 0, r);
  setPWM(lednum * 3 + 1, g);
  setPWM(lednum * 3 + 2, b);
}

/*!
 *    @brief  Setups the HW
 *    @return True if initialization was successful, otherwise false.
 */
boolean Adafruit_TLC5947::begin() {
  if (!pwmbuffer)
    return false;
  SPI.begin();
  pinMode(_lat0, OUTPUT);
  digitalWrite(_lat0, LOW);
  pinMode(_lat1, OUTPUT);
  digitalWrite(_lat1, LOW);
  pinMode(_lat2, OUTPUT);
  digitalWrite(_lat2, LOW);
  pinMode(_lat3, OUTPUT);
  digitalWrite(_lat3, LOW);
  pinMode(_lat4, OUTPUT);
  digitalWrite(_lat4, LOW);
  pinMode(_lat5, OUTPUT);
  digitalWrite(_lat5, LOW);
  pinMode(_lat6, OUTPUT);
  digitalWrite(_lat6, LOW);
  pinMode(_lat7, OUTPUT);
  digitalWrite(_lat7, LOW);
  pinMode(_lat8, OUTPUT);
  digitalWrite(_lat8, LOW);

  for(int16_t channel = NUM_LEDS_PER_DRIVER * numdrivers - 2; channel >= 0; channel=channel-2){
    setPWM(channel, 0);
  }
  write((unsigned char) _lat0);
  write((unsigned char) _lat1);
  write((unsigned char) _lat2);
  write((unsigned char) _lat3);
  write((unsigned char) _lat4);
  write((unsigned char) _lat5);
  write((unsigned char) _lat6);
  write((unsigned char) _lat7);
  write((unsigned char) _lat8);
  return true;
}
