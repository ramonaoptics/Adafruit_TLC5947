/********************
*******************************
  This is an example for our Adafruit 24-channel PWM/LED driver

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/1429

  These drivers uses SPI to communicate, 3 pins are required to
  interface: Data, Clock and Latch. The boards are chainable

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Jaehee_TLC5947.h"

// How many tlc'ss do you have chained?
#define NUM_TLC5947 12
#define data   11
#define clock  13
#define latch0 14
#define latch1 15
#define latch2 16
#define latch3 17
#define latch4 18
#define latch5 19
#define latch6 20
#define latch7 21
#define latch8 22
// set to -1 to not use the enable pin (its optional)
#define blank  6

Adafruit_TLC5947 tlc = Adafruit_TLC5947(NUM_TLC5947,
  (unsigned char)clock,
  (unsigned char)data,
  (unsigned char)latch0,
  (unsigned char)latch1,
  (unsigned char)latch2,
  (unsigned char)latch3,
  (unsigned char)latch4,
  (unsigned char)latch5,
  (unsigned char)latch6,
  (unsigned char)latch7,
  (unsigned char)latch8);

void setup() {
  pinMode(blank, OUTPUT);
  digitalWrite(blank, HIGH);
  tlc.begin();
  digitalWrite(blank, LOW);
}

void loop() {
  // cycle(100, 10, latch0); //4096
  // delay(400);
  cycle(100, 100, latch1); //4096
  delay(400);
  cycle(0, 100, latch1);
  delay(400);
  // cycle(100, 10, latch2); //4096
  // delay(400);
  // cycle(100, 10, latch3); //4096
  // delay(400);
  // cycle(100, 10, latch4); //4096
  // delay(400);
  // cycle(100, 10, latch5); //4096
  // delay(400);
  // cycle(100, 10, latch6); //4096
  // delay(400);
  // cycle(100, 10, latch7); //4096
  // delay(400);
  // cycle(100, 10, latch8); //4096
  // delay(400);

//  cycle(0, 200);
//  delay(400);
//  colorWipe(4095, 0, 0, 100); // "Red" (depending on your LED wiring)
//  delay(400);
//  colorWipe(0, 4095, 0, 100); // "Green" (depending on your LED wiring)
//  delay(200);
//  colorWipe(0, 0, 4095, 100); // "Blue" (depending on your LED wiring)
//  delay(200);
//  rainbowCycle(10);
}

void cycle(uint16_t b, uint16_t wait, uint8_t latch){
  for(uint16_t i = 0; i < NUM_LEDS_PER_DRIVER * NUM_TLC5947; i++){
    tlc.setPWM(i, b);
    tlc.write((unsigned char) latch);
    delay(wait);
  }
}
