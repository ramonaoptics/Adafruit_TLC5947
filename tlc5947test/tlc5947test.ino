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
#define analog_power_check 23
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

void wait_until_power_plugged_in(){
  Serial.begin(115200);
  pinMode(blank, OUTPUT);
  digitalWrite(blank, HIGH);

  while(true){
    // vref value from https://forum.pjrc.com/threads/54662-Teensy-3-2-ADC-Input-Range
    float v_ref = 3.3;
    // assumes 10 bit for max count value
    int val;
    val = analogRead(9);
    float analog_power = (val / 1023.0) * v_ref;
    float v_in = analog_power * 3;
    Serial.print("analog 9 is: ");
    Serial.println(val);
    Serial.print("Voltage is: ");
    Serial.println(v_in);

    // if ((v_in > 4.0) && (v_in < 5.5)){
    if ((v_in > 4.0)){
      // power cable plugged in
      break;
     }
     delay(500);
  }
}

void setup() {
  // pinMode(blank, OUTPUT); //this is inside wait_until_power_plugged_in
  // digitalWrite(blank, HIGH); //this is inside wait_until_power_plugged_in
  // wait until power plugged in
  wait_until_power_plugged_in();
  tlc.begin();
  digitalWrite(blank, LOW);
}

void loop() {
  latch_cycle(latch0);
  latch_cycle(latch1);
  latch_cycle(latch2);
  latch_cycle(latch3);
  latch_cycle(latch4);
  latch_cycle(latch5);
  latch_cycle(latch6);
  latch_cycle(latch7);
  latch_cycle(latch8);

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

void latch_cycle(uint8_t latch_var){
  // 4095 is max intensity
  wait_until_power_plugged_in();
  tlc.begin();
  digitalWrite(blank, LOW);
  cycle(10, 1, latch_var);
  cycle(0, 1, latch_var);
}

void cycle(uint16_t b, uint16_t wait, uint8_t latch){
  for(uint16_t i = 0; i < NUM_LEDS_PER_DRIVER * NUM_TLC5947; i++){
    tlc.setPWM(i, b);
    tlc.write((unsigned char) latch);
    delay(wait);
  }
}
