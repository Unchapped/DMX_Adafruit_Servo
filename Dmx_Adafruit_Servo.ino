// - - - - -
// DmxSerial - A hardware supported interface to DMX.
// DmxSerialRecv.ino: Sample DMX application for retrieving 3 DMX values:
// address 1 (red) -> PWM Port 9
// address 2 (green) -> PWM Port 6
// address 3 (blue) -> PWM Port 5
//
// Copyright (c) 2011-2015 by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
//
// Documentation and samples are available at http://www.mathertel.de/Arduino
// 25.07.2011 creation of the DmxSerial library.
// 10.09.2011 fully control the serial hardware register
//            without using the Arduino Serial (HardwareSerial) class to avoid ISR implementation conflicts.
// 01.12.2011 include file and extension changed to work with the Arduino 1.0 environment
// 28.12.2011 changed to channels 1..3 (RGB) for compatibility with the DmxSerialSend sample.
// 10.05.2012 added some lines to loop to show how to fall back to a default color when no data was received since some time.
// - - - - -

#include <DMXSerial.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

//Debug blinkenlight
#define _LED_PIN 13

/* Read PORTD 3-7 (Arduino D3-D7) for upper 5 bits of starting DMX Address
 * Assumes we get a 1-indexed aligned 16 channel segment of the DMX buffer
 * therefore if D2-D6 are set to b00001, that's equivelent to DMX addresses 17 through 32, and so forth
 * NOTE: we're using the internal pullups for simplicity, so we need to tie down "1" bits, and invert the read value in software.
 * NONPORTABLE: This is UNO specific and not portable, but we do this raw for speed
 *       If/when we migrate we'll need to add compiler directives for other boards. */
#define _PORTD_BITMASK B11111000
unsigned int read_dmx_address(){
  DDRD &= ~_PORTD_BITMASK; //All inputs, we'll initialize the serial port later.
  PORTD = _PORTD_BITMASK; //enable pullups
  unsigned int addr = ((~PIND & _PORTD_BITMASK) << 1) | 1; //read the setting
  PORTD &= ~_PORTD_BITMASK; //disable pullups to save power
  return addr;
}

unsigned int dmxStartAddress = 1; //Default DMX bus address to start with.
//const unsigned char portd_bitmask = B11111000; 

//Servo myservo;  // create servo object to control a servo

void setup() {
  dmxStartAddress = read_dmx_address();
  DMXSerial.init(DMXReceiver);
  pinMode(_LED_PIN, OUTPUT);
}


void loop() {
  dmxStartAddress = read_dmx_address();

  /* unsigned long lastPacket = DMXSerial.noDataSince();
  if (lastPacket < 100) {
    digitalWrite(_LED_PIN, HIGH);
  } else {
    digitalWrite(_LED_PIN, LOW);
  } */

  //if(DMXSerial.receive(100)){
  if(DMXSerial.noDataSince() < 100){
    digitalWrite(_LED_PIN, HIGH); //Turn On Debug LED
    //TODO: Tomorrow, dig into why this no worky!
    //if(DMXSerial.dataUpdated()) {
      for(int channel = 0; channel < 16; channel++) {
        pwm.setPWM(channel, 0, map(DMXSerial.read(dmxStartAddress + channel), 0, 255, SERVOMIN, SERVOMAX));
      }
      ///DMXSerial.resetUpdated();
    //}
  } else { //timeout
    digitalWrite(_LED_PIN, LOW); //Turn Off Debug LED
  }
}

  
