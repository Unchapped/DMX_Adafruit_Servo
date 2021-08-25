#include <DMXSerial.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

//Debug blinkenlight
#define LED_PIN 13

/* Read PORTD 3-7 (Arduino D3-D7) for upper 5 bits of starting DMX Address
 * Assumes we get a 1-indexed aligned 16 channel segment of the DMX buffer
 * therefore if D2-D6 are set to b00001, that's equivalent to DMX addresses
 * 17 through 32, and so forth. Since we're using the internal pullups for
 * simplicity, we need to tie down "1" bits, and invert the read value.
 * NONPORTABLE: This is UNO specific and not portable, but we do this raw for
 *              speed. If/when we migrate to a new architecture we'll need to
 *              add compiler directives for other boards. */
#define PORTD_BITMASK B11111000
unsigned int read_dmx_address(){
  DDRD &= ~PORTD_BITMASK; //All inputs, we'll initialize the serial port later.
  PORTD = PORTD_BITMASK; //enable pullups
  unsigned int addr = ((~PIND & PORTD_BITMASK) << 1) | 1; //read the setting
  PORTD &= ~PORTD_BITMASK; //disable pullups to save power
  return addr;
}

unsigned int dmxStartAddress = 1; //Default DMX bus address to start with.

void setup() {
  //Initalize DMX Library
  DMXSerial.init(DMXReceiver);

  //Initialize Adafruit PWM
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  //Initialize debug LED
  pinMode(LED_PIN, OUTPUT);
}


void loop() {
  dmxStartAddress = read_dmx_address();

  if(DMXSerial.noDataSince() < 100){
    digitalWrite(LED_PIN, HIGH); //Turn On Debug LED
    if(DMXSerial.dataUpdated()) {
      for(int channel = 0; channel < 16; channel++) {
        pwm.setPWM(channel, 0, map(DMXSerial.read(dmxStartAddress + channel), 0, 255, SERVOMIN, SERVOMAX));
      }
      DMXSerial.resetUpdated();
    } 
  } else { //timeout
    digitalWrite(LED_PIN, LOW); //Turn Off Debug LED
  }
}

  
