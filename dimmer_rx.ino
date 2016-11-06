#include "ManchesterRF.h" //https://github.com/cano64/ManchesterRF
#include <TimerOne.h>


#define RX_PIN 11 //any pin can receive

ManchesterRF rf(MAN_2400); //link speed, try also MAN_300, MAN_600, MAN_1200, MAN_2400, MAN_4800, MAN_9600, MAN_19200, MAN_38400

uint8_t channel, bh, bl;
uint16_t brightness = 0;
uint16_t old_brightness = 0;
uint8_t state = LOW;

void setup() {
//  Serial.begin(57600);
  rf.RXInit(RX_PIN);
  pinMode(9, OUTPUT); 
  pinMode(13, OUTPUT); 
  Timer1.initialize(10000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  Timer1.pwm(9, 0);
}

void loop() {

  if (rf.available()) { //something is in RX buffer
    if (rf.receiveByte(channel, bh, bl)) {
      if (channel == 1)
      {
        if (state == LOW) {state = HIGH;}
        else {state = LOW;}
/*
        Serial.print("bh=");
        Serial.print(bh);
        Serial.print(" bl=");
        Serial.print(bl);
*/
        brightness = (((uint16_t)bl) | (((uint16_t)bh) << 8));
/*        Serial.print(" brightness=");
        Serial.println(brightness);
*/
        if (brightness != old_brightness)
        {
          old_brightness = brightness;
          if (brightness < 750)
          {
            Timer1.pwm(9, brightness);
          }
          else
          {
//            Timer1.pwm(9, 1023);
            Timer1.pwm(9, 0);
            digitalWrite(9, HIGH);
          }
        }
      }
      //digitalWrite(13, state); //blink the LED on receive
    }
  }  

}


