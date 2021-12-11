#include <Arduino.h>
#include <Servo.h>
#include "SerialParser.h"

#define PARSE_AMOUNT 5

//front
#define pin1 2
//right
#define pin2 3
//back
#define pin3 4
//left
#define pin4 5
//servo front
#define s_pin1 11
//servo right
#define s_pin2 10
//servo back
#define s_pin3 12
//servo left
#define s_pin4 13
//light
#define ledPin 6

Servo m1, m2, m3, m4;
Servo s1, s2, s3, s4;

int m1_val, m2_val, m3_val, m4_val;

SerialParser parser(PARSE_AMOUNT);

int ledValue = 0;

void attach_pins()
{
  m1.attach(pin2);
  m1.writeMicroseconds(1500); // send "stop" signal to ESC.
  m2.attach(pin3);
  m2.writeMicroseconds(1500);
  m3.attach(pin1);
  m3.writeMicroseconds(1500);
  m4.attach(pin4);
  m4.writeMicroseconds(1500);

  //    s1.attach(s_pin1);
  //    s2.attach(s_pin2);
  //    s3.attach(s_pin3);
  //    s4.attach(s_pin4);

  pinMode(ledPin, OUTPUT);
}

void setup()
{
  Serial.begin(115200);

  attach_pins();

  analogWrite(ledPin, 120);

  delay(7000);

  analogWrite(ledPin, 0);
}

void loop()
{

  parser.update();

  if (parser.received()) {

        int *intData = parser.getData();
        int m1_val_new = map(intData[0], -100, 100, 1100, 1900);
        int m2_val_new = map(intData[1], -100, 100, 1100, 1900);
        int m3_val_new = map(intData[2], -100, 100, 1100, 1900);
        int m4_val_new = map(intData[3], -100, 100, 1100, 1900);

        if (intData[4] != ledValue) {
            ledValue = intData[4];
            analogWrite(ledPin, ledValue);
        }


//        int s1_val = dataArray[4];
//        int s2_val = dataArray[5];
//        int s3_val = dataArray[6];
//        int s4_val = dataArray[7];
//
//        int l1_val = dataArray[8];

        if (m1_val_new != m1_val) 
        {
          m1_val = m1_val_new;
          m1.writeMicroseconds(m1_val);
        }

        if (m2_val_new != m2_val) 
        {
          m2_val = m2_val_new;
          m2.writeMicroseconds(m2_val);
        }

        if (m3_val_new != m3_val) 
        {
          m3_val = m3_val_new;
          m3.writeMicroseconds(m3_val);
        }

        if (m4_val_new != m4_val) 
        {
          m4_val = m4_val_new;
          m4.writeMicroseconds(m4_val);
        }


        String answer = "#"
          +String(intData[0]) + " "
          +String(intData[1]) + " "
          +String(intData[2])  + " "
          +String(intData[3]) + " "
          +String(ledValue)
          +";";


        Serial.println(answer);
      }
}
