
#include <Arduino.h>
#include <SerialParser.h>
#include <GyverOS.h>

#define PARSE_AMOUNT 6

SerialParser parser(PARSE_AMOUNT);

GyverOS<1> OS; // указать макс. количество задач


void printMsg() {
    String answer = "#1 "
                    + String(parser.getData()[0]) + " "
                    + String(parser.getData()[1]) + " "
                    + String(parser.getData()[2]) + " "
                    // depth
                    + String(parser.getData()[3]) + " "
                    // temp
                    + String(parser.getData()[4]) + ";";
    Serial.print(answer);
}

void setup() {
  Serial.begin(115200);

  OS.attach(0, printMsg, 50);
}


void loop() {
  parser.update();

  OS.tick();
}