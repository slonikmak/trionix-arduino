
#include <Arduino.h>
#include <SerialParser.h>

#define PARSE_AMOUNT 6

SerialParser parser(PARSE_AMOUNT);


void setup() {
  Serial.begin(115200);
}

void loop() {
  parser.update();

    if (parser.received())
    {
        parser.getData();
        Serial.println("OK!;");
        delay(25);
    }
}