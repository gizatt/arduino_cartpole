#define ARDUINO_MAIN

#include "Arduino.h"

void setup() {  
  Serial.begin(115200);
  Serial.print("STARTING\n");
}

void loop() {
  Serial.print("loop\n");
  delay(1000);
}
