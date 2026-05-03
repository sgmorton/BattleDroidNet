#include <Arduino.h>

#define P34 34
#define P35 35
#define P36 36
#define P37 37
#define P38 38
#define P39 39

void setup() {
  Serial.begin(115200);
  pinMode(P34, INPUT);
  pinMode(P35, INPUT);
  pinMode(P36, INPUT);
  pinMode(P37, INPUT);
  pinMode(P38, INPUT);
  pinMode(P39, INPUT);
}

void loop() {
  Serial.printf("P34:%d P35:%d P36:%d P37:%d P38:%d P39:%d\n", 
    digitalRead(P34), digitalRead(P35), digitalRead(P36), 
    digitalRead(P37), digitalRead(P38), digitalRead(P39));
  delay(100);
}
