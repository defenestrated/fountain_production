#include <Arduino.h>
#include "utilities.h"

#define HWSERIAL Serial1

void hwsend(char c){
  Serial.println("sending:");
  Serial.print("<");
  Serial.print(c);
  Serial.print(", >");
  Serial.println();
  HWSERIAL.print("<");
  HWSERIAL.print(c);
  HWSERIAL.print(">");
  HWSERIAL.println();
}
void hwsend(char c, int i){
  Serial.println("sending:");
  Serial.print("<");
  Serial.print(c);
  Serial.print(", ");
  Serial.print(i);
  Serial.print(">");
  Serial.println();
  HWSERIAL.print("<");
  HWSERIAL.print(c);
  HWSERIAL.print(", ");
  HWSERIAL.print(i);
  HWSERIAL.print(">");
  HWSERIAL.println();
}
void hwsend(char c, int i, int j){
  Serial.println("sending:");
  Serial.print("<");
  Serial.print(c);
  Serial.print(", ");
  Serial.print(i);
  Serial.print(", ");
  Serial.print(j);
  Serial.print(">");
  Serial.println();
  HWSERIAL.print("<");
  HWSERIAL.print(c);
  HWSERIAL.print(", ");
  HWSERIAL.print(i);
  HWSERIAL.print(", ");
  HWSERIAL.print(j);
  HWSERIAL.print(">");
  HWSERIAL.println();
}
