#include <Arduino.h>

const int ENC_A_PIN = 32;
const int ENC_B_PIN = 33;

volatile int counter = 0;

void IRAM_ATTR handleA() 
{
  Serial.print("A: ");
  Serial.println(digitalRead(ENC_A_PIN));
  Serial.print("B: ");
  Serial.println(digitalRead(ENC_B_PIN));
  counter ++;
  if(counter > 1)
  {
    counter = 0;
    Serial.println("----");
  }
}


void setup() {
  Serial.begin(115200);

  pinMode(ENC_A_PIN, INPUT);
  pinMode(ENC_B_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), handleA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), handleA, FALLING);
}

void loop() 
{

}
