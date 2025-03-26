/*
  * Code oriented to explore the usage of a servo with the ESP32.
  * 
  * Modified by: DanielFLopez1620
  * Based on:
  * https://RandomNerdTutorials.com/esp32-servo-motor-web-server-arduino-ide/
  */

// ------------------ Required headers ----------------------------------------

#include <Arduino.h>
#include <ESP32Servo.h>

// ------------------- Global definitions -------------------------------------
static const int servoPin = 13;
Servo my_servo;

// ------------------- Single set up function ---------------------------------

void setup() 
{
    Serial.begin(115200);
    my_servo.attach(servoPin);
}

// --------------------- Loop implementation ----------------------------------
void loop() 
{
    // Goint clockwise
    for(int posDegrees = 0; posDegrees <= 180; posDegrees++) 
    {
        my_servo.write(posDegrees);
        Serial.println(posDegrees);
        delay(20);
    }

    // Going counter-clockwise
    for(int posDegrees = 180; posDegrees >= 0; posDegrees--) 
    {
        my_servo.write(posDegrees);
        Serial.println(posDegrees);
        delay(20);
    }
}