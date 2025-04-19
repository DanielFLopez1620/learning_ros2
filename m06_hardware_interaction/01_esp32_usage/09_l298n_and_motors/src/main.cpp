/**
 * Code oriented to use two motor DC with the L298N driver, to consider different 
 * cases of movement with both motors.
 * 
 * Original by: RandomNerdTutorials
 * https://randomnerdtutorials.com/esp32-dc-motor-l298n-motor-driver-control-speed-direction/
 * 
 * Modified by: DanielFLopez1620
 */

// -------------------------- REQUIRED HEADERS ---------------------------------
#include <Arduino.h>

// -------------------------- GLOBAL DEFINITIONS -------------------------------
// Motor Right
int motor1Pin1 = 19; 
int motor1Pin2 = 18; 
int enable1Pin = 16; 

// Motor Left
int motor2Pin1 = 21;
int motor2Pin2 = 22;
int enable2Pin = 17;

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

// ------------------------- SINGLE SET UP FUNCTION ---------------------------
void setup() 
{
    // Begin serial
    Serial.begin(115200);

    // Sets motor pins as outputs
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(enable1Pin, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);
    pinMode(enable2Pin, OUTPUT);

    // Set the PWM to consider in the enable pins
    analogWrite(enable1Pin, 120);
    analogWrite(enable2Pin, 120);
    
    // Testing
    Serial.print("Testing DC Motor...");
}

// -------------------------- LOOP FUNCTION ------------------------------------
void loop() 
{
    Serial.println("Moving Forward");
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH); 
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW); 
    delay(1000);

    Serial.println("Stop");
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW); 
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW); 
    delay(1000);

    Serial.println("Moving Backward");
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW); 
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH); 
    delay(1000);

    Serial.println("Stop");
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW); 
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW); 
    delay(1000);

    Serial.println("Gyro Right");
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW); 
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW); 
    delay(1000);

    Serial.println("Stop");
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW); 
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW); 
    delay(1000);

    Serial.println("Gyro Left");
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH); 
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH); 
    delay(1000);

    Serial.println("Stop");
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW); 
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW); 
    delay(1000);

    Serial.println("The one");
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH); 
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW); 
    delay(1000);

    Serial.println("Stop");
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW); 
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW); 
    delay(1000);

    Serial.println("The other one");
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW); 
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH); 
    delay(1000);

    Serial.println("Stop");
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW); 
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW); 
    delay(1000);
}