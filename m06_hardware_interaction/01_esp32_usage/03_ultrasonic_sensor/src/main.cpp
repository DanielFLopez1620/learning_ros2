/*
 * Code oriented to notify the pass of a threshold detected by using
 * a Ultrasonic sensor.
 *
 * Modified by DanielFLopez1620
 * 
 * Originally by: https://esp32io.com/tutorials/esp32-ultrasonic-sensor-led
 */

 // --------------------- Required libraries ----------------------------------
#include <Arduino.h>

// ----------------------- Global definitions --------------------------------
#define TRIG_PIN           26  // ESP32 pin GPIO26 to TRIG pin
#define ECHO_PIN           25  // ESP32 pin GPIO25 to ECHO pin
#define LED_PIN            2   // ESP32 pin GPIO17 to LED Built-In
#define DISTANCE_THRESHOLD 20  // Threshold in cm
 
float duration_us, distance_cm;

// ---------------------- Single set up function -----------------------------
void setup() 
{
   Serial.begin (115200);     // initialize serial port (Check platformio.ini)
   pinMode(TRIG_PIN, OUTPUT); // Output mode
   pinMode(ECHO_PIN, INPUT);  // Input mode
   pinMode(LED_PIN, OUTPUT);  // Output mode
}
 
void loop() 
{
	// Generate 10-microsecond pulse to TRIG pin
	digitalWrite(TRIG_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(TRIG_PIN, LOW);
 
	// Measure duration of pulse from ECHO pin by considering sound propagation
	duration_us = pulseIn(ECHO_PIN, HIGH);
	distance_cm = 0.017 * duration_us;
 
	// Check if threshold has been surpassed
	if (distance_cm < DISTANCE_THRESHOLD)
	{
		digitalWrite(LED_PIN, HIGH); 
	}
	else
	{
		digitalWrite(LED_PIN, LOW);  // turn off LED
	}
 
	// Print the value to Serial Monitor
	Serial.print("Distance: ");
	Serial.print(distance_cm);
	Serial.println(" cm");
	
	delay(500);
}
 