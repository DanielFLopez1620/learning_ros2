/*
 * Code oriented to ligth the build-in LED while the touch sensor gives
 * asignal.
 *
 * Modified by DanielFLopez1620.
 *
 * Based on https://esp32io.com/tutorials/esp32-touch-sensor-led
 */

// ---------------------- Required headers ------------------------------------
#include <Arduino.h>

// ---------------------- Global definitions ----------------------------------
#define TOUCH_PIN 33 // ESP32 pin GPIO33 connected for touch sensor
#define LED_PIN   2  // ESP32 Built-in LED Pin
 
// --------------------- Single configuration function ------------------------
void setup() 
{
	Serial.begin(115200);         // Initialize serial (Check platformio.ini)
	pinMode(TOUCH_PIN, INPUT);    // Set ESP32 pin to input mode
	pinMode(LED_PIN, OUTPUT);     // Set ESP32 pin to output mode
}
 
// --------------------- Loop function ---------------------------------------
void loop() 
{
	// Read touch sensor state
	int touchState = digitalRead(TOUCH_PIN); 
	
	// Respond accordingly to the state
	if (touchState == HIGH) 
	{
		Serial.println("The sensor is being touched");;
		digitalWrite(LED_PIN, HIGH);
	} 
	else if (touchState == LOW) 
	{
		Serial.println("The sensor is untouched");
		digitalWrite(LED_PIN, LOW);
	}
}
 