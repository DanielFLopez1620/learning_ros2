/*
 * Code oriented to ligth the build-in LED while the touch sensor gives
 * asignal.
 *
 * Modified by DanielFLopez1620.
 *
 * Based on:
 * https://randomnerdtutorials.com/vs-code-platformio-ide-esp32-esp8266-arduino/
 */

// ------------------ Required headers ----------------------------------------
#include <Arduino.h>

// ------------------- Global definitions -------------------------------------
#define LED 2

// ------------------- Single set up function ---------------------------------
void setup() 
{
	// Serial port baud rate, check platformio.ini
	Serial.begin(115200);

	// Set up LED pin
	pinMode(LED, OUTPUT);
}

void loop() 
{
	// First, turn on LED
	digitalWrite(LED, HIGH);
	Serial.println("LED is on");
	delay(1000);

	// Second, turn off LED
	digitalWrite(LED, LOW);
	Serial.println("LED is off");
	delay(1000);
}