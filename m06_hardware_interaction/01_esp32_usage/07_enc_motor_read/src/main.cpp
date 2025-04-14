/**
 * Code oriented to determinate the RPMs of a DC motor by considering
 * the encoder lectures (A & B).
 * 
 * Original by: RandomNerdTutorials
 * https://randomnerdtutorials.com/esp32-dc-motor-l298n-motor-driver-control-speed-direction/
 * 
 * Modified by: DanielFLopez1620
 */

// -------------------- REQUIRED HEADERS ----------------------------
#include "Arduino.h"

// -------------------- GLOBAL DEFINITIONS --------------------------

// Ports of the program:
#define ENC_A_PIN 32 // Channel A of motor encoder 
#define ENC_B_PIN 33 // Channel B of motor encoder 

// Volatile counter for encoder readings
volatile long pulse_count = 0;

// Lectures per revolution, varies depending on the motor.
// In my case with a 1000 RPM JGA25-371 DC Motor is 204 aprox.
const int pulses_per_rev = 204;

// Previous time required for vel calculation
unsigned long last_time = 0;

// Previous count of encoder for vel calculation
long last_pulse_count = 0;

// ---------------------- FUNCTION PROTOTYPES ------------------------------

void IRAM_ATTR handleEncoder();

// ------------------------- SINGLE SET UP FUNCTION ---------------------------
void setup() 
{
	// Initialize serial
	Serial.begin(115200);

	// Set up encoder pins
	pinMode(ENC_A_PIN, INPUT);
	pinMode(ENC_B_PIN, INPUT);

	// Attach interrupt on rising edge of channel A
	attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), handleEncoder, RISING);

	// Set up time
	last_time = millis();
}

// ---------------------------- MAIN LOOP DEFINITION --------------------------
void loop() 
{
	// Consider current time and calculate diff
	unsigned long now = millis();
	unsigned long elapsed = now - last_time;

	// Each 100 ms
	if (elapsed >= 100) 
	{
		// Block interrupts to update the pulse count
		noInterrupts();
		long pulses = pulse_count;
		interrupts();

		// Calculate the difference in time
		long delta = pulses - last_pulse_count;

		// Calculate RPM: (delta / pulses_per_rev) / (elapsed / 60000)
		float rpm = (delta * 600.0) / pulses_per_rev;

		// Print message
		Serial.print("RPM: ");
		Serial.println(rpm);

		// Update data before next iteration
		last_pulse_count = pulses;
		last_time = now;
	}
}

// ---------------------------- FUNCTION DEFINITIONS -------------------------

/**
 * According to the state of B when the encoder A is rising, it will consider
 * the increment or decrement of the encouder count.
 * 
 * If A == B, it is counterclockwise
 * If A != B, it is clockwise
 * 
 */
void IRAM_ATTR handleEncoder() 
{
	if (digitalRead(ENC_B_PIN)) 
	{
		pulse_count--;
	} 
	else 
	{
		pulse_count++;
	}
}
