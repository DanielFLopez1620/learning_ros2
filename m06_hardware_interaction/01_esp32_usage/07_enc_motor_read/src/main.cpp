#include "Arduino.h"

// Encoder pins
const int ENC_A_PIN = 32;
const int ENC_B_PIN = 33;

// Encoder pulse counter
volatile long pulse_count = 0;

// Encoder parameters
const int pulses_per_rev = 400; // Change based on your encoder

// For speed calculation
unsigned long last_time = 0;
long last_pulse_count = 0;

void IRAM_ATTR handleEncoder() 
{
  // Determine direction: based on the state of channel B
  if (digitalRead(ENC_B_PIN)) 
  {
    pulse_count--;
  } 
  else 
  {
    pulse_count++;
  }
}

void setup() {
  Serial.begin(115200);

  // Set up encoder pins
  pinMode(ENC_A_PIN, INPUT);
  pinMode(ENC_B_PIN, INPUT);

  // Attach interrupt on rising edge of channel A
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), handleEncoder, RISING);

  last_time = millis();
}

void loop() {
  unsigned long now = millis();
  unsigned long elapsed = now - last_time;

  if (elapsed >= 100) { // Every 100ms
    noInterrupts();
    long pulses = pulse_count;
    interrupts();

    long delta = pulses - last_pulse_count;

    // Calculate RPM: (delta / pulses_per_rev) / (elapsed / 60000)
    float rpm = (delta * 600.0) / pulses_per_rev;

    Serial.print("RPM: ");
    Serial.println(rpm);

    last_pulse_count = pulses;
    last_time = now;
  }
}
