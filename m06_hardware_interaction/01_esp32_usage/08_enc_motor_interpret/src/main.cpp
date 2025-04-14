/**
 * Code oriented to use a two-channel (A & B) encoder for reading the velicity
 * of a DC Motor. This code do not consider the motor itself or a driver, as it
 * is intended for manual development.
 * 
 * Based on: Tutorial on Encoders in Arduino (Hall effect encoder with DC motor)
 * by Aleksandar Haber PhD.
 * https://www.youtube.com/watch?v=1PJOzrXAlcg
 * 
 * Modified by: DanielFLopez1620
 */

// -------------------------- REQUIRED HEADERS --------------------------------
#include <Arduino.h>

// ------------------------- GLOBAL DEFINITONS --------------------------------

// Ports of the program:
#define ENC_A_PIN 32  // Channel A of motor encoder
#define ENC_B_PIN 33  // Channel B of motor encoder

// Volatile counter for reading encoder
volatile int counter = 0;

// -------------------------- FUNCTION PROTOTYPES --------------------------

void IRAM_ATTR handleEnc();

// ------------------------- SINGLE SET UP FUNCTION ------------------------
void setup() 
{
    // Initialize Serial
    Serial.begin(115200);

    // Set up pins
    pinMode(ENC_A_PIN, INPUT);
    pinMode(ENC_B_PIN, INPUT);

    // Link change of state of encoder A to a function
    attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), handleEnc, CHANGE);
}

// -------------------------- LOOP IMPLEMENTATION --------------------------
void loop() 
{
    // Print the encoder change:
    Serial.println(counter);
}

// --------------------------- FUNCTION DEFINITIONS ------------------------

/**
 * Handle encoder interrupt for a two-channel encoder. Based on the signal
 * launched by A consider two cases:
 * 
 * If A == B, the direction is clockwise
 * If A != B the direction is counterclockwise
 * 
 * The analysis was made based on oscilloscope visualization
 */
void IRAM_ATTR handleEnc() 
{
    if(digitalRead(ENC_B_PIN) != digitalRead(ENC_A_PIN))
    {
        counter--;
    }
    else
    {
        counter++;
    }
}
