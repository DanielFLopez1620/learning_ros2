/*
 * Code oriented to display the temperature provided by a DHT11 sensor.
 *
 * Modified by: DanielFLopez1620
 *
 * Based on: https://esp32io.com/tutorials/esp32-dht11
 */

// ------------------------------ Required headers ----------------------------
#include <Arduino.h>
#include <DHT.h>

// ------------------------------ Global definitions --------------------------
#define DHT11_PIN  21         // Pin to DHT11 sensor
DHT dht11(DHT11_PIN, DHT11);  // Create object for DHT Sensor
 
// --------------------------- Single setup function --------------------------
void setup() 
{
	Serial.begin(115200);  // Initialize serial
	dht11.begin();         // initialize the DHT11 sensor
}	

// --------------------------- Loop function ---------------------------------
void loop() 
{
	// Read humidity
	float humi  = dht11.readHumidity();
	// Read temperature in Celsius
	float tempC = dht11.readTemperature();
	// Read temperature in Fahrenheit
	float tempF = dht11.readTemperature(true);

	// check whether the reading is successful or not
	if ( isnan(tempC) || isnan(tempF) || isnan(humi)) 
	{
		Serial.println("Failed to read from DHT11 sensor!");
	} 
	else 
	{
		Serial.print("Humidity: ");
		Serial.print(humi);
		Serial.print("%");

		Serial.print("  |  ");

		Serial.print("Temperature: ");
		Serial.print(tempC);
		Serial.print("°C  ~  ");
		Serial.print(tempF);
		Serial.println("°F");
	}

	// wait a 2 seconds between readings
	delay(2000);
}
 