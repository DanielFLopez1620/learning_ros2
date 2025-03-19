/*
 * Code oriented to the usage of the ILI9225 screen.
 *
 * Modified by DanielFLopez1620.
 *
 * Based on:
 * https://github.com/Nkawu/TFT_22_ILI9225/wiki
 */

 // --------------------------- Required libraries --------------------------------
#include <Arduino.h>
#include <SPI.h>
#include <TFT_22_ILI9225.h>

// ---------------------------- Global defintions --------------------------------
// Define SPI Pins
#define TFT_RST        26   // Reset pin
#define TFT_RS         25   // Data/Command pin
#define TFT_CS         15   // Chip Select pin
#define TFT_SDI        13   // MOSI pin
#define TFT_CLK        14   // SCK pin
#define TFT_LED        0    // Should be 0 if wired to +5V
#define TFT_BRIGHTNESS 200  // Brightness

// Screen object
TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, TFT_SDI,
    TFT_CLK,TFT_LED, TFT_BRIGHTNESS);

// ------------------------ Single config function ----------------------------
void setup() 
{
    // Initialize serial
    Serial.begin(115200);

    // Initialize screen, set up color and clear
    tft.begin();
    tft.setBackgroundColor(COLOR_WHITE);
    tft.clear();

    // Set font and add text
    tft.setFont(Terminal6x8);
    tft.drawText(20, 20, "Hello ESP32!", COLOR_BLACK);
    tft.setBackgroundColor(COLOR_BLACK);
    tft.drawText(20, 40, "Witdh x Height", COLOR_CYAN);
    String scsize = String(tft.maxX()) + " x " + String(tft.maxY());
    tft.drawText(20, 60, scsize, COLOR_CYAN);

    // In my case, the screen size is  176 x 220
}

// --------------------- Loop implementation ---------------------------------
void loop() 
{
    // Loop for circle colors
    tft.fillCircle(88, 110, 20, COLOR_RED);
    delay(1000);
    tft.fillCircle(88, 110, 20, COLOR_BLUE);
    delay(1000);
}
