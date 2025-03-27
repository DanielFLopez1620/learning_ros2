/*
 * Code oriented to the usage of the ILI9225 screen in order to display
 * custom faces (eyes based) and tyr to reflect animic states.
 *
 * Modified by DanielFLopez1620.
 *
 * Based on:
 * https://github.com/Nkawu/TFT_22_ILI9225/wiki
 */

// ---------------------------- Required libraries ----------------------------
#include <Arduino.h>
#include <SPI.h>
#include <TFT_22_ILI9225.h>

// --------------------------- Global definitions -----------------------------
// Define SPI Pins
#define TFT_RST        26   // Reset pin
#define TFT_RS         25   // Data/Command pin
#define TFT_CS         15   // Chip Select pin
#define TFT_SDI        13   // MOSI pin
#define TFT_CLK        14   // SCK pin
#define TFT_LED        0    // Should be 0 if wired to +5V
#define TFT_BRIGHTNESS 200  // Brightness level
#define X_INI 45    // X begin for base eyes
#define X_END 125   // X end for base eyes
#define X_OFF 30    // X Offset planned
#define Y1_INI 20   // Y first eye begin for base eye
#define Y2_INI 130  // Y second eye begin for base eye
#define Y_SIZE 70   // Y size of eyes
#define TIME_OUT 2500 // Time out

// Screen definitions
TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, TFT_SDI, TFT_CLK,
    TFT_LED, TFT_BRIGHTNESS);

// -------------------------- Single function configuration -----------------
void setup() 
{
    // Begin serial
    Serial.begin(115200);

    // Start screen and clear
    tft.begin();
    tft.setBackgroundColor(COLOR_BLACK);
    tft.clear();
}

// ------------------------- Loop implementation ---------------------------
void loop() 
{
    // Surprise
    tft.fillCircle((X_END + X_INI)/2, Y1_INI + Y_SIZE/2, Y_SIZE/2, COLOR_YELLOW);
    tft.fillCircle((X_END + X_INI)/2, Y2_INI + Y_SIZE/2, Y_SIZE/2, COLOR_YELLOW);
    delay(TIME_OUT);

    tft.clear();
    delay(100);

    // Inexpressive
    tft.fillRectangle(X_INI + X_OFF, Y1_INI, X_END - X_OFF, Y1_INI + Y_SIZE, COLOR_YELLOW);
    tft.fillRectangle(X_INI + X_OFF, Y2_INI, X_END - X_OFF, Y2_INI + Y_SIZE, COLOR_YELLOW);
    delay(TIME_OUT);

    tft.clear();
    delay(100);

    // Angry
    tft.fillRectangle(X_INI, Y1_INI, X_END, Y1_INI + Y_SIZE, COLOR_ORANGE);
    tft.fillRectangle(X_INI, Y2_INI, X_END, Y2_INI + Y_SIZE, COLOR_ORANGE);
    tft.fillTriangle(X_END, Y1_INI, X_END + 12, Y1_INI, X_END, Y1_INI + Y_SIZE, COLOR_ORANGE);
    tft.fillTriangle(X_END, Y2_INI, X_END, Y2_INI + Y_SIZE, X_END + 12, Y2_INI + Y_SIZE, COLOR_ORANGE);
    delay(TIME_OUT);

    tft.clear();
    delay(100);

    // Sad
    tft.fillRectangle(X_INI, Y1_INI, X_END, Y1_INI + Y_SIZE, COLOR_LIGHTBLUE);
    tft.fillRectangle(X_INI, Y2_INI, X_END, Y2_INI + Y_SIZE, COLOR_LIGHTBLUE);
    tft.fillTriangle(X_END, Y1_INI, X_END, Y1_INI + Y_SIZE, X_END + 10, Y1_INI + Y_SIZE, COLOR_LIGHTBLUE);
    tft.fillTriangle(X_END, Y2_INI, X_END + 10, Y2_INI, X_END, Y2_INI + Y_SIZE, COLOR_LIGHTBLUE);
    delay(TIME_OUT);

    tft.clear();
    delay(100);
    
    // Happy
    tft.fillRectangle(X_INI, Y1_INI, X_END, Y1_INI + Y_SIZE, COLOR_YELLOW);
    tft.fillRectangle(X_INI, Y2_INI, X_END, Y2_INI + Y_SIZE, COLOR_YELLOW);
    tft.fillCircle(X_INI, Y1_INI + Y_SIZE/2, Y_SIZE/2, COLOR_YELLOW);
    tft.fillCircle(X_END, Y1_INI + Y_SIZE/2, Y_SIZE/2, COLOR_YELLOW);
    tft.fillCircle(X_INI, Y2_INI + Y_SIZE/2, Y_SIZE/2, COLOR_YELLOW);
    tft.fillCircle(X_END, Y2_INI + Y_SIZE/2, Y_SIZE/2, COLOR_YELLOW);
    delay(TIME_OUT);

    tft.clear();
    delay(100);
    
    // Neutral
    tft.fillRectangle(X_INI, Y1_INI, X_END, Y1_INI + Y_SIZE, COLOR_YELLOW);
    tft.fillRectangle(X_INI, Y2_INI, X_END, Y2_INI + Y_SIZE, COLOR_YELLOW);
    delay(TIME_OUT);

    tft.clear();
    delay(100);
}
