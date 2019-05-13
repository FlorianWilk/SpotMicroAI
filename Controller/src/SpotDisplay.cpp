/**************************************************************************
 This is an example for our Monochrome OLEDs based on SSD1306 drivers

 Pick one up today in the adafruit shop!
 ------> http://www.adafruit.com/category/63_98

 This example is for a 128x32 pixel display using I2C to communicate
 3 pins are required to interface (two I2C and one reset).

 Adafruit invests time and resources providing this open
 source code, please support Adafruit and open-source
 hardware by purchasing products from Adafruit!

 Written by Limor Fried/Ladyada for Adafruit Industries,
 with contributions from the open source community.
 BSD license, check license.txt for more information
 All text above, and the splash screen below must be
 included in any redistribution.
 **************************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "SpotDisplay.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET 4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES 10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT 16
#define LOGO_WIDTH 16
static const unsigned char PROGMEM logo_bmp[] =
    {B00000000, B11000000,
     B00000001, B11000000,
     B00000001, B11000000,
     B00000011, B11100000,
     B11110011, B11100000,
     B11111110, B11111000,
     B01111110, B11111111,
     B00110011, B10011111,
     B00011111, B11111100,
     B00001101, B01110000,
     B00011011, B10100000,
     B00111111, B11100000,
     B00111111, B11110000,
     B01111100, B11110000,
     B01110000, B01110000,
     B00000000, B00110000};




void loopDisplay()
{
    display.fillRect(0,8,128,64-8,0);
//    display.fillCircle(32,40,20,1);
    display.display();

}

void testdrawstyles(void)
{
    display.clearDisplay();

    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner
    display.println(F("Hello, world!"));

    display.setTextColor(BLACK, WHITE); // Draw 'inverse' text
    display.println(3.141592);

    display.setTextSize(2); // Draw 2X-scale text
    display.setTextColor(WHITE);
    display.print(F("0x"));
    display.println(0xDEADBEEF, HEX);

    display.display();
    delay(2000);
}

void splashScreen()
{
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.cp437(true);
    display.setCursor(10, 20);
    display.println(F("MICROSPOT"));
    display.setCursor(10, 40);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.display();
    delay(2000);
    display.println(F("WILK DYNAMICS"));
//    display.write(16);
//    display.write(17);
//    display.write(18);
    display.display();
    delay(5000);
    display.clearDisplay();
    display.display();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println(F("MICROSPOT B-001"));
    display.display();
}


void testscrolltext(void)
{
    display.clearDisplay();

    display.setTextSize(2); // Draw 2X-scale text
    display.setTextColor(WHITE);
    display.setCursor(10, 0);
    display.println(F("MICROSPOT"));
    display.display(); // Show initial text
    delay(100);

    // Scroll in various directions, pausing in-between:
    display.startscrollright(0x00, 0x0F);
    delay(2000);
    display.stopscroll();
    delay(1000);
    display.startscrollleft(0x00, 0x0F);
    delay(2000);
    display.stopscroll();
    delay(1000);
    display.startscrolldiagright(0x00, 0x07);
    delay(2000);
    display.startscrolldiagleft(0x00, 0x07);
    delay(2000);
    display.stopscroll();
    delay(1000);
}

void testdrawbitmap(void)
{
    display.clearDisplay();

    display.drawBitmap(
        (display.width() - LOGO_WIDTH) / 2,
        (display.height() - LOGO_HEIGHT) / 2,
        logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
    display.display();
    delay(1000);

    testdrawstyles(); // Draw 'stylized' characters

    testscrolltext(); // Draw scrolling text

    testdrawbitmap(); // Draw a small bitmap image

    // Invert and restore display, pausing in-between
    display.invertDisplay(true);
    delay(1000);
    display.invertDisplay(false);
    delay(1000);
}

void setupDisplay()
{
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    { 
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ; // Don't proceed, loop forever
    }
    display.clearDisplay();
    display.display();
}
