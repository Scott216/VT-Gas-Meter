/*
Code for Adafruit OLED disiplay 128x32
http://www.adafruit.com/products/326

 
If using 128x32 display, you need to uncomment the "#define SSD1306_128_32" in 
the top of Adafruit_SSD1306.h to change the buffer size

Wiring 128x32 display: http://learn.adafruit.com/monochrome-oled-breakouts/wiring-128x32-i2c-display
GND, 5V, SDA, SCL, and Reset to D6. You can change reset pin in the code - see OLED_RESET

For regular size text, you can fit 21 characters on a line, there are 4 lines

You will need a mega if using the display.  If upload to cosm is moved to another device, maybe this will if on an Uno
 
*/

#ifdef USEDISPLAY
  #include <Wire.h>       // for I2C communication
  #include <Adafruit_GFX.h>     // For LCD display, supplies core graphics library http://github.com/adafruit/Adafruit-GFX-Library 
  #include <Adafruit_SSD1306.h> // For LCD display  http://github.com/adafruit/Adafruit_SSD1306

  #define OLED_RESET 6
  Adafruit_SSD1306 display(OLED_RESET);

  #if (SSD1306_LCDHEIGHT != 32)
    #error("Height incorrect, please fix Adafruit_SSD1306.h!");
  #endif
#endif

void setupOled()
{
  #ifdef USEDISPLAY
    // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
    display.clearDisplay();   // clears the screen and buffer
  #endif
} // end setupOled()

// Display text on LCD
void dispText(char textToDisplay[], byte line, bool clearDisp)
{
  #ifdef USEDISPLAY  
    if (clearDisp)
    { display.clearDisplay(); }
    
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(line,0);
    display.println(textToDisplay);
    display.display();
  #else
    Serial.println(textToDisplay);
  #endif
  
} // end dispText()




