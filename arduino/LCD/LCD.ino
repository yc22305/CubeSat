#include <LiquidCrystal_I2C_Wire1.h>
#include <Wire.h> 

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup()
{
  lcd.init();                      // initialize the lcd 
  // Print a message to the LCD.
  lcd.backlight();
  lcd.print("CANADA ROBOTIX");
  delay(1000);
  lcd.setCursor(1,1);
  lcd.print("Hello!");
  delay(1000);
  //lcd.clear();
}

void loop()
{
}
