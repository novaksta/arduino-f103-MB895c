#include "lcd.c"
#include <SPI.h>


void setup() {
  // put your setup code here, to run once:
  BSP_LCD_Init();

}

void loop() {
  // put your main code here, to run repeatedly:
	  LCD_demo();

}
