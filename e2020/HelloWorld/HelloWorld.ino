#include <Arduino.h>
#include <U8g2lib.h>

#include <SPI.h>

U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 39, /* dc=*/ 2, /* reset=*/ 3);  // Enable U8G2_16BIT in u8g2.h

void setup(void) {
  u8g2.begin();
}

void loop(void) {
  u8g2.clearBuffer();         // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  u8g2.drawStr(0,10,"hi,xiao qi. i miss you. i <3 you");  // write something to the internal memory
  u8g2.sendBuffer();          // transfer internal memory to the display
  delay(1000);  
}
