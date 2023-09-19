#include <Arduino.h>
#include <U8g2lib.h>

// init display
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

void setup()
{
  // put your setup code here, to run once:
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 10, "Hello World!");
  u8g2.sendBuffer();
}

void loop()
{
}
