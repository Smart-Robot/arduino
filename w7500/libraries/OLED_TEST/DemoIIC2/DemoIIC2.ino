//

#include <IIC_without_ACK.h>
#include <Adafruit_GFX.h>
#include "oledfont.c"   //codetab

//#define OLED_SDA 8 //Arduino Digital Pin
//#define OLED_SCL 9 //Arduino Digital Pin

#define OLED_SDA 14 //Arduino Digital Pin
#define OLED_SCL 15 //Arduino Digital Pin

IIC_without_ACK lucky(OLED_SDA, OLED_SCL);//9 -- sda,10 -- scl

void setup()
{
  lucky.Initial();
  delay(10);
}

void loop()
{
  lucky.Fill_Screen(0xff);
  delay(2000);
  lucky.Fill_Screen(0xf0);
  delay(2000);
  lucky.Fill_Screen(0x00);
  lucky.Char_F6x8(25,1,"Squirrel-Labs"); 
  lucky.Char_F6x8(0,4,"www.squirrel-labs.net"); 
  lucky.Char_F6x8(8,7,"Testing OLED Screen");  
//  lucky.CN_F16x16(0,0,0);
//  lucky.CN_F16x16(16,0,1);
//  lucky.CN_F16x16(32,0,2);
//  lucky.CN_F16x16(48,0,3);
//  lucky.CN_F16x16(64,0,4);
  delay(5000);
  lucky.Fill_Screen(0x00); 
//  delay(1000);
//  lucky.Fill_Screen(0x00);
  lucky.Draw_BMP(0,1,128,8,BMP1); // run olefont.c
  delay(5000);
}