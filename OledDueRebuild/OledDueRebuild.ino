
#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <DueTimer.h>



//U8G2_SSD1322_NHD_256X64_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);	// Enable U8G2_16BIT in u8g2.h
U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);	// Enable U8G2_16BIT in u8g2.h

#define GCenterX 226 //211-229.5-248
#define GCenterY 21  //0-18-37
#define GCenter18 21 //size
#define GCenter9 11  //half size

bool fastboot=0;
int rpm = 0;
int rpm_last = 0;
int rpm_same = 0;
float spd_hz = 8;
int spd = 0;
int temp = 0;

int GearRatio = 0; //0.43-3.0
//#define GearRatioConstant = 0.17522;
int TimeH = 3;
int TimeMM = 59;
int TimeSS = 59;
int GForceX = 0;
int GForceY = 0;
int GForceXScreen = GCenterX - 1;
int GForceYScreen = GCenterY - 1;
int GearRatioCube = 0;
float Volt = 0.00;
long BoostStartTime=0;
int BootedTime=0;
int BootDistance=0;
int BootedDistance=0;

void setup(void)
{
    Timer3.attachInterrupt(tictoc);//start tictoc
    Timer3.start(1000000); // Calls every 1s
    u8g2.begin(); //from example
    u8g2.clearBuffer();
    bootbmp();
    u8g2.sendBuffer();
    delay(3200);
    u8g2.clearBuffer();
}

void loop(void)
{

    // u8g2.clearBuffer();                  // clear the internal memory
    // u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font
    // u8g2.drawStr(0, 10, "Hello World!"); // write something to the internal memory
    // u8g2.sendBuffer();                   // transfer internal memory to the display
    // delay(10);

    //
    //GetNewData
    spd = random(10, 50);
    temp = random(28, 50);
    rpm = random(2700, 3800);
    GForceX = random(-900, 900);
    GForceY = random(-300, 300);
    GForceXScreen = GForceX / 60 + GCenterX - 1;
    GForceYScreen = GForceY / 60 + GCenterY - 1;
    GearRatio = random(0, 9);
    Volt -= 0.01;
    //

    //Post Process
    if (rpm_last == rpm) //Drop Outdate RPM
    {
        if (rpm_same >= 5)
            rpm = 0;
        else
            rpm_same++;
    }

    if (TimeSS == -1)
    {
        TimeSS = 59;
        TimeMM--;
    }
    if (TimeMM == -1)
    {
        TimeMM = 59;
        TimeH--;
    }
    if (TimeH == -1)
    {
        TimeH = 0;
        TimeMM = 0;
        TimeSS = 0;
    }

//
// frames
//

 u8g2.clearBuffer();

// u8g2.setFont(u8g2_font_6x10_tf);
//   u8g2.drawStr(109, 7, "Speed");
//   u8g2.drawStr(151, 7, " Temp");
//   u8g2.drawStr(68, 7, "Gear");
//   u8g2.drawStr(0, 64, "RPM:");
//   u8g2.drawStr(205, 53, "Gx:");
//   u8g2.drawStr(205, 61, "Gy:");
//   u8g2.drawStr(0, 7, "Volt");

// #define GCenterX 220 //211-229.5-248
// #define GCenterY 21  //0-18-37
// #define GCenter18 21 //size
// #define GCenter9 11  //half size

//   //GForceCube:
//   u8g2.drawFrame(GCenterX - GCenter18, GCenterY - GCenter18, 2 * GCenter18 + 1, 2 * GCenter18 + 1); //211-229.5-248
//   u8g2.drawFrame(GCenterX - GCenter9, GCenterY - GCenter9, 2 * GCenter9 + 1, 2 * GCenter9 + 1);     //220-229.5-239
//   u8g2.drawLine(GCenterX, GCenterY - GCenter18, GCenterX, GCenterY - 1);
//   u8g2.drawLine(GCenterX, GCenterY + 1, GCenterX, 2 * GCenter18);
//   u8g2.drawLine(GCenterX - GCenter18, GCenterY, GCenterX - 1, GCenterY);
//   u8g2.drawLine(GCenterX + 1, GCenterY, GCenterX + GCenter18, GCenterY);

//   //Temp Cube:
//   u8g2.drawFrame(152, 11, 4, 19); //211-229.5-248
//                                   //Gear Cube:
//   u8g2.drawFrame(67, 11, 4, 19);  //211-229.5-248
//                                   //Speed Cube:
//   u8g2.drawFrame(100, 0, 5, 47);  //211-229.5-248

//   u8g2.drawBox(GForceXScreen, GForceYScreen, 3, 3); //GForceBox

  //-----
  //DATAS:
  //----

 //temp
 if (temp > 40)
     u8g2.drawStr(150, 7, "!");
 u8g2.drawBox(153, 30 - map(temp, 20, 60, 1, 19), 2, map(temp, 20, 60, 1, 19));
 //gear
 u8g2.drawBox(68, 30 - map(GearRatio, 0, 9, 1, 19), 2, map(GearRatio, 0, 9, 1, 19));
 //speed
 u8g2.drawBox(101, 47 - map(spd, 0, 60, 1, 47), 3, map(spd, 0, 60, 1, 47));

 u8g2.setFont(u8g2_font_6x10_tf);
 u8g2.drawStr(222, 53, itostr4(GForceX)); //gforcex
 u8g2.drawStr(222, 61, itostr4(GForceY)); //gforcey

 if (rpm >= 999 && rpm <= 4000)
 {
     u8g2.drawStr(25, 64, itostr4(rpm)); //rpm
 }
 else
     //data.error

 u8g2.setFont(u8g2_font_logisoso18_tr);
 u8g2.drawStr(0, 52, itostrtime());    //time
 u8g2.drawStr(157, 29, itostr2(temp)); //temp
 if (GearRatio == 0)
     u8g2.drawStr(72, 29, "N");
 else
     u8g2.drawStr(72, 29, itostr2(GearRatio)); //Gear

 u8g2.drawStr(0, 29, ftostr4(Volt)); //battery

 u8g2.setFont(u8g2_font_logisoso34_tn);
 u8g2.drawStr(105, 47, itostr2(spd)); //speed}

u8g2.sendBuffer();

}


void bootbmp(void)
{
static const unsigned char boot_bits[] U8X8_PROGMEM = {
     
    0x70, 0xa2, 0xe7, 0xe0, 0x21, 0x20, 0x02, 0x8e, 0xa3, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x88, 0xa2, 0x10, 0x21, 0x22, 0x20, 0x02, 0x51, 0xa4, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x88, 0xa2, 0x10, 0x21, 0x22, 0x20, 0x02, 0x51, 0xb4, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x08, 0xa2, 0x10, 0x20, 0x52, 0x20, 0x05, 0x50, 0x24, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x10, 0xa2, 0x20, 0xe0, 0x51, 0x20, 0x05, 0x48, 0x24, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x20, 0xa2, 0x47, 0x20, 0x52, 0x20, 0x05, 0x48, 0x24, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0xa2, 0x80, 0x20, 0x52, 0x20, 0x05, 0x44, 0x24, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x80, 0xa2, 0x00, 0x21, 0x52, 0x22, 0x05, 0x42, 0x24, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x88, 0xa2, 0x10, 0x21, 0xfa, 0xa2, 0x0f, 0x42, 0x24, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x88, 0xa2, 0x10, 0x21, 0x8a, 0xa2, 0x08, 0x41, 0x24, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x70, 0x9c, 0xe7, 0xe0, 0x89, 0x9c, 0x08, 0x9f, 0x23, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xe0, 0x0f, 0x7f, 0xf0, 0xf1, 0x0f, 0xc3, 0x00, 0xfe, 0xe1, 0x03, 0xf8, 0x03, 0x1f, 0x7c, 0xf0, 0x07,
    0xe0, 0x1f, 0x7f, 0xfc, 0xf3, 0x1f, 0xe3, 0x00, 0xff, 0xf9, 0x07, 0xf8, 0xc7, 0x3f, 0xff, 0xf0, 0x07,
    0x70, 0x9c, 0x03, 0x8e, 0x33, 0x1c, 0x63, 0x00, 0x18, 0x1c, 0x07, 0x1c, 0xe7, 0x38, 0xe3, 0x38, 0x00,
    0x30, 0x9c, 0x01, 0x86, 0x33, 0x1c, 0x33, 0x00, 0x18, 0x0c, 0x07, 0x0c, 0x67, 0xb8, 0xe3, 0x18, 0x00,
    0x30, 0x8c, 0x01, 0x86, 0x39, 0x0c, 0x33, 0x00, 0x18, 0x0c, 0x03, 0x0c, 0x63, 0x98, 0x61, 0x18, 0x00,
    0x38, 0x8e, 0x01, 0x87, 0x19, 0x0c, 0x1b, 0x00, 0x1c, 0x0e, 0x03, 0x8e, 0x73, 0x98, 0x01, 0x18, 0x00,
    0x38, 0xce, 0x01, 0xc7, 0x19, 0x0e, 0x1b, 0x00, 0x0c, 0x8e, 0x03, 0x8e, 0x73, 0xdc, 0x01, 0x1c, 0x00,
    0x18, 0xcf, 0x00, 0xc3, 0x18, 0x0e, 0x0f, 0x00, 0x0c, 0x86, 0x01, 0xc6, 0x33, 0xcc, 0x00, 0x0c, 0x00,
    0x18, 0xc7, 0x1f, 0xc3, 0x1c, 0x06, 0x0f, 0x00, 0x0c, 0x86, 0x01, 0xc6, 0x31, 0xcc, 0x00, 0xfc, 0x01,
    0xfc, 0xc7, 0x8f, 0xff, 0x0c, 0x07, 0x07, 0x00, 0x0e, 0x87, 0x01, 0xff, 0xf9, 0xcf, 0x00, 0xfc, 0x00,
    0xfc, 0xe1, 0x8f, 0xff, 0x0c, 0x07, 0x07, 0x00, 0x06, 0xc3, 0x01, 0x7f, 0xf8, 0xef, 0x00, 0xfe, 0x00,
    0xcc, 0x60, 0x80, 0x61, 0x0e, 0x03, 0x03, 0x00, 0x06, 0xc3, 0x00, 0x33, 0x18, 0x66, 0x00, 0x06, 0x00,
    0xcc, 0x60, 0xc0, 0x61, 0x0e, 0x03, 0x03, 0x00, 0x06, 0xc3, 0x00, 0x33, 0x1c, 0x66, 0x00, 0x06, 0x00,
    0xce, 0x70, 0xc0, 0x61, 0x86, 0x03, 0x03, 0x00, 0x87, 0xe3, 0x80, 0x33, 0x1c, 0x76, 0x00, 0x07, 0x00,
    0xc6, 0x70, 0xc0, 0x70, 0x86, 0x83, 0x03, 0x00, 0x83, 0xe1, 0x80, 0x31, 0x0c, 0x77, 0x10, 0x07, 0x00,
    0xc6, 0x30, 0xc0, 0x30, 0x87, 0x81, 0x01, 0x00, 0x83, 0x61, 0x80, 0x31, 0x0c, 0x33, 0x0c, 0x03, 0x00,
    0xc7, 0x30, 0xe0, 0x30, 0x87, 0x81, 0x01, 0x00, 0x83, 0x61, 0xc0, 0x31, 0x0e, 0x33, 0x0c, 0x03, 0x00,
    0xc7, 0xf8, 0xe7, 0x38, 0xff, 0x80, 0x01, 0x80, 0xc3, 0x3f, 0xc0, 0x31, 0x8e, 0xf3, 0x87, 0x7f, 0x00,
    0xc3, 0xf8, 0x63, 0x38, 0x7f, 0xc0, 0x01, 0x80, 0x81, 0x1f, 0xc0, 0x30, 0x86, 0xf3, 0x83, 0x3f, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x20, 0xa0, 0x00, 0xfe, 0x00, 0xc8, 0x01, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x20, 0xe0, 0x00, 0xaa, 0x00, 0x88, 0x00, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x20, 0x7e, 0xe7, 0xbf, 0xee, 0x8e, 0xdc, 0xfd, 0x77, 0x07, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x20, 0x2a, 0xa1, 0x6a, 0x8a, 0x8a, 0x54, 0x55, 0x55, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x20, 0x2e, 0xa2, 0xaa, 0xea, 0x8a, 0x54, 0x5d, 0x75, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xa0, 0x22, 0xa4, 0xaa, 0xaa, 0x8a, 0xd4, 0x45, 0x15, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xe0, 0x6e, 0xe7, 0xaa, 0xee, 0x8e, 0x1c, 0xdd, 0x75, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
#define boot_width 131
#define boot_height 60
u8g2.drawXBMP(65, 0, boot_width, boot_height, boot_bits); 
};

void tictoc() //倒计时读秒
{
  TimeSS--;
}



char *ftostr4(float i)
{
  char buff[13];
  //sprintf(buff, "%fV",i);
  dtostrf(i, 1, 2, buff);
//  strcat(buff, "V");
  return buff;
}

char *itostr2(int i)
{
  char buff[3];
  sprintf(buff, "%02d", i);
  //itoa(i, buff, 10);

  return buff;
}

char *itostr4(int i)
{
  char buff[4];
  sprintf(buff, "%#04d", i);
  //itoa(i, buff, 10);
  return buff;
}

char *itostrtime()
{
  char buff[8];
  sprintf(buff, "%02d:%02d:%02d", TimeH, TimeMM, TimeSS);
  return buff;
}

char *dtostrf(double val, signed char width, unsigned char prec, char *sout)
{
  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}