#include <DueFlashStorage.h>
#include <efc.h>
#include <flash_efc.h>

//2018.8.15.3
#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <DueTimer.h>
#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <include/twi.h>
#include <ResponsiveAnalogRead.h>
#include <Adafruit_MLX90614.h>
#include <Encoder.h>



//Serial0.debug; 
//Serial1.rpm;
//Serial2.GPS;
//Serial3.Wireless;

//Pin2 rear  speed sonsor
//Pin3 Front speed sonsor
DueFlashStorage dueFlashStorage;


U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);	
//!!!IMPORTANT!!! GO Enable U8G2_16BIT in u8g2.h

#define GCenterX 233 //211-229.5-248
#define GCenterY 21  //0-18-37
#define GCenter18 21 //size
#define GCenter9 11  //half size

// BMX055 Accl I2C address is 0x18(24)
#define Addr_Accl 0x18
// BMX055 Gyro I2C address is 0x68(104)
#define Addr_Gyro 0x68
// BMX055 Mag I2C address is 0x10(16)
#define Addr_Mag 0x10



bool fastboot=0;
int rpm = 0;
int rpm_last = 0;
int rpm_same = 0;
int SpeedFF = 0;
int SpeedRR = 0;
int temp = 0;

int GearRatio = 0; //0.43-3.0
int TimeH = 3;
int TimeMM = 59;
int TimeSS = 59;
int GForceX = 0;
int GForceY = 0;
int GForceXScreen = GCenterX - 1;
int GForceYScreen = GCenterY - 1;
float Volt = 3.30;

double wheelrange=0.0; //  0.1755522 meter per rotation
float wheelrangeContant = 0.003;//0.00292587 at beganing 0.003
int wheelrangeContantStore = 30;






ResponsiveAnalogRead analog(A0, true);



/*
Buttons:


Button1.    Sound Power                 LED.    D32
Button2.    Radio                       LED.    D30
Button3.    k3a.        D34.            LED.    D28
Button4.    Glass Power                 LED.    D26
Button5.    k5a.        Main Power      LED.    D24
Button6.    k6a.        NanoDemo.D4.    LED.    D22

BUZ:
BUZ1:   D31
BUZ2:   DAC0    D35.    NanoDemo.D3.

Tone1:D10.D39.
Tone2:D11.D37.
Tone3:D12.D35.

*/

// int rpmFF;
// int rpm_lastFF;
// int calFF = 10;
// volatile unsigned long lastPulseTimeFF;
// volatile unsigned long intervalFF = 0;
// const int timeoutValueFF = 10;
// volatile int timeoutCounterFF;
// int justfixedFF;

// long totalFF = 0; // the running total
// const int numReadingsFF = 5;
// int rpmarrayFF[numReadingsFF];
// int indexFF = 0;    // the index of the current reading
// long averageFF = 0; // the average

long rpmRR;
long rpm_lastRR;
int calRR = 38;
volatile unsigned long lastPulseTimeRR;
volatile unsigned long intervalRR = 0;
const int timeoutValueRR = 5;
volatile int timeoutCounterRR;
int justfixedRR;

long totalRR = 0; // the running total
const int numReadingsRR = 5;
int rpmarrayRR[numReadingsRR];
int indexRR = 0;    // the index of the current reading
long averageRR = 0; // the average

String lastWords = "-=SUES BAJA 2018 =-";

const unsigned char UBX_HEADER[] = {0xB5, 0x62};

struct NAV_PVT
{

    unsigned char cls;
    unsigned char id;
    unsigned short len;
    unsigned long iTOW; // GPS time of week of the navigation epoch (ms)

    unsigned short year;   // Year (UTC)
    unsigned char month;   // Month, range 1..12 (UTC)
    unsigned char day;     // Day of month, range 1..31 (UTC)
    unsigned char hour;    // Hour of day, range 0..23 (UTC)
    unsigned char minute;  // Minute of hour, range 0..59 (UTC)
    unsigned char second;  // Seconds of minute, range 0..60 (UTC)
    char valid;            // Validity Flags (see graphic below)
    unsigned long tAcc;    // Time accuracy estimate (UTC) (ns)
    long nano;             // Fraction of second, range -1e9 .. 1e9 (UTC) (ns)
    unsigned char fixType; // GNSSfix Type, range 0..5
    char flags;            // Fix Status Flags
    char flags2;           // reserved
    unsigned char numSV;   // Number of satellites used in Nav Solution

    long lon;           // Longitude (deg)
    long lat;           // Latitude (deg)
    long height;        // Height above Ellipsoid (mm)
    long hMSL;          // Height above mean sea level (mm)
    unsigned long hAcc; // Horizontal Accuracy Estimate (mm)
    unsigned long vAcc; // Vertical Accuracy Estimate (mm)

    long velN;             // NED north velocity (mm/s)
    long velE;             // NED east velocity (mm/s)
    long velD;             // NED down velocity (mm/s)
    long gSpeed;           // Ground Speed (2-D) (mm/s)
    long headMot;          // Heading of motion 2-D (deg)
    unsigned long sAcc;    // Speed Accuracy Estimate
    unsigned long headAcc; // Heading Accuracy Estimate
    unsigned short pDOP;   // Position dilution of precision
    unsigned short reserved1;
    unsigned long reserved;
    long headVeh;
    short magDec;          // Reserved
    unsigned short magACC; // Reserved
};

NAV_PVT pvt;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

Encoder myEnc(27, 29);
long oldPosition  = -999;


int MenuPage=0;
boolean EnterPageSetting=false;
boolean pushingButton=false;

void setup(void)
{
  myEnc.write(0);
  
    wheelrangeContantStore = dueFlashStorage.read(1);
    wheelrangeContant = (wheelrangeContantStore/1000.1);
    
    Serial.begin(115200);  //debug
        Serial.print("Serial Boot Success");
    Serial1.begin(115200); //rpm
        Serial.print("RPM Connection Success");
    Serial2.begin(115200); //GPS
        Serial.print("GPS Connection Success");
    Serial3.begin(9600);   //Wireless
    Serial3.print("Wireless Connection Success");   //Wireless
    Serial.print("Wireless Connection Success");

    pinMode(32,OUTPUT); //LED For Button1. 
    pinMode(30,OUTPUT); //LED For Button2. 
    pinMode(28,OUTPUT); //LED For Button3. 
    pinMode(26,OUTPUT); //LED For Button4. 
    pinMode(24,OUTPUT); //LED For Button5. 
    pinMode(22,OUTPUT); //LED For Button6. 

    pinMode(31,OUTPUT); //BUZ1
    pinMode(35,OUTPUT); //BUZ2
    pinMode(39,OUTPUT); //BUZ2 Tone1
    pinMode(37,OUTPUT); //BUZ2 Tone2
    pinMode(35,OUTPUT); //BUZ2 Tone3

    pinMode(23,OUTPUT); //HC12 SetKey
    

    pinMode(34,INPUT_PULLUP); //Button3.

    pinMode(2, INPUT); //Pin2 Front  speed sonsor
    pinMode(3, INPUT); //Pin3 Rear   speed sonsor

    pinMode(A1, INPUT); //PinA1 Front speed sonsor
    pinMode(A2, INPUT); //PinA2 Rear speed sonsor

    pinMode(25, INPUT_PULLUP); //ROT Button
    pinMode(27, INPUT); //ROT 1
    pinMode(29, INPUT); //ROT 2
    

    attachInterrupt(digitalPinToInterrupt(3), sensorIsrRR, FALLING);
    // attachInterrupt(digitalPinToInterrupt(2), sensorIsrFF, FALLING);

    attachInterrupt(digitalPinToInterrupt(25), MenuButtonPressed, FALLING);

    Serial.print("Interrupt Success");

    digitalWrite(32,HIGH);  //LED For Button1. 
    digitalWrite(30,HIGH);  //LED For Button2. 
    digitalWrite(28,HIGH);  //LED For Button3. 
    digitalWrite(26,HIGH);  //LED For Button4. 
    digitalWrite(24,HIGH);  //LED For Button5. 
    digitalWrite(22,HIGH);  //LED For Button6. 
    digitalWrite(31,HIGH);  //BUZ1
    digitalWrite(23,HIGH);  //HC12 SET Key

    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.sendBuffer();
    bootbmp();
    u8g2.sendBuffer();
    
    digitalWrite(32,LOW);  //LED For Button1. 
    digitalWrite(30,LOW);  //LED For Button2. 
    digitalWrite(28,LOW);  //LED For Button3. 
    digitalWrite(26,LOW);  //LED For Button4. 
    digitalWrite(24,LOW);  //LED For Button5. 
    digitalWrite(22,LOW);  //LED For Button6. 
    digitalWrite(31,LOW);  //BUZ1

    bmx055Setup();

    Serial.print("9DOF Sensor Boot Success");


    mlx.begin();

    u8g2.clearBuffer();
    u8g2.sendBuffer();
}

void loop(void)
{

    if (MenuPage > 0)
    {
        if (MenuPage == 1 && pushingButton) //in the exit page,going out
        {
            delay(250);
            MenuPage = 0;
            pushingButton = 0;
            EnterPageSetting = 0;
            delay(150);
        }

        if (EnterPageSetting == 0)
        {
            if (pushingButton)
            {
                EnterPageSetting = 1;
                pushingButton = 0;
                delay(150);
            }

            long newPosition = myEnc.read();

            if (newPosition > oldPosition)
            {
                oldPosition = newPosition;
                Serial.print(" Pos:");
                Serial.print(newPosition);
                MenuPage++;
            }
            else if (newPosition < oldPosition && MenuPage > 1)
            {
                oldPosition = newPosition;
                MenuPage--;
            }
        }
        else
        {
            if (pushingButton)
            {
                EnterPageSetting = 0;
                pushingButton = 0;
                delay(150);
            }
        }

        u8g2.setFont(u8g2_font_logisoso34_tn);
        u8g2.setCursor(105, 47);
        u8g2.print(MenuPage);
        u8g2.print(EnterPageSetting);

        Serial.print(" read:");
        Serial.print(myEnc.read());
        Serial.print(" EnterPageSetting:");
        Serial.print(EnterPageSetting);
        Serial.print(" MenuPage:");
        Serial.print(MenuPage);
        Serial.println(" ");

        u8g2.sendBuffer();
        u8g2.clearBuffer();

        delay(150);
    }
    else
    {
        if (pushingButton == true)
        {

            Serial.println("here13");
            MenuPage = 1;
            // digitalWrite(31, HIGH); //BUZ1
            // delay(100);
            // digitalWrite(31, LOW); //BUZ1
            Serial.println("here3");
            pushingButton = false;
        }

        u8g2.clearBuffer();
        //
        //GetNewData

        // rpmFF = long(60e7 / calFF) / (float)intervalFF;
        // digitalWrite(32, LOW); //LED For Button1.
        rpmRR = long(6319879.2 / calRR) / (float)intervalRR;
        digitalWrite(30, LOW); //LED For Button5.

        // //Front Speed calculation

        // if (timeoutCounterFF > 0)
        //     timeoutCounterFF--;
        // if (timeoutCounterFF <= 0)
        // {
        //     //        rpmFF = 0;
        // }

        // if (((rpmFF > (rpm_lastFF + (rpm_lastFF * .2))) || (rpmFF < (rpm_lastFF - (rpm_lastFF * .2)))) && (rpm_lastFF > 0) && (justfixedFF < 3))
        // {
        //     rpmFF = rpm_lastFF;
        //     justfixedFF ++ ;
        // }
        // else
        // {
        //     rpm_lastFF = rpmFF;
        //     justfixedFF--;
        //     if (justfixedFF <= 0)
        //         justfixedFF = 0;
        // }

        // totalFF = totalFF - rpmarrayFF[indexFF];
        // rpmarrayFF[indexFF] = rpmFF;
        // totalFF = totalFF + rpmarrayFF[indexFF];
        // indexFF = indexFF + 1;
        // if (indexFF >= numReadingsFF)
        //     indexFF = 0;
        // averageFF = totalFF / numReadingsFF;
        // SpeedFF = averageFF;

        //rear speed calculation

        if (timeoutCounterRR > 0)
            timeoutCounterRR--;
        if (timeoutCounterRR <= 0)
        {
            rpmRR = 0;
        }

        if (((rpmRR > (rpm_lastRR + (rpm_lastRR * .2))) || (rpmRR < (rpm_lastRR - (rpm_lastRR * .2)))) && (rpm_lastRR > 0) && (justfixedRR < 3))
        {
            rpmRR = rpm_lastRR;
            justfixedRR++;
        }
        else
        {
            rpm_lastRR = rpmRR;
            justfixedRR--;
            if (justfixedRR <= 0)
                justfixedRR = 0;
        }

        totalRR = totalRR - rpmarrayRR[indexRR];
        rpmarrayRR[indexRR] = rpmRR;
        totalRR = totalRR + rpmarrayRR[indexRR];
        indexRR = indexRR + 1;
        if (indexRR >= numReadingsRR)
            indexRR = 0;
        averageRR = totalRR / numReadingsRR;
        if (averageRR < 100)
        {
            SpeedRR = averageRR;
            SpeedFF = averageRR;
        }
        Serial.println("getting Datas");

        getNewDataFromRPM();
        Serial.println("getting Wifi");
        getNewDataFromWifi();
        Serial.println("getting BMX");
        getNewDataFromBMX();
        Serial.println("got Gforce");
        GForceXScreen = GForceX / 90 + GCenterX - 1;
        GForceYScreen = GForceY / 90 + GCenterY - 1;

        Serial.println("getting temp");
        temp = mlx.readObjectTempC();

        // GearRatio = SpeedRR / 84.4; //#define GearRatioConstant = 0.17522;
        // GearRatio = GearRatio / rpm;
        GearRatio = SpeedRR * 611.1;
        GearRatio = GearRatio / rpm;

        analog.update();
        Volt = analog.getValue();
        Volt = Volt * 0.0064453125;
        // Volt = Volt / 1024;
        // Volt = Volt * 6.6;

        // getNewDataFromGPS();

        Serial.println("data got");

        //Serial3.print("R");Serial3.print(rpm);Serial3.print("@");
        Serial.print("R");
        Serial.print(rpm);
        Serial.print("@");
        Serial.print(" Rsame");
        Serial.print(rpm_same);
        Serial.print(" Rlast");
        Serial.print(rpm_last);

        tmElements_t tm;

        //data postProcess
        if (rpm_last == rpm) //Drop Outdate RPM
        {
            if (rpm_same >= 5)
                rpm = 0;
            else
                rpm_same++;
        }
        else
        {
            rpm_same = 0;
        }
        rpm_last = rpm;
        Serial.print("Ra");
        Serial.print(rpm);
        Serial.print("@");
        Serial.print(" Rsamea");
        Serial.print(rpm_same);
        Serial.print(" Rlasta");
        Serial.print(rpm_last);

        if (RTC.read(tm))
        {
            TimeH = tm.Hour;
            TimeMM = tm.Minute;
            TimeSS = tm.Second;
            Serial.print("time updated");
        }

        //
        // frames
        //
        Serial.print("Starting draw datas");

        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(109, 7, "Speed");
        u8g2.drawStr(151, 7, " Temp");
        u8g2.drawStr(68, 7, "Gear");
        u8g2.drawStr(0, 64, "RPM:");
        u8g2.drawStr(213, 53, "Gx:");
        u8g2.drawStr(213, 61, "Gy:");
        u8g2.drawStr(0, 7, "Volt");

#define GCenterX 233 //211-229.5-248
#define GCenterY 21  //0-18-37
#define GCenter18 21 //size
#define GCenter9 11  //half size

        //GForceCube:
        u8g2.drawFrame(GCenterX - GCenter18, GCenterY - GCenter18, 2 * GCenter18 + 1, 2 * GCenter18 + 1); //211-229.5-248
        u8g2.drawFrame(GCenterX - GCenter9, GCenterY - GCenter9, 2 * GCenter9 + 1, 2 * GCenter9 + 1);     //220-229.5-239
        u8g2.drawLine(GCenterX, GCenterY - GCenter18, GCenterX, GCenterY - 1);
        u8g2.drawLine(GCenterX, GCenterY + 1, GCenterX, 2 * GCenter18);
        u8g2.drawLine(GCenterX - GCenter18, GCenterY, GCenterX - 1, GCenterY);
        u8g2.drawLine(GCenterX + 1, GCenterY, GCenterX + GCenter18, GCenterY);

        //Temp Cube:
        u8g2.drawFrame(152, 11, 4, 19); //211-229.5-248
        //Gear Cube:
        u8g2.drawFrame(67, 11, 4, 19); //211-229.5-248
        //Speed Cube:
        u8g2.drawFrame(100, 0, 5, 47);                    //211-229.5-248
        u8g2.drawBox(GForceXScreen, GForceYScreen, 3, 3); //GForceBox
        Serial.print("Frame drawed");
        //-----
        //DRAW DATAS:
        //----

        //temp
        if (temp > 50)
            u8g2.drawStr(150, 7, "!");
        u8g2.drawBox(153, 30 - map(temp, 20, 70, 1, 19), 2, map(temp, 20, 70, 1, 19));
        //gear
        u8g2.drawBox(68, 30 - map(GearRatio, 0, 9, 1, 19), 2, map(GearRatio, 0, 9, 1, 19));
        //speed
        u8g2.drawBox(101, 47 - map(SpeedFF, 0, 60, 1, 47), 3, map(SpeedFF, 0, 60, 1, 47));

        u8g2.setFont(u8g2_font_6x10_tf);

        u8g2.setCursor(230, 53);
        if (GForceX > 0)
            u8g2.print(0);
        u8g2.print(GForceX); //gforcex
        u8g2.setCursor(230, 61);
        if (GForceY > 0)
            u8g2.print(0);
        u8g2.print(GForceY); //gforcey

        u8g2.setCursor(25, 64);
        if ((rpm <= 5000) && (rpm > 1000))
        {
            u8g2.print(rpm); //rpm
            //Serial3.print("R");Serial3.print(rpm);Serial3.print("@");
        }
        else
        {
            u8g2.print("0000"); //rpm
            //Serial3.print("R");Serial3.print("0000");Serial3.print("@");
        }
        u8g2.setCursor(53, 64);
        u8g2.print("TRIP:"); //0.00292587
        u8g2.print(wheelrange);
        u8g2.print("M");

        u8g2.print("  ");
        u8g2.print(lastWords);

        u8g2.setFont(u8g2_font_logisoso18_tr);
        u8g2.setCursor(0, 52);
        display2digits(TimeH); //time
        u8g2.print(":");
        display2digits(TimeMM); //time
        u8g2.print(":");
        display2digits(TimeSS); //time

        u8g2.setCursor(157, 29);
        u8g2.print(temp);
        Serial3.print("T");
        Serial3.print(temp);
        Serial3.print("@");

        if (GearRatio == 0)
        {
            u8g2.setCursor(72, 29);
            u8g2.print("N");
        }
        else
        {
            u8g2.setCursor(72, 29);
            u8g2.print(GearRatio);
        }
        //Serial3.print("G");Serial3.print(GearRatio);Serial3.print("@");

        u8g2.setCursor(0, 29);
        u8g2.print(Volt); //battery
        u8g2.print("V");  //battery
        Serial3.print("V");
        Serial3.print(Volt);
        Serial3.print("@");

        u8g2.setFont(u8g2_font_logisoso34_tn);
        u8g2.setCursor(105, 47); //speed
        u8g2.print(SpeedFF);
        Serial3.print("S");
        Serial3.print(SpeedFF);
        Serial3.print("@");

        Serial3.println("");
        u8g2.sendBuffer();
        Serial.println("DATA Refreshed!");
    }
}
// void sensorIsrFF()
// {
//     unsigned long nowFF = micros();
//     intervalFF = nowFF - lastPulseTimeFF;
//     lastPulseTimeFF = nowFF;
//     timeoutCounterFF = timeoutValueFF;
//     digitalWrite(32, HIGH); //LED For Button1.
// }

void sensorIsrRR()
{
    unsigned long nowRR = micros();
    intervalRR = nowRR - lastPulseTimeRR;
    lastPulseTimeRR = nowRR;
    timeoutCounterRR = timeoutValueRR;
    digitalWrite(30,HIGH);  //LED For Button2. 
    wheelrange+=wheelrangeContant;//0.00292587
}

void MenuButtonPressed()
{
    if (pushingButton == false)
        pushingButton = true;
}

/*************************************************************************
//Function to calculate the distance between two waypoints
 *************************************************************************/
float calc_dist(float flat1, float flon1, float flat2, float flon2)
{
    float dist_calc = 0;
    float dist_calc2 = 0;
    float diflat = 0;
    float diflon = 0;

    //I've to spplit all the calculation in several steps. If i try to do it in a single line the arduino will explode.
    diflat = radians(flat2 - flat1);
    flat1 = radians(flat1);
    flat2 = radians(flat2);
    diflon = radians((flon2) - (flon1));

    dist_calc = (sin(diflat / 2.0) * sin(diflat / 2.0));
    dist_calc2 = cos(flat1);
    dist_calc2 *= cos(flat2);
    dist_calc2 *= sin(diflon / 2.0);
    dist_calc2 *= sin(diflon / 2.0);
    dist_calc += dist_calc2;

    dist_calc = (2 * atan2(sqrt(dist_calc), sqrt(1.0 - dist_calc)));

    dist_calc *= 6371000.0; //Converting to meters
    Serial.println(dist_calc);
    return dist_calc;
}

void getNewDataFromRPM(void)
{
    if (Serial1.available() > 0 && Serial1.read() == 'R')
    {
        rpm = Serial1.readStringUntil('@').toInt();
    }
}

void getNewDataFromWifi(void)
{
    if (Serial3.available() > 0 && Serial3.read() == 'X')
    {
        lastWords = Serial3.readStringUntil('@');
        Serial3.print("gotyou!");
        //delay(500);
    }
}

void getNewDataFromGPS(void)
{
    if (processGPS())
    {
        Serial.print("#SV: ");
        Serial.print(pvt.numSV);
        Serial.print(" fixType: ");
        Serial.print(pvt.fixType);
        // Serial.print(" Date:");
        // Serial.print(pvt.year);
        // Serial.print("/");
        // Serial.print(pvt.month);
        // Serial.print("/");
        // Serial.print(pvt.day);
        // Serial.print(" ");
        // Serial.print(pvt.hour);
        // Serial.print(":");
        // Serial.print(pvt.minute);
        // Serial.print(":");
        // Serial.print(pvt.second);
        Serial.print(" lat/lon: ");
        Serial.print(pvt.lat / 10000000.0f);
        Serial.print(",");
        Serial.print(pvt.lon / 10000000.0f);
        Serial.print(" gSpeed: ");
        Serial.print(pvt.gSpeed / 1000.0f);
        Serial.print(" heading: ");
        Serial.print(pvt.headMot / 100000.0f);
        Serial.print(" hAcc: ");
        Serial.print(pvt.hAcc / 1000.0f);
        Serial.println();
    }
}

void calcChecksum(unsigned char *CK)
{
    memset(CK, 0, 2);
    for (int i = 0; i < (int)sizeof(NAV_PVT); i++)
    {
        CK[0] += ((unsigned char *)(&pvt))[i];
        CK[1] += CK[0];
    }
}

bool processGPS()
{
    static int fpos = 0;
    static unsigned char checksum[2];
    const int payloadSize = sizeof(NAV_PVT);

    while (Serial2.available())
    {
        byte c = Serial2.read();
        if (fpos < 2)
        {
            if (c == UBX_HEADER[fpos])
                fpos++;
            else
                fpos = 0;
        }
        else
        {
            if ((fpos - 2) < payloadSize)
                ((unsigned char *)(&pvt))[fpos - 2] = c;
            fpos++;
            if (fpos == (payloadSize + 2))
                calcChecksum(checksum);
            else if (fpos == (payloadSize + 3))
            {
                if (c != checksum[0])
                    fpos = 0;
            }
            else if (fpos == (payloadSize + 4))
            {
                fpos = 0;
                if (c == checksum[1])
                    return true;
            }
            else if (fpos > (payloadSize + 4))
                fpos = 0;
        }
    }
    return false;
}

void display2digits(int number)
{
    if (number >= 0 && number < 10)
    {
        u8g2.print(0);
    }
    u8g2.print(number);
}

void bootbmp(void)
{
    static const unsigned char boot_bits[] U8X8_PROGMEM = 
    {
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
    delay(2000);
};

void bmx055Setup()
{
  // Initialise I2C communication as MASTER
  Wire.begin();

  // Start I2C Transmission
  Wire.beginTransmission(Addr_Accl);
  // Select PMU_Range register
  Wire.write(0x0F);
  // Range = +/- 2g
  Wire.write(0x03);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr_Accl);
  // Select PMU_BW register
  Wire.write(0x10);
  // Bandwidth = 7.81 Hz
  Wire.write(0x08);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr_Accl);
  // Select PMU_LPW register
  Wire.write(0x11);
  // Normal mode, Sleep duration = 0.5ms
  Wire.write(0x00);
  // Stop I2C Transmission on the device
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr_Gyro);
  // Select Range register
  Wire.write(0x0F);
  // Full scale = +/- 125 degree/s
  Wire.write(0x04);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr_Gyro);
  // Select Bandwidth register
  Wire.write(0x10);
  // ODR = 100 Hz
  Wire.write(0x07);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr_Gyro);
  // Select LPM1 register
  Wire.write(0x11);
  // Normal mode, Sleep duration = 2ms
  Wire.write(0x00);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr_Mag);
  // Select Mag register
  Wire.write(0x4B);
  // Soft reset
  Wire.write(0x83);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr_Mag);
  // Select Mag register
  Wire.write(0x4C);
  // Normal Mode, ODR = 10 Hz
  Wire.write(0x00);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr_Mag);
  // Select Mag register
  Wire.write(0x4E);
  // X, Y, Z-Axis enabled
  Wire.write(0x84);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr_Mag);
  // Select Mag register
  Wire.write(0x51);
  // No. of Repetitions for X-Y Axis = 9
  Wire.write(0x04);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr_Mag);
  // Select Mag register
  Wire.write(0x52);
  // No. of Repetitions for Z-Axis = 15
  Wire.write(0x0F);
  // Stop I2C Transmission
  Wire.endTransmission();
  delay(300);
}

void getNewDataFromBMX()
{
    unsigned int data[6];

    for (int i = 0; i < 6; i++)
    {
        // Start I2C Transmission
        Wire.beginTransmission(Addr_Accl);
        // Select data register
        Wire.write((2 + i));
        // Stop I2C Transmission
        Wire.endTransmission();

        // Request 1 byte of data
        Wire.requestFrom(Addr_Accl, 1);

        // Wire.requestFrom(Addr_Accl, 1,(2 + i), 1);
        //Where the parameters are, in this order,
        //the slave address,
        //the number of consecutive registers to read,
        //the first register to read,
        //the length of the data (in bytes) and the stop signal at the end (if true).






        // Read 6 bytes of data
        // xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
        if (Wire.available() == 1)
        {
            data[i] = Wire.read();
        }
    }

    // Convert the data to 12-bits
    int xAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
    if (xAccl > 2047)
    {
        xAccl -= 4096;
    }
    int yAccl = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
    if (yAccl > 2047)
    {
        yAccl -= 4096;
    }
    int zAccl = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
    if (zAccl > 2047)
    {
        zAccl -= 4096;
    }

    for (int i = 0; i < 6; i++)
    {
        // Start I2C Transmission
        Wire.beginTransmission(Addr_Gyro);
        // Select data register
        Wire.write((2 + i));
        // Stop I2C Transmission
        Wire.endTransmission();

        // Request 1 byte of data
        Wire.requestFrom(Addr_Gyro, 1);

        // Read 6 bytes of data
        // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
        if (Wire.available() == 1)
        {
            data[i] = Wire.read();
        }
    }

    // Convert the data
    int xGyro = (data[1] * 256) + data[0];
    if (xGyro > 32767)
    {
        xGyro -= 65536;
    }
    int yGyro = (data[3] * 256) + data[2];
    if (yGyro > 32767)
    {
        yGyro -= 65536;
    }
    int zGyro = (data[5] * 256) + data[4];
    if (zGyro > 32767)
    {
        zGyro -= 65536;
    }

    for (int i = 0; i < 6; i++)
    {
        // Start I2C Transmission
        Wire.beginTransmission(Addr_Mag);
        // Select data register
        Wire.write((66 + i));
        // Stop I2C Transmission
        Wire.endTransmission();

        // Request 1 byte of data
        Wire.requestFrom(Addr_Mag, 1);

        // Read 6 bytes of data
        // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
        if (Wire.available() == 1)
        {
            data[i] = Wire.read();
        }
    }

    // Convert the data
    int xMag = ((data[1] * 256) + (data[0] & 0xF8)) / 8;
    if (xMag > 4095)
    {
        xMag -= 8192;
    }
    int yMag = ((data[3] * 256) + (data[2] & 0xF8)) / 8;
    if (yMag > 4095)
    {
        yMag -= 8192;
    }
    int zMag = ((data[5] * 256) + (data[4] & 0xFE)) / 2;
    if (zMag > 16383)
    {
        zMag -= 32768;
    }

    // Output data to Serial monitor
    //  Serial.print("Acceleration in X/Y/Z-Axis : ");
    //   Serial.print(xAccl);  Serial.print("\t");  Serial.print(yAccl);  Serial.print("\t");  Serial.println(zAccl);
    //  Serial.print("X-Axis of rotation : ");
    //   Serial.print(xGyro);  Serial.print("\t");  Serial.print(yGyro);  Serial.print("\t");  Serial.print(zGyro);  Serial.print("\t");
    //  Serial.print("Magnetic field in XYZ-Axis : ");
    //   Serial.print(xMag);  Serial.print("\t");  Serial.print(yMag);  Serial.print("\t");  Serial.println(zMag);
    GForceX = xAccl;
    GForceY = yAccl;
}
