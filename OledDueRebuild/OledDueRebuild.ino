//2018.6.12.1
#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <DueTimer.h>
#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <include/twi.h>
#include <ResponsiveAnalogRead.h>
#include <InterruptFreqMeasure.h>

//Serial0.debug; 
//Serial1.rpm;
//Serial2.GPS;
//Serial3.Wireless;

//Pin2 rear  speed sonsor
//Pin3 Front speed sonsor

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

//temp starts here

#define ADDR      0x5A

//EEPROM 32x16
#define TO_MAX    0x00
#define TO_MIN    0x01
#define PWM_CTRL  0x02

//RAM 32x16
#define RAW_IR_1  0x04
#define RAW_IR_2  0x05
#define TA        0x06
#define TOBJ_1    0x07
#define TOBJ_2    0x08

#define SYNC_PIN  2

static const uint32_t TWI_CLOCK = 100000;
static const uint32_t RECV_TIMEOUT = 100000;
static const uint32_t XMIT_TIMEOUT = 100000;

Twi *pTwi = WIRE_INTERFACE;

static void Wire_Init(void) {
  pmc_enable_periph_clk(WIRE_INTERFACE_ID);
  PIO_Configure(
  g_APinDescription[PIN_WIRE_SDA].pPort,
  g_APinDescription[PIN_WIRE_SDA].ulPinType,
  g_APinDescription[PIN_WIRE_SDA].ulPin,
  g_APinDescription[PIN_WIRE_SDA].ulPinConfiguration);
  PIO_Configure(
  g_APinDescription[PIN_WIRE_SCL].pPort,
  g_APinDescription[PIN_WIRE_SCL].ulPinType,
  g_APinDescription[PIN_WIRE_SCL].ulPin,
  g_APinDescription[PIN_WIRE_SCL].ulPinConfiguration);

  NVIC_DisableIRQ(TWI1_IRQn);
  NVIC_ClearPendingIRQ(TWI1_IRQn);
  NVIC_SetPriority(TWI1_IRQn, 0);
  NVIC_EnableIRQ(TWI1_IRQn);
}

static void Wire1_Init(void) {
  	pmc_enable_periph_clk(WIRE1_INTERFACE_ID);
	PIO_Configure(
			g_APinDescription[PIN_WIRE1_SDA].pPort,
			g_APinDescription[PIN_WIRE1_SDA].ulPinType,
			g_APinDescription[PIN_WIRE1_SDA].ulPin,
			g_APinDescription[PIN_WIRE1_SDA].ulPinConfiguration);
	PIO_Configure(
			g_APinDescription[PIN_WIRE1_SCL].pPort,
			g_APinDescription[PIN_WIRE1_SCL].ulPinType,
			g_APinDescription[PIN_WIRE1_SCL].ulPin,
			g_APinDescription[PIN_WIRE1_SCL].ulPinConfiguration);

	NVIC_DisableIRQ(TWI0_IRQn);
	NVIC_ClearPendingIRQ(TWI0_IRQn);
	NVIC_SetPriority(TWI0_IRQn, 0);
	NVIC_EnableIRQ(TWI0_IRQn);
}
//temp ends here


bool fastboot=0;
int rpm = 0;
int rpm_last = 0;
int rpm_same = 0;
float spd_hz = 0;
int spd = 0;
int temp = 0;

int GearRatio = 0; //0.43-3.0
int TimeH = 3;
int TimeMM = 59;
int TimeSS = 59;
int GForceX = 0;
int GForceY = 0;
int GForceXScreen = GCenterX - 1;
int GForceYScreen = GCenterY - 1;
int GearRatioCube = 0;
float Volt = 3.30;
long BoostStartTime=0;
int BootedTime=0;
int BootDistance=0;
int BootedDistance=0;

ResponsiveAnalogRead analog(A0, true);
unsigned long timebetweenRear = 0;
unsigned long timelastRear = 0;
unsigned long timenowRear = 0;

unsigned long timebetweenFront = 0;
unsigned long timelastFront = 0;
unsigned long timenowFront = 0;

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

void setup(void)
{
    u8g2.begin(); 
    bootbmp();
    u8g2.sendBuffer();
    Serial.begin(115200);  //debug
    Serial1.begin(115200); //rpm
    Serial2.begin(115200); //GPS
    Serial3.begin(9600);   //Wireless

Serial.print("01");
    
    bmx055Setup();

Serial.print("02");
    //time setup start here
    pinMode(SYNC_PIN, OUTPUT);
Serial.print("03");
    digitalWrite(SYNC_PIN, LOW);

Serial.print("04");
    Wire_Init();
Serial.print("05");
    // Disable PDC channel
    pTwi->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
Serial.print("06");
    TWI_ConfigureMaster(pTwi, TWI_CLOCK, VARIANT_MCK);
    //time setup ends here

Serial.print("07");

    pinMode(2, INPUT); //Pin2 rear  speed sonsor
    pinMode(3, INPUT); //Pin3 Front speed sonsor
    Serial.begin(115200);
    attachInterrupt(2, countnowRear, RISING);
    attachInterrupt(3, countnowFront, RISING);

    u8g2.clearBuffer();
    u8g2.sendBuffer();
}

void loop(void)
{
    u8g2.clearBuffer();
    //
    //GetNewData
    spdRear = 6319879.2 / (timebetweenRear * NumPerCircleRear);
    spdFront = 6319879.2 / (timebetweenFront * NumPerCircleFront);

    getNewDataFromRPM();
    getNewDataFromWifi();
    getNewDataFromBMX();

    GForceXScreen = GForceX / 90 + GCenterX - 1;
    GForceYScreen = GForceY / 90 + GCenterY - 1;

    temp = getNewDataFromTemp();

    GearRatio = spd * 0.17522; //#define GearRatioConstant = 0.17522;
    GearRatio = GearRatio / rpm;

    analog.update();
    Volt = analog.getValue();
    Volt = Volt / 1024;
    Volt = Volt * 6.6;

    getNewDataFromGPS();

    // Serial3.print("R");Serial3.print(rpm);Serial3.print("@");
     Serial.print("R");Serial.print(rpm);Serial.print("@");
     Serial.print(" Rsame");Serial.print(rpm_same);Serial.print(" Rlast");Serial.print(rpm_last);

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
    rpm_last=rpm;
     Serial.print("Ra");Serial.print(rpm);Serial.print("@");
     Serial.print(" Rsamea");Serial.print(rpm_same);Serial.print(" Rlasta");Serial.print(rpm_last);

    if (RTC.read(tm))
    {
        TimeH = tm.Hour;
        TimeMM = tm.Minute;
        TimeSS = tm.Second;
    }

    //
    // frames
    //
    Serial.print("07");

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
    u8g2.drawFrame(67, 11, 4, 19);  //211-229.5-248
    //Speed Cube:
    u8g2.drawFrame(100, 0, 5, 47);  //211-229.5-248
    u8g2.drawBox(GForceXScreen, GForceYScreen, 3, 3); //GForceBox
Serial.print("07");
    //-----
    //DRAW DATAS:
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

    u8g2.setCursor(230, 53);
    if(GForceX>0)
    u8g2.print(0);
    u8g2.print(GForceX); //gforcex
    u8g2.setCursor(230, 61);
    if(GForceY>0)
    u8g2.print(0);
    u8g2.print(GForceY); //gforcey

    u8g2.setCursor(25, 64);
    if (rpm <= 5000)
    {
        u8g2.print(rpm); //rpm
    }
    else
    {
        u8g2.print("0000"); //rpm
    }

    u8g2.setFont(u8g2_font_logisoso18_tr);
    u8g2.setCursor(0, 52);
    display2digits(TimeH); //time
    u8g2.print(":");
    display2digits(TimeMM); //time
    u8g2.print(":");
    display2digits(TimeSS); //time
    
    u8g2.setCursor(157, 29);
    u8g2.print(temp);

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

    u8g2.setCursor(0, 29);
    u8g2.print(Volt); //battery
    u8g2.print("V"); //battery

    u8g2.setFont(u8g2_font_logisoso34_tn);
    u8g2.setCursor(105, 47); //speed
    u8g2.print(spd);
    u8g2.sendBuffer();
Serial.print("Freshed!");


}


void countnowRear()
{
    timenowRear = micros();
    if (timenowRear - timelastRear > 40000)
    {
        timebetweenRear = timenowRear - timelastRear;
        timelastRear = timenowRear;
    }
}

void countnowFront()
{
    timenowFront = micros();
    if (timenowFront - timelastFront > 40000)
    {
        timebetweenFront = timenowFront - timelastFront;
        timelastFront = timenowFront;
    }
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

int getNewDataFromRPM(void)
{
    if (Serial1.available() > 0 && Serial1.read() == 'R')
    {
        rpm = Serial1.parseInt();
    }
}

int getNewDataFromWifi(void)
{
    if (Serial3.available() > 0 && Serial3.read() == 'X')
    {
        u8g2.setCursor(100, 64);
        u8g2.print(Serial3.readStringUntil('@'));
        Serial3.print(Serial3.readStringUntil('@'));
        Serial3.print("gotyou!");
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

  // Output data to serial monitor
  //  Serial.print("Acceleration in X/Y/Z-Axis : ");
//   Serial.print(xAccl);  Serial.print("\t");  Serial.print(yAccl);  Serial.print("\t");  Serial.println(zAccl); 
  //  Serial.print("X-Axis of rotation : ");
//   Serial.print(xGyro);  Serial.print("\t");  Serial.print(yGyro);  Serial.print("\t");  Serial.print(zGyro);  Serial.print("\t");
  //  Serial.print("Magnetic field in XYZ-Axis : ");
//   Serial.print(xMag);  Serial.print("\t");  Serial.print(yMag);  Serial.print("\t");  Serial.println(zMag);
GForceX=xAccl;
GForceY=yAccl;
}


float getNewDataFromTemp()
{
  uint16_t tempUK;
  float tempK;
  uint8_t hB, lB, pec;

  digitalWrite(SYNC_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SYNC_PIN, LOW);

  TWI_StartRead(pTwi, ADDR, TOBJ_1, 1);

  lB = readByte();
  hB = readByte();

  //last read
  TWI_SendSTOPCondition(pTwi);
  pec = readByte();

  while (!TWI_TransferComplete(pTwi))
    ;
  //TWI_WaitTransferComplete(pTwi, RECV_TIMEOUT);

  tempUK = (hB << 8) | lB;
  if (tempUK & (1 << 16))
  {
    Serial.print("Error !");
    Serial.println(tempK);
  }
  else
  {
    tempK = ((float)tempUK * 2) / 100;
    Serial.print("gettempinC: ");
    Serial.println(tempK - 273.15);
    return(tempK - 273.15);
  }
}


//👇temp function starts here
uint8_t readByte() {
  //TWI_WaitByteReceived(pTwi, RECV_TIMEOUT);
  while (!TWI_ByteReceived(pTwi))
    ;
  return TWI_ReadByte(pTwi);
}

static inline bool TWI_WaitTransferComplete(Twi *_twi, uint32_t _timeout) {
  while (!TWI_TransferComplete(_twi)) {
    if (TWI_FailedAcknowledge(_twi))
      return false;
    if (--_timeout == 0)
      return false;
  }
  return true;
}

static inline bool TWI_WaitByteReceived(Twi *_twi, uint32_t _timeout) {
  while (!TWI_ByteReceived(_twi)) {
    if (TWI_FailedAcknowledge(_twi))
      return false;
    if (--_timeout == 0)
      return false;
  }
  return true;
}

static inline bool TWI_FailedAcknowledge(Twi *pTwi) {
  return pTwi->TWI_SR & TWI_SR_NACK;
}
//👆temp function ends here
