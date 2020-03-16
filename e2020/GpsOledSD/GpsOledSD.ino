#include <Arduino.h>
#include <U8g2lib.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <SD.h>

#define ss Serial2
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
const int chipSelect = BUILTIN_SDCARD;

U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/39, /* dc=*/2, /* reset=*/3); // Enable U8G2_16BIT in u8g2.h

void setup(void)
{
    u8g2.begin();
    Serial.begin(115200);
    ss.begin(GPSBaud);
    if (!SD.begin(chipSelect))
    {
        Serial.println("Card failed, or not present");
        // don't do anything more:
        return;
    }
    Serial.println("card initialized.");
    Serial.println(F("FullExample.ino"));
    Serial.println(F("An extensive example of many interesting TinyGPS++ features"));
    Serial.print(F("Testing TinyGPS++ library v. "));
    Serial.println(TinyGPSPlus::libraryVersion());
    Serial.println(F("by Mikal Hart"));
    Serial.println();
    Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
    Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
    Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));

    u8g2.setFont(u8g2_font_profont10_mf);                    // choose a suitable font
    u8g2.clearBuffer();                                      // clear the internal memory
    u8g2.drawStr(0, 10, "Latitude   Longitude   Fix  ");     // write something to the internal memory
    u8g2.drawStr(0, 40, "Date       Time     Alt    Speed"); // write something to the internal memory
    u8g2.sendBuffer();                                       // transfer internal memory to the display
}

void loop(void)
{
    static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

    u8g2.clearBuffer();                                      // clear the internal memory
    u8g2.drawStr(0, 8, "Latitude   Longitude   Sats  HDOP"); // write something to the internal memory
    if (gps.location.isValid())
    {
        
        u8g2.setCursor(0, 18);
        u8g2PrintFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
        u8g2.setCursor(55, 18);
        u8g2PrintFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
        u8g2.setCursor(115, 18);
        u8g2.print(gps.satellites.value());
    }
    else
    {
        u8g2.setCursor(0, 18);
        u8g2.print("---");
        u8g2.setCursor(55, 18);
        u8g2.print("---");
        u8g2.setCursor(115, 18);
        u8g2.print("---");
    }

    u8g2.setCursor(140, 18);
    if (gps.hdop.isValid())
        u8g2.print(gps.hdop.hdop());
    else
    {
        u8g2.print("---");
    }

    u8g2.drawStr(0, 40, "Date       Time     Alt    Speed    Course"); // write something to the internal memory
    u8g2.setCursor(0, 48);
    oledPrintDateTime(gps.date, gps.time);

    if (gps.location.isValid())
    {
        u8g2.setCursor(100, 48);
        u8g2.print(gps.altitude.meters());
        u8g2.setCursor(135, 48);
        u8g2.print(gps.speed.kmph());
    }
    else
    {
        u8g2.setCursor(100, 48);
        u8g2.print("---");
        u8g2.setCursor(135, 48);
        u8g2.print("---");
    }

    u8g2.setCursor(175, 48);
    if (gps.course.isValid())
        u8g2.print(gps.course.deg());
    else
    {
        u8g2.print("---");
    }

    u8g2.sendBuffer(); // transfer internal memory to the display

    //--- sd here

    // make a string for assembling the data to log:
    String dataStringToSD = "";

    // read three sensors and append to the string:

    if (gps.location.isValid())
    {
        char sz[32];
        sprintf(sz, "%02d/%02d/%02d", gps.date.month(), gps.date.day(), gps.date.year());
        dataStringToSD += String(sz);
        dataStringToSD += ",";
        sprintf(sz, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
        dataStringToSD += String(sz);
        dataStringToSD += ",";
        dataStringToSD += String(gps.satellites.value());
        dataStringToSD += ",";
        dataStringToSD += String(gps.hdop.hdop());
        dataStringToSD += ",";
        dataStringToSD += String(gps.location.lat(),11);
        dataStringToSD += ",";
        dataStringToSD += String(gps.location.lng(),11);
        dataStringToSD += ",";
        dataStringToSD += String(gps.altitude.meters());
        dataStringToSD += ",";
        dataStringToSD += String(gps.speed.kmph());
        dataStringToSD += ",";
        dataStringToSD += String(gps.course.deg());


        File dataFile = SD.open("datalog.txt", FILE_WRITE);
        if (dataFile)
        {
            dataFile.println(dataStringToSD);
            dataFile.close();
            // print to the serial port too:
        }
        // if the file isn't open, pop up an error:
        else
        {
            Serial.println("error opening datalog.txt");
        }
    }

    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.

    // if the file is available, write to it:

    //--- sd ends

    printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
    printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
    printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
    printInt(gps.location.age(), gps.location.isValid(), 5);
    printDateTime(gps.date, gps.time);
    printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
    printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
    printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
    printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

    unsigned long distanceKmToLondon =
        (unsigned long)TinyGPSPlus::distanceBetween(
            gps.location.lat(),
            gps.location.lng(),
            LONDON_LAT,
            LONDON_LON) /
        1000;
    printInt(distanceKmToLondon, gps.location.isValid(), 9);

    double courseToLondon =
        TinyGPSPlus::courseTo(
            gps.location.lat(),
            gps.location.lng(),
            LONDON_LAT,
            LONDON_LON);

    printFloat(courseToLondon, gps.location.isValid(), 7, 2);

    const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

    printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

    printInt(gps.charsProcessed(), true, 6);
    printInt(gps.sentencesWithFix(), true, 10);
    printInt(gps.failedChecksum(), true, 9);
    Serial.println();

    smartDelay(1000);

    if (millis() > 5000 && gps.charsProcessed() < 10)
        Serial.println(F("No GPS data received: check wiring"));


    int serialchar = Serial.read();
    if (serialchar == 'S')
    {
      showAllDataLog();
    }
    if (serialchar == 'C')
    {
      clearAllDataLog();
    }
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
    unsigned long start = millis();
    do
    {
        while (ss.available())
            gps.encode(ss.read());
    } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
    if (!valid)
    {
        while (len-- > 1)
            Serial.print('*');
        Serial.print(' ');
    }
    else
    {
        Serial.print(val, prec);
        int vi = abs((int)val);
        int flen = prec + (val < 0.0 ? 2 : 1); // . and -
        flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
        for (int i = flen; i < len; ++i)
            Serial.print(' ');
    }
    smartDelay(0);
}

static void u8g2PrintFloat(float val, bool valid, int len, int prec)
{
    if (!valid)
    {
        while (len-- > 1)
            u8g2.print('*');
        u8g2.print(' ');
    }
    else
    {
        u8g2.print(val, prec);
        int vi = abs((int)val);
        int flen = prec + (val < 0.0 ? 2 : 1); // . and -
        flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
        for (int i = flen; i < len; ++i)
            u8g2.print(' ');
    }
    smartDelay(0);
}


static void printInt(unsigned long val, bool valid, int len)
{
    char sz[32] = "*****************";
    if (valid)
        sprintf(sz, "%ld", val);
    sz[len] = 0;
    for (int i = strlen(sz); i < len; ++i)
        sz[i] = ' ';
    if (len > 0)
        sz[len - 1] = ' ';
    Serial.print(sz);
    smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
    if (!d.isValid())
    {
        Serial.print(F("********** "));
    }
    else
    {
        char sz[32];
        sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
        Serial.print(sz);
    }

    if (!t.isValid())
    {
        Serial.print(F("******** "));
    }
    else
    {
        char sz[32];
        sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
        Serial.print(sz);
    }

    printInt(d.age(), d.isValid(), 5);
    smartDelay(0);
}

static void printStr(const char *str, int len)
{
    int slen = strlen(str);
    for (int i = 0; i < len; ++i)
        Serial.print(i < slen ? str[i] : ' ');
    smartDelay(0);
}

static void oledPrintDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
    if (!d.isValid())
    {
        u8g2.print(F("********** "));
    }
    else
    {
        char sz[32];
        sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
        u8g2.print(sz);
    }

    if (!t.isValid())
    {
        u8g2.print(F("******** "));
    }
    else
    {
        char sz[32];
        sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
        u8g2.print(sz);
    }

    printInt(d.age(), d.isValid(), 5);
    smartDelay(0);
}
void showAllDataLog()
{
  File dataFile = SD.open("datalog.txt");

  // if the file is available, write to it:
  if (dataFile)
  {
    while (dataFile.available())
    {
      Serial.write(dataFile.read());
    }
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else
  {
    Serial.println("error opening datalog.txt");
  }
}

void clearAllDataLog()
{

  // delete the file:
  Serial.println("Removing datalog.txt...");
  SD.remove("datalog.txt");

  if (SD.exists("datalog.txt"))
  {
    Serial.println("datalog.txt exists.");
  }
  else
  {
    Serial.println("datalog.txt doesn't exist.");
  }
}
