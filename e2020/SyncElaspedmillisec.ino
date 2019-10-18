#include <elapsedMillis.h>
#include <TimeLib.h>
#include <Wire.h>
#include <DS1307RTC.h> // a basic DS1307 library that returns time as a time_t

elapsedMillis millisecElapsed;

unsigned int interval = 1000;
bool ElaspedMillisecSynced = false;

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        ;                     // wait until Arduino Serial Monitor opens
    setSyncProvider(RTC.get); // the function to get the time from the RTC
    if (timeStatus() != timeSet)
        Serial.println("Unable to sync with the RTC");
    else
        Serial.println("RTC has set the system time");
}

void loop()
{
    if (ElaspedMillisecSynced==false)
    {
        SyncElaspedMillisec(now())
    }
    if (millisecElapsed>=999)
    {
        millisecElapsed=0;
    }
    


    
    if (timeStatus() == timeSet)
    {
        digitalClockDisplay();
    }
    else
    {
        Serial.println("The time has not been set.  Please run the Time");
        Serial.println("TimeRTCSet example, or DS1307RTC SetTime example.");
        Serial.println();
    }
}

void digitalClockDisplay()
{
    // digital clock display of the time
    Serial.print(hour());
    printDigits(minute());
    printDigits(second());
    Serial.print(" ");
    Serial.print(day());
    Serial.print(" ");
    Serial.print(month());
    Serial.print(" ");
    Serial.print(year());
    Serial.println();
}

void printDigits(int digits)
{
    // utility function for digital clock display: prints preceding colon and leading 0
    Serial.print(":");
    if (digits < 10)
        Serial.print('0');
    Serial.print(digits);
}

bool SyncElaspedMillisec(uint32_t oldnow)
{
    if (now() == oldnow + 1)
    {
        millisecElapsed = 0; // reset the counter to 0 so the counting starts over...
        return true;
        ElaspedMillisecSynced = true;
    }
}
