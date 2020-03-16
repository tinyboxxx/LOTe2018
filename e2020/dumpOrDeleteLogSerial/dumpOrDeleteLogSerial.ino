
#include <SD.h>
#include <SPI.h>

const int chipSelect = BUILTIN_SDCARD;

void setup()
{

  Serial.begin(9600);


  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect))
  {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  // ask user here

  Serial.println(F(
      "S to show all datalog\r\n"
      "C to clear all datalog\r\n"));



  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
}

void loop()
{
    while (!Serial.available())
  {
    yield();
  }
  int serialchar=Serial.read();
  if (serialchar == 'S')
  {
    showAllDataLog();
  }
    if (serialchar== 'C')
    {
      clearAllDataLog();
    }

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
