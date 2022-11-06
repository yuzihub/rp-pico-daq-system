/*
  SD card basic file example

 This example shows how to create and destroy an SD card file
 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4

 created   Nov 2010
 by David A. Mellis
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */
#include <SPI.h>
#include <SD.h>

File myFile;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  Serial.print(F("Initializing SD card..."));

  if (!SD.begin(4)) {
    Serial.println(F("initialization failed!"));
    return;
  }
  Serial.println(F("initialization done."));

  if (SD.exists("example.txt")) {
    Serial.println(F("example.txt exists."));
  } else {
    Serial.println(F("example.txt doesn't exist."));
  }

  // open a new file and immediately close it:
  Serial.println(F("Creating example.txt..."));
  myFile = SD.open("example.txt", FILE_WRITE);
  myFile.close();

  // Check to see if the file exists:
  if (SD.exists("example.txt")) {
    Serial.println(F("example.txt exists."));
  } else {
    Serial.println(F("example.txt doesn't exist."));
  }

  // delete the file:
  Serial.println(F("Removing example.txt..."));
  SD.remove("example.txt");

  if (SD.exists("example.txt")) {
    Serial.println(F("example.txt exists."));
  } else {
    Serial.println(F("example.txt doesn't exist."));
  }
}

void loop() {
}
